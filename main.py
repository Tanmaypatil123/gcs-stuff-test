# rifm_app_planner_v2.py
# PyQt5 GCS with Mission Planner-style "Plan" tab:
# - Two maps (live map + planner map) kept in sync with the table
# - Click-to-add WPs on the Plan tab map
# - Read / Write / Clear missions with popups
# - High-contrast markers and indices

import sys, time, re, threading, datetime, math, sqlite3, queue
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Dict, Any, List, Tuple

from PyQt5 import QtCore, QtGui, QtWidgets, QtNetwork
from PyQt5.QtCore import Qt, pyqtSignal, QThread, QByteArray, QUrl, QRect, QTimer
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox, QPlainTextEdit, QMessageBox, QFileDialog,
    QCheckBox, QStackedWidget, QStatusBar, QToolBar, QTabWidget, QTableWidget,
    QTableWidgetItem, QSpinBox, QDoubleSpinBox
)
from PyQt5.QtGui import QPainter, QPixmap, QImage, QPen, QFont
from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkRequest, QNetworkReply, QSslSocket

# ---------------- Common helpers ----------------
PREFERRED_PORTS = ["COM5", "COM4"]
BAUDS = [115200, 57600, 38400, 9600]
TILE_SIZE = 256
CACHE_DIR = Path("./tile_cache").resolve()

def ensure_dir(p: Path): p.mkdir(parents=True, exist_ok=True)

def win_path(port: str) -> str:
    m = re.match(r"^COM(\d+)$", port.upper())
    if m and int(m.group(1)) >= 10:
        return r"\\.\%s" % port.upper()
    return port

try:
    import serial
    import serial.tools.list_ports
except Exception:
    serial = None

def list_candidate_ports() -> List[str]:
    if serial is None:
        return []
    all_ports = list(serial.tools.list_ports.comports())
    names = [p.device for p in all_ports]
    out = []
    for want in PREFERRED_PORTS:
        if want in names: out.append(want)
    for p in all_ports:
        if "standard serial over bluetooth" in (p.description or "").lower() and p.device not in out:
            out.append(p.device)
    for p in all_ports:
        if p.device not in out: out.append(p.device)
    return out

# ---------------- Waypoint model ----------------
CMD_TAKEOFF = 22
CMD_WAYPOINT = 16
CMD_LAND    = 21
CMD_RTL     = 20

CMD_CHOICES: List[Tuple[str,int]] = [
    ("TAKEOFF", CMD_TAKEOFF),
    ("WAYPOINT", CMD_WAYPOINT),
    ("LAND", CMD_LAND),
    ("RTL", CMD_RTL),
]

# MAVLink Mission ACK types for error decoding
MAV_MISSION_ACK_TYPES = {
    0: "MAV_MISSION_ACCEPTED - Mission accepted OK",
    1: "MAV_MISSION_ERROR - Generic error / not accepting mission commands",
    2: "MAV_MISSION_UNSUPPORTED_FRAME - Coordinate frame not supported",
    3: "MAV_MISSION_UNSUPPORTED - Command not supported",
    4: "MAV_MISSION_NO_SPACE - Mission item exceeds storage space",
    5: "MAV_MISSION_INVALID - One of the parameters invalid",
    6: "MAV_MISSION_INVALID_PARAM1 - param1 invalid",
    7: "MAV_MISSION_INVALID_PARAM2 - param2 invalid",
    8: "MAV_MISSION_INVALID_PARAM3 - param3 invalid",
    9: "MAV_MISSION_INVALID_PARAM4 - param4 invalid",
    10: "MAV_MISSION_INVALID_PARAM5_X - x/param5 (latitude) invalid",
    11: "MAV_MISSION_INVALID_PARAM6_Y - y/param6 (longitude) invalid",
    12: "MAV_MISSION_INVALID_PARAM7 - z/param7 (altitude) invalid",
    13: "MAV_MISSION_INVALID_SEQUENCE - Received waypoint out of sequence",
    14: "MAV_MISSION_DENIED - Not accepting any mission commands",
    15: "MAV_MISSION_OPERATION_CANCELLED - Mission operation cancelled",
}

def decode_mission_ack(ack_type: int) -> str:
    """Decode MAVLink mission ACK type to human-readable string"""
    return MAV_MISSION_ACK_TYPES.get(ack_type, f"UNKNOWN ACK TYPE: {ack_type}")

def get_cmd_name(cmd: int) -> str:
    """Get human-readable command name"""
    cmd_names = {22: "TAKEOFF", 16: "WAYPOINT", 21: "LAND", 20: "RTL"}
    return cmd_names.get(cmd, f"CMD_{cmd}")

@dataclass
class WP:
    cmd: int
    lat: float
    lon: float
    alt: float
    p1: float = 0.0
    p2: float = 0.0
    p3: float = 0.0
    p4: float = 0.0
    frame: int = 6  # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT

# ---------------- MAVLink worker ----------------
class MavWorker(QThread):
    log = pyqtSignal(str)
    status = pyqtSignal(dict)
    connected = pyqtSignal(str, int, dict)
    disconnected = pyqtSignal(str)

    mission_read_ok = pyqtSignal(list)   # list[WP]
    mission_write_ok = pyqtSignal()
    mission_clear_ok = pyqtSignal()
    mission_error = pyqtSignal(str)

    def __init__(self, demo_mode: bool = False):
        super().__init__()
        self.demo_mode = demo_mode
        self._stop = threading.Event()
        self._port: Optional[str] = None
        self._baud: Optional[int] = None
        self._auto: bool = False

        self._ser = None
        self._mav = None
        self._fc_sid = None
        self._fc_cid = None

        self._cmdq: "queue.Queue" = queue.Queue()

        # Mission upload state
        self._upl_active = False
        self._upl_items: List[WP] = []
        self._upl_start_time = 0
        self._upl_sent: set = set()  # Track which waypoints have been sent
        self._upl_retry_count = 0
        self._upl_max_retries = 3

        # Mission download state
        self._dl_active = False
        self._dl_expected = 0
        self._dl_got: List[WP] = []
        self._dl_start_time = 0
        self._dl_next_seq = 0  # Next waypoint to request

        # Mission clear state
        self._clear_pending = False

    # --------- public API from UI (thread-safe) ----------
    def configure(self, port: Optional[str], baud: Optional[int], auto: bool):
        self._port, self._baud, self._auto = port, baud, auto

    def stop(self):
        self._stop.set()

    def L(self, s: str): self.log.emit(s)

    def request_read_mission(self): self._cmdq.put(("read", None))
    def request_write_mission(self, items: List[WP]): self._cmdq.put(("write", items))
    def request_clear_mission(self): self._cmdq.put(("clear", None))

    # --------- demo stream (unchanged) ----------
    def run_demo(self):
        self.L("[demo] Simulated FC stream starting…")
        info = {"sysid": 1, "compid": 1, "type": "MAV_TYPE_QUADROTOR", "autopilot": "MAV_AUTOPILOT_ARDUPILOTMEGA",
                "mode": "STABILIZE", "armed": False}
        self.connected.emit("DEMO", 115200, info)
        t0 = time.time(); mode = "STABILIZE"; armed = False
        base_lat, base_lon = 18.5204, 73.8567
        while not self._stop.is_set():
            dt = time.time() - t0
            if int(dt) % 8 == 0:
                mode = "LOITER" if mode == "STABILIZE" else "STABILIZE"
                armed = not armed
                self.status.emit({"mode": mode, "armed": armed})
            lat = base_lat + 0.0005 * math.sin(dt/5.0)
            lon = base_lon + 0.0005 * math.cos(dt/5.0)
            self.status.emit({"gps": f"3D | Sats 12 | {lat:.7f}, {lon:.7f} | 550.0 m",
                              "_gps_lat": lat, "_gps_lon": lon, "_sats": 12, "_fix": "3D", "_alt": 550.0})
            self.status.emit({"battery": f"25.3 V | CPU 28.0%",
                              "_vb": 25.3, "_cpu": 28.0})
            self.status.emit({"hud": f"GS 0.0 m/s | Alt 30.0 m | Thr 0% | Climb +0.0 m/s",
                              "_gs": 0.0, "_vz": 0.0})
            self.status.emit({"attitude": f"roll +0.0° pitch +0.0° yaw 0.0°",
                              "_roll": 0.0, "_pitch": 0.0, "_yaw": 0.0})
            self.msleep(250)
        self.disconnected.emit("demo_stop")

    # --------- helpers ----------
    @staticmethod
    def _to_mission_int(seq: int, wp: WP, target_sys: int, target_comp: int, mavutil, mav):
        """Convert WP to MISSION_ITEM_INT message (compatible with older pymavlink)"""
        lat_i = int(round(wp.lat * 1e7))
        lon_i = int(round(wp.lon * 1e7))
        # Note: mission_type parameter removed for compatibility with older pymavlink versions
        return mavutil.mavlink.MAVLink_mission_item_int_message(
            target_system=target_sys,
            target_component=target_comp,
            seq=seq,
            frame=wp.frame,
            command=wp.cmd,
            current=1 if seq == 0 else 0,
            autocontinue=1,
            param1=float(wp.p1),
            param2=float(wp.p2),
            param3=float(wp.p3),
            param4=float(wp.p4),
            x=lat_i,
            y=lon_i,
            z=float(wp.alt)
        )

    @staticmethod
    def _from_mission_int(msg) -> WP:
        return WP(
            cmd=int(msg.command),
            lat=float((msg.x or 0)/1e7),
            lon=float((msg.y or 0)/1e7),
            alt=float(msg.z or 0.0),
            p1=float(msg.param1 or 0.0),
            p2=float(msg.param2 or 0.0),
            p3=float(msg.param3 or 0.0),
            p4=float(msg.param4 or 0.0),
            frame=int(msg.frame or 6),
        )

    # --------- Mission Operations (Robust Implementation) ----------
    def _do_mission_read(self):
        """Initiate mission download from flight controller with robust handling"""
        try:
            print(f"\n{'='*60}")
            print(f"[MAVLink] MISSION DOWNLOAD")
            print(f"  Target: sys={self._fc_sid}, comp={self._fc_cid}")
            print(f"{'='*60}")

            self.L("Requesting mission list from flight controller...")

            # Reset download state
            self._dl_active = True
            self._dl_got = []
            self._dl_expected = 0
            self._dl_next_seq = 0
            self._dl_start_time = time.time()

            # Request mission list - vehicle will respond with MISSION_COUNT
            self._mav.mission_request_list_send(self._fc_sid, self._fc_cid)

            print(f"[MAVLink] MISSION_REQUEST_LIST sent")
            print(f"[MAVLink] Waiting for MISSION_COUNT response...")
            print(f"[MAVLink] Timeout: 30 seconds")

        except Exception as e:
            print(f"[MAVLink] Mission read initiation failed: {e}")
            self._dl_active = False
            self.mission_error.emit(f"Read failed: {e}")

    def _do_mission_write(self, items: List[WP]):
        """Initiate mission upload to flight controller with robust handling"""
        if not items:
            print("[MAVLink] Write aborted: No waypoints provided")
            self.mission_error.emit("No waypoints to upload.")
            return

        try:
            print(f"\n{'='*60}")
            print(f"[MAVLink] MISSION UPLOAD")
            print(f"  Waypoints: {len(items)}")
            print(f"  Target: sys={self._fc_sid}, comp={self._fc_cid}")
            print(f"{'='*60}")

            # Display waypoints to upload
            print("\nWAYPOINTS TO UPLOAD:")
            print("-" * 70)
            for i, wp in enumerate(items):
                cmd_name = get_cmd_name(wp.cmd)
                print(f"  WP{i}: {cmd_name:10} | Frame={wp.frame} | "
                      f"Lat={wp.lat:.7f}, Lon={wp.lon:.7f}, Alt={wp.alt:.1f}m")
            print("-" * 70)

            self.L(f"Uploading {len(items)} waypoint(s) to flight controller...")

            # Reset upload state
            self._upl_active = True
            self._upl_items = items[:]
            self._upl_start_time = time.time()
            self._upl_sent = set()
            self._upl_retry_count = 0

            # Send mission count to start upload protocol
            self._mav.mission_count_send(self._fc_sid, self._fc_cid, len(items))

            print(f"\n[MAVLink] MISSION_COUNT sent: {len(items)} waypoints")
            print(f"[MAVLink] Waiting for vehicle to request waypoints...")
            print(f"[MAVLink] Upload timeout: 45 seconds")

        except Exception as e:
            print(f"[MAVLink] Mission write initiation failed: {e}")
            self._upl_active = False
            self.mission_error.emit(f"Write failed: {e}")

    def _do_mission_clear(self):
        """Clear mission from flight controller with verification"""
        try:
            print(f"\n{'='*60}")
            print(f"[MAVLink] MISSION CLEAR")
            print(f"  Target: sys={self._fc_sid}, comp={self._fc_cid}")
            print(f"{'='*60}")

            self.L("Clearing all mission items from flight controller...")

            # Set pending flag to track ACK
            self._clear_pending = True

            # Send clear command
            self._mav.mission_clear_all_send(self._fc_sid, self._fc_cid)
            print("[MAVLink] MISSION_CLEAR_ALL command sent")

            # Wait for ACK or timeout
            self.msleep(800)

            # Verify by requesting mission count
            print("[MAVLink] Verifying mission clear...")
            self._mav.mission_request_list_send(self._fc_sid, self._fc_cid)

            # Wait for count response
            verify_start = time.time()
            verified = False
            while (time.time() - verify_start) < 3.0:
                for b in self._ser.read(512):
                    msg = self._mav.parse_char(bytes([b]))
                    if not msg:
                        continue
                    if msg.get_type() == "MISSION_COUNT":
                        count = msg.count if hasattr(msg, 'count') else 0
                        print(f"[MAVLink] Verification: Vehicle reports {count} waypoints")
                        if count == 0:
                            print("[MAVLink] ✓ Mission cleared successfully!")
                            verified = True
                        else:
                            print(f"[MAVLink] Warning: Vehicle still has {count} waypoints")
                        break
                if verified or (time.time() - verify_start) > 3.0:
                    break
                self.msleep(50)

            if not verified:
                print("[MAVLink] Note: Could not verify clear (assuming success)")

            self.mission_clear_ok.emit()
            print("[MAVLink] Mission clear complete")

        except Exception as e:
            print(f"[MAVLink] Mission clear failed: {e}")
            self._clear_pending = False
            self.mission_error.emit(f"Clear failed: {e}")

    def _request_next_waypoint(self):
        """Request the next waypoint during mission download"""
        if self._dl_next_seq < self._dl_expected:
            print(f"[MAVLink] Requesting waypoint {self._dl_next_seq + 1}/{self._dl_expected}...")
            self._mav.mission_request_int_send(
                self._fc_sid, self._fc_cid,
                self._dl_next_seq
            )

    def _verify_mission_upload(self):
        """Verify uploaded mission by requesting count from vehicle"""
        try:
            print(f"\n[MAVLink] Verifying upload...")
            self._mav.mission_request_list_send(self._fc_sid, self._fc_cid)

            verify_start = time.time()
            while (time.time() - verify_start) < 3.0:
                for b in self._ser.read(512):
                    msg = self._mav.parse_char(bytes([b]))
                    if not msg:
                        continue
                    if msg.get_type() == "MISSION_COUNT":
                        count = msg.count if hasattr(msg, 'count') else 0
                        print(f"[MAVLink] Verification: Vehicle reports {count} waypoints")
                        if count == len(self._upl_items):
                            print("[MAVLink] ✓ VERIFICATION PASSED!")
                            return True
                        else:
                            print(f"[MAVLink] Warning: Count mismatch (expected {len(self._upl_items)})")
                            return False
                self.msleep(50)

            print("[MAVLink] Verification timeout")
            return False

        except Exception as e:
            print(f"[MAVLink] Verification error: {e}")
            return False

    # --------- main thread ----------
    def run(self):
        if self.demo_mode:
            self.run_demo(); return

        try:
            from pymavlink import mavutil
            from pymavlink.dialects.v20 import ardupilotmega as mavlink2
        except Exception as e:
            self.mission_error.emit(f"pymavlink import error: {e}")
            self.disconnected.emit("pymavlink_import_error")
            return
        if serial is None:
            self.mission_error.emit("pyserial not installed.")
            self.disconnected.emit("no_pyserial")
            return

        def enum_name(table: str, value: int) -> str:
            try: return mavutil.mavlink.enums[table][int(value)].name
            except Exception: return f"UNKNOWN({value})"

        def probe_once(port: str, baud: int, seconds: float = 2.0) -> Dict[str, Any]:
            out = dict(ok=False, total_msgs=0, heartbeats=0, fc_candidates=[], err=None)
            try:
                ser = serial.Serial(port=win_path(port), baudrate=baud, timeout=0.05, write_timeout=0.5,
                                    rtscts=False, dsrdtr=False, xonxoff=False)
            except Exception as e:
                out["err"] = f"open_error: {e}"; return out
            mav = mavlink2.MAVLink(None); mav.robust_parsing = True
            start = time.time()
            try:
                while time.time() - start < seconds:
                    for b in ser.read(512):
                        msg = mav.parse_char(bytes([b])); 
                        if not msg: continue
                        out["total_msgs"] += 1
                        if msg.get_type() == "HEARTBEAT":
                            sid, cid = msg.get_srcSystem(), msg.get_srcComponent()
                            if sid != 255 and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                                out["heartbeats"] += 1
                                out["fc_candidates"].append((sid, cid, msg.type, msg.autopilot))
            except Exception as e:
                out["err"] = f"read_error: {e}"
            finally:
                try: ser.close()
                except Exception: pass
            out["ok"] = out["total_msgs"] > 0
            return out

        def choose_best():
            ports = list_candidate_ports()
            if not ports:
                self.log.emit("No serial/Bluetooth ports found.")
                return None
            best_fc = best_any = None
            for port in ports:
                for baud in BAUDS:
                    r = probe_once(port, baud, 2.0)
                    if r["err"]: continue
                    fc_score = len(r["fc_candidates"])*100 + r["heartbeats"]*2 + r["total_msgs"]
                    any_score = r["total_msgs"]
                    if r["fc_candidates"] and (best_fc is None or fc_score > best_fc[0]): best_fc = (fc_score, port, baud)
                    if r["ok"] and (best_any is None or any_score > best_any[0]): best_any = (any_score, port, baud)
            if best_fc: return best_fc[1], best_fc[2]
            if best_any: return best_any[1], best_any[2]
            return None

        port, baud = self._port, self._baud
        if self._auto or not (port and baud):
            c = choose_best()
            if not c: self.disconnected.emit("no_mavlink"); return
            port, baud = c

        try:
            self._ser = serial.Serial(port=win_path(port), baudrate=baud, timeout=0.05, write_timeout=0.5,
                                      rtscts=False, dsrdtr=False, xonxoff=False)
        except Exception as e:
            self.disconnected.emit("open_error"); return

        self._mav = mavlink2.MAVLink(self._ser); self._mav.robust_parsing = True
        self._mav.srcSystem = 255; self._mav.srcComponent = 190

        # wait heartbeat
        start = time.time()
        while (time.time() - start < 12.0) and not self._stop.is_set():
            for b in self._ser.read(512):
                msg = self._mav.parse_char(bytes([b])); 
                if not msg: continue
                if msg.get_type() == "HEARTBEAT":
                    sid, cid = msg.get_srcSystem(), msg.get_srcComponent()
                    if sid != 255:
                        mode = mavutil.mode_string_v10(msg)
                        info = {"sysid": sid, "compid": cid,
                                "type": enum_name('MAV_TYPE', msg.type),
                                "autopilot": enum_name('MAV_AUTOPILOT', msg.autopilot),
                                "mode": mode,
                                "armed": bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}
                        self._fc_sid, self._fc_cid = sid, cid
                        self.connected.emit(port, baud, info)
                        break
            if self._fc_sid is not None: break
        if self._fc_sid is None:
            self.disconnected.emit("no_heartbeat"); return

        # main loop
        while not self._stop.is_set():
            # Check for mission operation timeouts (30 seconds)
            current_time = time.time()
            if self._dl_active and self._dl_start_time > 0 and (current_time - self._dl_start_time > 30):
                print(f"\n{'='*60}")
                print(f"[MAVLink] ✗ Mission download TIMEOUT (>30 seconds)")
                print(f"  Expected: {self._dl_expected} waypoints")
                print(f"  Received: {len(self._dl_got)} waypoints")
                print(f"{'='*60}\n")
                self._dl_active = False
                self._dl_start_time = 0
                self.mission_error.emit("Mission download timeout - no response from vehicle")

            if self._upl_active and self._upl_start_time > 0 and (current_time - self._upl_start_time > 30):
                print(f"\n{'='*60}")
                print(f"[MAVLink] ✗ Mission upload TIMEOUT (>30 seconds)")
                print(f"  Total waypoints: {len(self._upl_items)}")
                print(f"{'='*60}\n")
                self._upl_active = False
                self._upl_start_time = 0
                self._upl_items = []
                self.mission_error.emit("Mission upload timeout - no response from vehicle")

            for b in self._ser.read(512):
                msg = self._mav.parse_char(bytes([b]))
                if not msg: continue
                t = msg.get_type()

                # basic status
                if t == "HEARTBEAT":
                    from pymavlink import mavutil
                    mode = mavutil.mode_string_v10(msg)
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.status.emit({"mode": mode, "armed": armed})
                elif t == "SYS_STATUS":
                    vb = (msg.voltage_battery or 0) / 1000.0
                    cpu = msg.load/10.0
                    self.status.emit({"battery": f"{vb:.2f} V | CPU {cpu:.1f}%",
                                      "_vb": vb, "_cpu": cpu})
                elif t == "GPS_RAW_INT":
                    lat = msg.lat/1e7; lon = msg.lon/1e7; alt = msg.alt/1000.0
                    fixmap = {0:'NO_GPS',1:'NO_FIX',2:'2D',3:'3D',4:'DGPS',5:'RTK_FLOAT',6:'RTK_FIXED'}
                    self.status.emit({"gps": f"{fixmap.get(msg.fix_type,'?')} | Sats {msg.satellites_visible} | {lat:.7f}, {lon:.7f} | {alt:.1f} m",
                                      "_gps_lat": lat, "_gps_lon": lon, "_sats": msg.satellites_visible, "_fix": fixmap.get(msg.fix_type,'?'),
                                      "_alt": alt})
                elif t == "ATTITUDE":
                    from math import degrees
                    roll_deg = degrees(msg.roll)
                    pitch_deg = degrees(msg.pitch)
                    yaw_deg = degrees(msg.yaw)
                    self.status.emit({"attitude": f"roll {roll_deg:+.1f}° pitch {pitch_deg:+.1f}° yaw {yaw_deg:.1f}°",
                                      "_roll": roll_deg, "_pitch": pitch_deg, "_yaw": yaw_deg})
                elif t == "VFR_HUD":
                    # Ground speed, climb rate, throttle, altitude
                    gs = msg.groundspeed  # m/s
                    vz = msg.climb  # m/s
                    thr = msg.throttle  # %
                    alt_msl = msg.alt  # meters MSL
                    self.status.emit({"hud": f"GS {gs:.1f} m/s | Alt {alt_msl:.1f} m | Thr {thr}% | Climb {vz:+.1f} m/s",
                                      "_gs": gs, "_vz": vz, "_throttle": thr, "_alt_msl": alt_msl})

                # mission download - MISSION_COUNT response
                elif t == "MISSION_COUNT" and self._dl_active:
                    self._dl_expected = int(msg.count or 0)
                    self._dl_got = []
                    self._dl_next_seq = 0
                    print(f"[MAVLink] ✓ Vehicle responded: {self._dl_expected} waypoint(s) in mission")

                    if self._dl_expected == 0:
                        print(f"[MAVLink] Mission is empty - no waypoints to download")
                        self._dl_active = False
                        self._dl_start_time = 0
                        self.mission_read_ok.emit([])
                    else:
                        print(f"[MAVLink] Starting waypoint download...")
                        self.L(f"Vehicle has {self._dl_expected} waypoint(s), downloading...")
                        # Actively request first waypoint
                        self._request_next_waypoint()

                # Handle both MISSION_ITEM_INT and legacy MISSION_ITEM
                elif (t == "MISSION_ITEM_INT" or t == "MISSION_ITEM") and self._dl_active:
                    # Handle legacy format conversion
                    if t == "MISSION_ITEM":
                        # Legacy MISSION_ITEM uses float lat/lon
                        wp = WP(
                            cmd=int(msg.command),
                            lat=float(msg.x or 0),
                            lon=float(msg.y or 0),
                            alt=float(msg.z or 0.0),
                            p1=float(msg.param1 or 0.0),
                            p2=float(msg.param2 or 0.0),
                            p3=float(msg.param3 or 0.0),
                            p4=float(msg.param4 or 0.0),
                            frame=int(msg.frame or 6),
                        )
                        print(f"[MAVLink] Received LEGACY MISSION_ITEM (converted)")
                    else:
                        wp = self._from_mission_int(msg)

                    seq = len(self._dl_got)
                    self._dl_got.append(wp)
                    cmd_name = get_cmd_name(wp.cmd)
                    print(f"[MAVLink] Downloaded WP{seq}: {cmd_name} | "
                          f"Lat={wp.lat:.7f}, Lon={wp.lon:.7f}, Alt={wp.alt:.1f}m "
                          f"[{len(self._dl_got)}/{self._dl_expected}]")

                    if len(self._dl_got) == self._dl_expected:
                        # All waypoints received - send ACK and complete
                        self._mav.mission_ack_send(self._fc_sid, self._fc_cid, 0)
                        self._dl_active = False
                        self._dl_start_time = 0
                        print(f"\n{'='*60}")
                        print(f"[MAVLink] ✓ Mission download COMPLETE: {len(self._dl_got)} waypoint(s)")
                        print(f"{'='*60}\n")
                        self.L(f"Mission download complete: {len(self._dl_got)} waypoint(s)")
                        self.mission_read_ok.emit(self._dl_got[:])
                    else:
                        # Request next waypoint
                        self._dl_next_seq = len(self._dl_got)
                        self._request_next_waypoint()

                # mission upload - handle BOTH MISSION_REQUEST_INT and legacy MISSION_REQUEST
                elif (t == "MISSION_REQUEST_INT" or t == "MISSION_REQUEST") and self._upl_active:
                    seq = int(msg.seq)
                    request_type = "legacy" if t == "MISSION_REQUEST" else "INT"
                    print(f"[MAVLink] Vehicle requesting waypoint {seq+1}/{len(self._upl_items)} ({request_type})")

                    if seq >= len(self._upl_items):
                        print(f"[MAVLink] ERROR: Requested waypoint {seq} out of range!")
                        continue

                    try:
                        wp = self._upl_items[seq]
                        cmd_name = get_cmd_name(wp.cmd)
                        print(f"[MAVLink] Sending WP{seq}: {cmd_name} | "
                              f"Lat={wp.lat:.7f}, Lon={wp.lon:.7f}, Alt={wp.alt:.1f}m")

                        pkt = self._to_mission_int(seq, wp, self._fc_sid, self._fc_cid, mavutil, self._mav)
                        self._mav.send(pkt)
                        self._upl_sent.add(seq)
                        self.msleep(30)  # Small delay for Bluetooth reliability

                        print(f"[MAVLink] Waypoint {seq+1}/{len(self._upl_items)} sent ✓")

                    except Exception as e:
                        print(f"[MAVLink] ERROR sending waypoint {seq}: {e}")
                        continue

                elif t == "MISSION_ACK" and self._upl_active:
                    ack_type = msg.type if hasattr(msg, 'type') else 0
                    print(f"\n[MAVLink] MISSION_ACK received - Type: {ack_type}")
                    print(f"[MAVLink] Status: {decode_mission_ack(ack_type)}")

                    if ack_type == 0:  # MAV_MISSION_ACCEPTED
                        print(f"\n{'='*60}")
                        print(f"[MAVLink] ✓ MISSION UPLOAD SUCCESSFUL!")
                        print(f"[MAVLink] {len(self._upl_items)} waypoints saved to vehicle")
                        print(f"{'='*60}")

                        # Verify upload
                        self._verify_mission_upload()

                        self.L(f"Mission upload complete! {len(self._upl_items)} waypoints saved.")
                        self._upl_active = False
                        self._upl_items = []
                        self._upl_start_time = 0
                        self._upl_sent = set()
                        self.mission_write_ok.emit()
                    else:
                        print(f"\n{'='*60}")
                        print(f"[MAVLink] ✗ MISSION UPLOAD FAILED!")
                        print(f"[MAVLink] Error: {decode_mission_ack(ack_type)}")
                        print(f"{'='*60}")

                        # Provide debugging hints
                        if ack_type == 2:
                            print("\n[HINT] Frame type not supported.")
                            print("       Try using frame=3 (GLOBAL_RELATIVE_ALT) or frame=6 (GLOBAL_RELATIVE_ALT_INT)")
                        elif ack_type == 3:
                            print("\n[HINT] Command not supported by this vehicle type.")
                        elif ack_type == 14:
                            print("\n[HINT] Vehicle not accepting missions.")
                            print("       Check if vehicle is armed or in a mode that blocks uploads.")
                        elif ack_type in [6, 7, 8, 9, 10, 11, 12]:
                            print("\n[HINT] One or more waypoint parameters are invalid.")
                            print("       Check coordinates and altitude values.")

                        self.mission_error.emit(f"Mission rejected: {decode_mission_ack(ack_type)}")
                        self._upl_active = False
                        self._upl_items = []
                        self._upl_start_time = 0
                        self._upl_sent = set()

                # mission clear acknowledge
                elif t == "MISSION_ACK" and self._clear_pending:
                    ack_type = msg.type if hasattr(msg, 'type') else 0
                    self._clear_pending = False
                    if ack_type == 0:
                        print(f"[MAVLink] ✓ Mission CLEAR acknowledged by vehicle")
                    else:
                        print(f"[MAVLink] Mission clear ACK: {decode_mission_ack(ack_type)}")

                # general command acknowledge
                elif t == "COMMAND_ACK":
                    # Could inspect msg.command for specific commands
                    pass

            # process UI commands
            try:
                c, payload = self._cmdq.get_nowait()
            except queue.Empty:
                c = None

            if c == "read":
                self._do_mission_read()

            elif c == "write":
                items: List[WP] = payload or []
                self._do_mission_write(items)

            elif c == "clear":
                self._do_mission_clear()

            self.msleep(5)

        try:
            if self._ser: self._ser.close()
        except Exception: pass
        self.disconnected.emit("ok")

# ---------------- UI chips / connect page ----------------
def badge(text: str, ok: bool = True) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(
        "QLabel {"
        f"border: 1px solid {'#26734d' if ok else '#8a1f1f'};"
        f"background: {'#dff5ea' if ok else '#fbe3e3'};"
        "border-radius: 6px; padding: 4px 8px;}"
    )
    return lbl

class ConnectPage(QWidget):
    request_go_map = pyqtSignal()

    def __init__(self, worker: MavWorker):
        super().__init__()
        self.worker = worker
        self._build()
        self.worker.log.connect(self._append_log)
        self.worker.connected.connect(self._on_connected)
        self.worker.disconnected.connect(self._on_disconnected)
        self.worker.status.connect(self._on_status)

    def _build(self):
        main = QVBoxLayout(self)
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox(); self.baud_combo.addItems([str(b) for b in BAUDS])
        self.refresh_btn = QPushButton("Refresh Ports")
        self.auto_check = QCheckBox("Auto-scan")
        self.demo_check = QCheckBox("Demo Mode")
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect"); self.disconnect_btn.setEnabled(False)

        top = QHBoxLayout()
        top.addWidget(QLabel("Port:")); top.addWidget(self.port_combo, 1)
        top.addWidget(QLabel("Baud:")); top.addWidget(self.baud_combo)
        top.addWidget(self.refresh_btn); top.addWidget(self.auto_check)
        top.addWidget(self.demo_check); top.addWidget(self.connect_btn); top.addWidget(self.disconnect_btn)
        main.addLayout(top)

        grid = QGridLayout()
        def key(txt):
            lab = QLabel(txt); f = lab.font(); f.setBold(True); lab.setFont(f); return lab
        self.fc_chip = badge("Disconnected", ok=False)
        self.mode_chip = badge("Mode: —", ok=False)
        self.gps_val = QLabel("—"); self.bat_val = QLabel("—")
        self.hud_val = QLabel("—"); self.att_val = QLabel("—")
        grid.addWidget(key("Connection:"),0,0); grid.addWidget(self.fc_chip,0,1)
        grid.addWidget(key("Mode/Arm:"),1,0);   grid.addWidget(self.mode_chip,1,1)
        grid.addWidget(key("GPS:"),2,0);        grid.addWidget(self.gps_val,2,1)
        grid.addWidget(key("Battery/CPU:"),3,0);grid.addWidget(self.bat_val,3,1)
        grid.addWidget(key("HUD:"),4,0);        grid.addWidget(self.hud_val,4,1)
        grid.addWidget(key("Attitude:"),5,0);   grid.addWidget(self.att_val,5,1)
        main.addLayout(grid)

        self.log = QPlainTextEdit(); self.log.setReadOnly(True)
        self.log.setStyleSheet("QPlainTextEdit { background:#101010; color:#f0f0f0; font-family:Consolas,monospace; }")
        main.addWidget(self.log, 1)

        self.refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn.clicked.connect(self._connect)
        self.disconnect_btn.clicked.connect(self._disconnect)
        self._refresh_ports()

    def _refresh_ports(self):
        self.port_combo.clear()
        ports = list_candidate_ports()
        self.port_combo.addItems(ports if ports else ["(no ports found)"])

    def _append_log(self, s: str):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self.log.appendPlainText(f"[{ts}] {s}")
        self.log.moveCursor(self.log.textCursor().End)

    def _connect(self):
        if self.demo_check.isChecked():
            self.worker.demo_mode = True
            self.worker.configure(None, None, False)
        else:
            self.worker.demo_mode = False
            auto = self.auto_check.isChecked()
            if not auto:
                port = self.port_combo.currentText()
                if "(no ports found)" in port or not port:
                    QMessageBox.warning(self, "No Ports", "No serial ports available."); return
                try: baud = int(self.baud_combo.currentText())
                except Exception: QMessageBox.warning(self, "Bad Baud", "Select a valid baud."); return
                self.worker.configure(port, baud, False)
            else:
                self.worker.configure(None, None, True)

        self.worker.start()
        self.connect_btn.setEnabled(False); self.disconnect_btn.setEnabled(True)
        self.port_combo.setEnabled(False); self.baud_combo.setEnabled(True)
        self.auto_check.setEnabled(False); self.demo_check.setEnabled(False); self.refresh_btn.setEnabled(False)

    def _disconnect(self):
        if self.worker.isRunning():
            self.worker.stop(); self.worker.wait(2000)
        self.connect_btn.setEnabled(True); self.disconnect_btn.setEnabled(False)
        self.port_combo.setEnabled(True); self.baud_combo.setEnabled(True)
        self.auto_check.setEnabled(True); self.demo_check.setEnabled(True); self.refresh_btn.setEnabled(True)
        self.fc_chip.setText("Disconnected"); self.fc_chip.setStyleSheet(badge("", False).styleSheet())
        self.mode_chip.setText("Mode: —");    self.mode_chip.setStyleSheet(badge("", False).styleSheet())
        self.gps_val.setText("—"); self.bat_val.setText("—"); self.hud_val.setText("—"); self.att_val.setText("—")

    def _on_connected(self, port, baud, info):
        conn = f"{info.get('type','?')} / {info.get('autopilot','?')} (sys:{info.get('sysid')} comp:{info.get('compid')})"
        loc = f" on {port}@{baud}" if port != "DEMO" else " (demo)"
        print(f"\n{'='*60}")
        print(f"[Connection] CONNECTED to flight controller")
        print(f"  Type: {info.get('type','?')}")
        print(f"  Autopilot: {info.get('autopilot','?')}")
        print(f"  System ID: {info.get('sysid')}")
        print(f"  Component ID: {info.get('compid')}")
        print(f"  Port: {port}")
        print(f"  Baud Rate: {baud}")
        print(f"  Mode: {info.get('mode','?')}")
        print(f"  Armed: {info.get('armed')}")
        print(f"{'='*60}\n")
        self.fc_chip.setText(conn + loc); self.fc_chip.setStyleSheet(badge("", True).styleSheet())
        arm = "ARMED" if info.get("armed") else "DISARMED"
        self.mode_chip.setText(f"{info.get('mode','?')} | {arm}")
        self.mode_chip.setStyleSheet(badge("", info.get("armed")).styleSheet())
        self.request_go_map.emit()

    def _on_status(self, d: dict):
        if "mode" in d and "armed" in d:
            self.mode_chip.setText(f"{d['mode']} | {'ARMED' if d['armed'] else 'DISARMED'}")
        if "gps" in d: self.gps_val.setText(d["gps"])
        if "battery" in d: self.bat_val.setText(d["battery"])
        if "hud" in d: self.hud_val.setText(d["hud"])
        if "attitude" in d: self.att_val.setText(d["attitude"])

    def _on_disconnected(self, reason):
        self._append_log(f"Disconnected ({reason}).")
        self._disconnect()

# ---------------- Map / tiles ----------------
PROVIDERS = {
    "OSM Standard": {"template": "https://tile.openstreetmap.org/{z}/{x}/{y}.png", "max_zoom": 19},
    "ESRI Satellite": {"template": "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", "max_zoom": 19},
}
def wrap_x(x, z): return x % (1 << z)
def clamp_y(y, z): n = 1 << z; return max(0, min(n-1, y))

class DiskCache:
    def __init__(self, base: Path): self.base = base; ensure_dir(self.base)
    def _path(self, z, x, y): return self.base / str(z) / str(x) / f"{y}.png"
    def get(self, z, x, y):
        p = self._path(z, x, y)
        if p.is_file():
            pm = QPixmap(str(p))
            if not pm.isNull(): return pm
        return None
    def put(self, z, x, y, data: bytes):
        p = self._path(z, x, y); ensure_dir(p.parent)
        try: open(p, "wb").write(data)
        except Exception: pass

class MBTilesProvider:
    def __init__(self, path: Path):
        self.path = Path(path); self.conn=None; self.valid=False
        try:
            self.conn = sqlite3.connect(str(self.path))
            c = self.conn.cursor()
            c.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='tiles';")
            self.valid = bool(c.fetchone())
        except Exception: self.valid=False
    def close(self):
        try:
            if self.conn: self.conn.close()
        except Exception: pass
        self.conn=None; self.valid=False
    def get_tile(self, z,x,y_xyz):
        if not self.valid: return None
        try:
            n = 1<<z; y_tms = (n-1)-y_xyz
            c = self.conn.cursor()
            c.execute("SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",(z,x,y_tms))
            row=c.fetchone()
            if row and row[0]:
                img = QImage.fromData(QByteArray(row[0]))
                if not img.isNull():
                    pm = QPixmap.fromImage(img)
                    if not pm.isNull(): return pm
        except Exception: return None
        return None

class HttpTileProvider(QtCore.QObject):
    tile_available = pyqtSignal(int,int,int); tile_error = pyqtSignal(str)
    def __init__(self, template: str, cache: DiskCache):
        super().__init__()
        self.template=template; self.cache=cache
        self.net=QNetworkAccessManager(self); self.net.finished.connect(self._on_finished)
        self.inflight=set(); self.reply_key={}
    def set_template(self, t: str):
        self.template=t; self.inflight.clear(); self.reply_key.clear()
    def get_tile(self,z,x,y, allow_network: bool):
        pm=self.cache.get(z,x,y)
        if pm is not None: return pm
        if not allow_network: return None
        key=(z,x,y)
        if key in self.inflight: return None
        url = QUrl(self.template.format(z=z,x=x,y=y))
        if not url.isValid():
            self.tile_error.emit("Invalid tile URL"); return None
        req = QNetworkRequest(url)
        req.setRawHeader(b"User-Agent", b"rifm/1.0 (+https://www.openstreetmap.org)")
        reply=self.net.get(req); self.inflight.add(key); self.reply_key[reply]=key
        return None
    def _on_finished(self, reply: QNetworkReply):
        key=self.reply_key.pop(reply, None)
        if reply.error()==QNetworkReply.NoError and key:
            data=bytes(reply.readAll())
            if data:
                self.cache.put(*key,data); self.tile_available.emit(*key)
        elif key:
            self.tile_error.emit(f"Network error: {reply.errorString()}")
        if key and key in self.inflight: self.inflight.remove(key)
        reply.deleteLater()

# ---------------- AHRS + HUD ----------------
class AHRSWidget(QWidget):
    """Artificial Horizon / Attitude Indicator Widget"""
    def __init__(self):
        super().__init__()
        self.setFixedSize(260, 260)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.setAutoFillBackground(False)
        # Ensure widget is visible and painted
        self.setAttribute(Qt.WA_OpaquePaintEvent, False)

    def set_attitude(self, roll_deg: float, pitch_deg: float, yaw_deg: float):
        self.roll = float(roll_deg or 0.0)
        self.pitch = float(pitch_deg or 0.0)
        self.yaw = float(yaw_deg or 0.0)
        self.update()

    def paintEvent(self, event):
        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        radius = 100  # Main horizon circle radius

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)

        # Draw semi-transparent background panel
        p.setBrush(QtGui.QColor(0, 0, 0, 140))
        p.setPen(Qt.NoPen)
        p.drawRoundedRect(0, 0, w, h, 10, 10)

        # Create circular clipping region for the horizon
        clip_path = QtGui.QPainterPath()
        clip_path.addEllipse(cx - radius, cy - radius, radius * 2, radius * 2)
        p.setClipPath(clip_path)

        # Pixels per degree of pitch
        ppd = 2.5

        # Save state and apply roll rotation around center
        p.save()
        p.translate(cx, cy)
        p.rotate(-self.roll)

        # Calculate pitch offset
        y_off = ppd * self.pitch

        # Draw sky (blue) - large rect that extends beyond visible area
        sky_color = QtGui.QColor(70, 130, 200)
        p.fillRect(-radius - 50, -radius * 3 - int(y_off), (radius + 50) * 2, radius * 3, sky_color)

        # Draw ground (brown)
        ground_color = QtGui.QColor(130, 85, 35)
        p.fillRect(-radius - 50, -int(y_off), (radius + 50) * 2, radius * 3, ground_color)

        # Draw horizon line
        p.setPen(QPen(Qt.white, 2))
        p.drawLine(-radius - 20, -int(y_off), radius + 20, -int(y_off))

        # Draw pitch ladder lines
        p.setPen(QPen(Qt.white, 1))
        for deg in [-30, -20, -10, 10, 20, 30]:
            y_ladder = -int(y_off) - int(deg * ppd)
            line_width = 30 if abs(deg) == 10 else 20
            p.drawLine(-line_width, y_ladder, line_width, y_ladder)
            # Draw degree labels
            if abs(deg) <= 30:
                font = QFont("Arial", 7)
                p.setFont(font)
                p.drawText(line_width + 3, y_ladder + 4, str(abs(deg)))
                p.drawText(-line_width - 15, y_ladder + 4, str(abs(deg)))

        p.restore()

        # Remove clipping for overlay elements
        p.setClipping(False)

        # Draw bezel ring
        p.setPen(QPen(Qt.white, 2))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(cx - radius, cy - radius, radius * 2, radius * 2)

        # Draw fixed aircraft symbol (center reference)
        p.setPen(QPen(QtGui.QColor(255, 200, 0), 3))
        # Left wing
        p.drawLine(cx - 50, cy, cx - 20, cy)
        # Right wing
        p.drawLine(cx + 20, cy, cx + 50, cy)
        # Center dot
        p.setBrush(QtGui.QColor(255, 200, 0))
        p.drawEllipse(cx - 5, cy - 5, 10, 10)
        # Vertical tail indicator
        p.drawLine(cx, cy - 15, cx, cy - 5)

        # Draw roll indicator arc at top
        p.setPen(QPen(Qt.white, 1))
        arc_radius = radius + 10
        # Draw the arc
        p.drawArc(cx - arc_radius, cy - arc_radius, arc_radius * 2, arc_radius * 2, 30 * 16, 120 * 16)

        # Draw roll tick marks
        for angle in [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]:
            rad = math.radians(90 + angle)
            x1 = cx + int((arc_radius - 5) * math.cos(rad))
            y1 = cy - int((arc_radius - 5) * math.sin(rad))
            tick_len = 10 if angle % 30 == 0 else 5
            x2 = cx + int((arc_radius - 5 - tick_len) * math.cos(rad))
            y2 = cy - int((arc_radius - 5 - tick_len) * math.sin(rad))
            p.drawLine(x1, y1, x2, y2)

        # Draw roll pointer (triangle at top, rotates with roll)
        p.save()
        p.translate(cx, cy)
        p.rotate(-self.roll)
        pointer = QtGui.QPolygon([
            QtCore.QPoint(0, -arc_radius + 5),
            QtCore.QPoint(-6, -arc_radius + 15),
            QtCore.QPoint(6, -arc_radius + 15)
        ])
        p.setBrush(Qt.white)
        p.setPen(Qt.white)
        p.drawPolygon(pointer)
        p.restore()

        # Draw heading/yaw value at bottom
        p.setPen(Qt.white)
        font = QFont("Arial", 10, QFont.Bold)
        p.setFont(font)
        yaw_text = f"HDG {int(self.yaw) % 360}°"
        text_rect = p.fontMetrics().boundingRect(yaw_text)
        p.drawText(cx - text_rect.width() // 2, h - 15, yaw_text)

        # Draw pitch value
        pitch_text = f"PITCH {self.pitch:+.1f}°"
        p.drawText(10, h - 15, pitch_text)

        # Draw roll value
        roll_text = f"ROLL {self.roll:+.1f}°"
        text_rect = p.fontMetrics().boundingRect(roll_text)
        p.drawText(w - text_rect.width() - 10, h - 15, roll_text)

class HUDPanel(QWidget):
    """Head-Up Display Panel showing flight data"""
    def __init__(self):
        super().__init__()
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background: rgba(0,0,0,160); color: #e8e8e8; border-radius: 8px;")
        lay = QGridLayout(self)
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(4)

        def key(t):
            lbl = QLabel(t)
            f = lbl.font()
            f.setBold(True)
            lbl.setFont(f)
            lbl.setStyleSheet("color: #aaaaaa;")
            return lbl

        def val():
            lbl = QLabel("—")
            lbl.setStyleSheet("color: #ffffff; font-size: 12px;")
            return lbl

        # Mode and Armed status row with visual indicator
        self.mode = QLabel("—")
        self.mode.setStyleSheet("color: #ffffff; font-size: 13px; font-weight: bold;")

        self.armed_indicator = QLabel("DISARMED")
        self.armed_indicator.setAlignment(Qt.AlignCenter)
        self._set_armed_style(False)

        # Create other value labels
        self.gps = val()
        self.alt = QLabel("0.0 m")
        self.alt.setStyleSheet("color: #00ff00; font-size: 12px; font-weight: bold;")
        self.gs = val()
        self.vz = val()
        self.yaw = val()
        self.bat = val()

        # Layout
        row = 0
        lay.addWidget(key("Mode:"), row, 0)
        lay.addWidget(self.mode, row, 1)
        row += 1
        lay.addWidget(key("Status:"), row, 0)
        lay.addWidget(self.armed_indicator, row, 1)
        row += 1
        lay.addWidget(key("GPS:"), row, 0)
        lay.addWidget(self.gps, row, 1)
        row += 1
        lay.addWidget(key("Alt:"), row, 0)
        lay.addWidget(self.alt, row, 1)
        row += 1
        lay.addWidget(key("GS:"), row, 0)
        lay.addWidget(self.gs, row, 1)
        row += 1
        lay.addWidget(key("VZ:"), row, 0)
        lay.addWidget(self.vz, row, 1)
        row += 1
        lay.addWidget(key("Yaw:"), row, 0)
        lay.addWidget(self.yaw, row, 1)
        row += 1
        lay.addWidget(key("Batt:"), row, 0)
        lay.addWidget(self.bat, row, 1)

        self.setFixedWidth(220)

    def _set_armed_style(self, armed: bool):
        """Update armed indicator visual style"""
        if armed:
            self.armed_indicator.setText("ARMED")
            self.armed_indicator.setStyleSheet(
                "background: #cc0000; color: white; font-weight: bold; "
                "padding: 2px 8px; border-radius: 4px; font-size: 11px;"
            )
        else:
            self.armed_indicator.setText("DISARMED")
            self.armed_indicator.setStyleSheet(
                "background: #006600; color: white; font-weight: bold; "
                "padding: 2px 8px; border-radius: 4px; font-size: 11px;"
            )

    def set_mode_armed(self, mode: str, armed: bool):
        """Update mode and armed status"""
        self.mode.setText(mode if mode else "—")
        self._set_armed_style(armed)

# ---------------- Path Planning Algorithm ----------------
class PathPlanner:
    """
    Coverage Path Planning Algorithm (Lawnmower/Boustrophedon Pattern)

    This class generates a survey flight path inside a polygon boundary.
    The path consists of parallel lines (rows) that cover the entire area.

    Features:
    1. Takes a boundary polygon (yellow zone) as input
    2. Avoids red zones (no-fly areas)
    3. Respects user-defined start and end points
    4. Generates parallel survey lines at the specified row width
    5. Clips lines to stay inside the boundary and outside red zones
    6. Connects lines in a boustrophedon (back-and-forth) pattern
    """

    @staticmethod
    def log(msg: str):
        """Print log message with timestamp"""
        import datetime
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[{ts}] [PathPlanner] {msg}")

    @staticmethod
    def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate distance between two GPS coordinates in meters.
        Uses Haversine formula for great-circle distance.
        """
        R = 6371000  # Earth's radius in meters
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        return R * c

    @staticmethod
    def meters_to_degrees_lat(meters: float) -> float:
        """Convert meters to approximate degrees latitude"""
        return meters / 111320.0

    @staticmethod
    def meters_to_degrees_lon(meters: float, lat: float) -> float:
        """Convert meters to approximate degrees longitude at given latitude"""
        return meters / (111320.0 * math.cos(math.radians(lat)))

    @staticmethod
    def point_in_polygon(x: float, y: float, polygon: List[Tuple[float, float]]) -> bool:
        """
        Ray casting algorithm to check if point is inside polygon.
        Returns True if point (x, y) is inside the polygon.
        """
        n = len(polygon)
        if n < 3:
            return False
        inside = False

        j = n - 1
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[j]

            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i

        return inside

    @staticmethod
    def point_in_any_red_zone(lat: float, lon: float, red_zones: List[List[Tuple[float, float]]]) -> bool:
        """
        Check if a point is inside any of the red zones.
        Returns True if point is in a no-fly area.
        """
        for red_zone in red_zones:
            # Red zone uses (lat, lon), convert to (lon, lat) for x, y
            polygon = [(rz_lon, rz_lat) for rz_lat, rz_lon in red_zone]
            if PathPlanner.point_in_polygon(lon, lat, polygon):
                return True
        return False

    @staticmethod
    def line_polygon_intersections(y: float, polygon: List[Tuple[float, float]]) -> List[float]:
        """
        Find all x-coordinates where horizontal line y intersects the polygon edges.
        Used to clip survey lines to the polygon boundary.
        """
        intersections = []
        n = len(polygon)

        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]

            # Check if the edge crosses the horizontal line
            if (y1 <= y < y2) or (y2 <= y < y1):
                if y2 != y1:  # Avoid division by zero
                    x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                    intersections.append(x)

        return sorted(intersections)

    @staticmethod
    def clip_segment_around_red_zones(
        lat: float,
        lon_start: float,
        lon_end: float,
        red_zones: List[List[Tuple[float, float]]],
        step_size: float = 0.00001  # ~1 meter
    ) -> List[Tuple[float, float]]:
        """
        Clip a horizontal line segment to avoid red zones.
        Returns list of (lon_start, lon_end) segments that are outside red zones.
        """
        if not red_zones:
            return [(lon_start, lon_end)]

        # Ensure lon_start < lon_end
        if lon_start > lon_end:
            lon_start, lon_end = lon_end, lon_start

        valid_segments = []
        current_start = None
        lon = lon_start

        while lon <= lon_end:
            in_red = PathPlanner.point_in_any_red_zone(lat, lon, red_zones)

            if not in_red:
                if current_start is None:
                    current_start = lon
            else:
                if current_start is not None:
                    valid_segments.append((current_start, lon - step_size))
                    current_start = None

            lon += step_size

        # Close last segment
        if current_start is not None:
            valid_segments.append((current_start, lon_end))

        return valid_segments

    @staticmethod
    def generate_coverage_path(
        boundary: List[Tuple[float, float]],  # List of (lat, lon) tuples
        row_width_meters: float = 10.0,
        altitude: float = 30.0,
        angle_deg: float = 0.0,  # Survey angle (0 = North-South lines)
        red_zones: List[List[Tuple[float, float]]] = None,  # List of no-fly polygons
        start_point: Tuple[float, float] = None,  # (lat, lon) start point
        end_point: Tuple[float, float] = None,  # (lat, lon) end point
    ) -> Tuple[List[WP], dict]:
        """
        Generate a coverage path (lawnmower pattern) inside the boundary polygon.

        Args:
            boundary: List of (lat, lon) tuples defining the survey area (yellow zone)
            row_width_meters: Distance between parallel survey lines in meters
            altitude: Flight altitude in meters
            angle_deg: Survey line angle (0=N-S, 90=E-W)
            red_zones: List of polygons defining no-fly areas (red zones)
            start_point: Optional (lat, lon) where path should start
            end_point: Optional (lat, lon) where path should end

        Returns:
            Tuple of (waypoints list, statistics dict)
        """
        planner = PathPlanner
        planner.log("=" * 60)
        planner.log("COVERAGE PATH GENERATION STARTED")
        planner.log("=" * 60)

        if red_zones is None:
            red_zones = []

        if len(boundary) < 3:
            planner.log("ERROR: Boundary must have at least 3 points!")
            return [], {"error": "Boundary must have at least 3 points"}

        planner.log(f"Input Parameters:")
        planner.log(f"  - Boundary points: {len(boundary)}")
        planner.log(f"  - Row width: {row_width_meters} meters")
        planner.log(f"  - Altitude: {altitude} meters")
        planner.log(f"  - Survey angle: {angle_deg} degrees")
        planner.log(f"  - Red zones (no-fly areas): {len(red_zones)}")
        if start_point:
            planner.log(f"  - Start point: ({start_point[0]:.7f}, {start_point[1]:.7f})")
        if end_point:
            planner.log(f"  - End point: ({end_point[0]:.7f}, {end_point[1]:.7f})")

        # Log red zones info
        for i, rz in enumerate(red_zones):
            planner.log(f"  - Red Zone {i+1}: {len(rz)} points")

        # Extract coordinates
        lats = [p[0] for p in boundary]
        lons = [p[1] for p in boundary]

        # Calculate bounding box
        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)
        center_lat = (min_lat + max_lat) / 2
        center_lon = (min_lon + max_lon) / 2

        planner.log(f"Boundary Analysis:")
        planner.log(f"  - Latitude range: {min_lat:.7f} to {max_lat:.7f}")
        planner.log(f"  - Longitude range: {min_lon:.7f} to {max_lon:.7f}")
        planner.log(f"  - Center: ({center_lat:.7f}, {center_lon:.7f})")

        # Calculate area dimensions in meters
        height_m = planner.haversine_distance(min_lat, center_lon, max_lat, center_lon)
        width_m = planner.haversine_distance(center_lat, min_lon, center_lat, max_lon)

        planner.log(f"  - Approximate width: {width_m:.1f} meters")
        planner.log(f"  - Approximate height: {height_m:.1f} meters")

        # Convert row width to degrees
        row_width_lat = planner.meters_to_degrees_lat(row_width_meters)
        row_width_lon = planner.meters_to_degrees_lon(row_width_meters, center_lat)

        planner.log(f"Row Spacing Conversion:")
        planner.log(f"  - {row_width_meters}m = {row_width_lat:.8f} degrees latitude")
        planner.log(f"  - {row_width_meters}m = {row_width_lon:.8f} degrees longitude")

        # Create polygon for intersection tests (using lon, lat order for x, y)
        polygon = [(lon, lat) for lat, lon in boundary]

        # Generate all survey lines first
        all_lines = []  # List of (lat, [(lon1, lon2), ...]) for each line
        planner.log("Generating Survey Lines:")
        current_lat = min_lat + row_width_lat / 2  # Start half row-width from edge

        while current_lat <= max_lat:
            # Find intersections with boundary polygon at this latitude
            intersections = planner.line_polygon_intersections(current_lat, polygon)

            if len(intersections) >= 2:
                # Get boundary segments
                segments = []
                for i in range(0, len(intersections) - 1, 2):
                    x_start = intersections[i]
                    x_end = intersections[i + 1] if i + 1 < len(intersections) else intersections[i]
                    segments.append((x_start, x_end))

                # Clip each segment around red zones
                valid_segments = []
                for seg_start, seg_end in segments:
                    clipped = planner.clip_segment_around_red_zones(
                        current_lat, seg_start, seg_end, red_zones,
                        step_size=row_width_lon / 10  # Use 1/10 of row width for precision
                    )
                    valid_segments.extend(clipped)

                if valid_segments:
                    all_lines.append((current_lat, valid_segments))

            current_lat += row_width_lat

        planner.log(f"  - Total survey lines generated: {len(all_lines)}")

        # Determine path direction based on start point
        reverse_order = False
        if start_point and all_lines:
            # Check if start point is closer to first or last line
            first_line_lat = all_lines[0][0]
            last_line_lat = all_lines[-1][0]
            dist_to_first = abs(start_point[0] - first_line_lat)
            dist_to_last = abs(start_point[0] - last_line_lat)
            if dist_to_last < dist_to_first:
                reverse_order = True
                planner.log(f"  - Reversing line order (start point closer to last line)")

        if reverse_order:
            all_lines = all_lines[::-1]

        # Build waypoints with boustrophedon pattern
        waypoints = []
        total_lines = 0
        skipped_red = 0

        for line_idx, (lat, segments) in enumerate(all_lines):
            go_right = (line_idx % 2 == 0)

            # Determine start direction based on start point for first line
            if line_idx == 0 and start_point and segments:
                # Check which end of first segment is closer to start point
                first_seg = segments[0]
                last_seg = segments[-1]
                dist_to_left = abs(start_point[1] - first_seg[0])
                dist_to_right = abs(start_point[1] - last_seg[1])
                go_right = dist_to_right > dist_to_left

            for seg_start, seg_end in (segments if go_right else reversed(segments)):
                if go_right:
                    wp1_lon, wp2_lon = seg_start, seg_end
                else:
                    wp1_lon, wp2_lon = seg_end, seg_start

                # Verify waypoints are not in red zones
                if not planner.point_in_any_red_zone(lat, wp1_lon, red_zones):
                    waypoints.append(WP(cmd=CMD_WAYPOINT, lat=lat, lon=wp1_lon, alt=altitude))
                else:
                    skipped_red += 1

                if not planner.point_in_any_red_zone(lat, wp2_lon, red_zones):
                    waypoints.append(WP(cmd=CMD_WAYPOINT, lat=lat, lon=wp2_lon, alt=altitude))
                else:
                    skipped_red += 1

                total_lines += 1
                planner.log(f"  Line {total_lines}: Lat={lat:.7f}, Lon {wp1_lon:.7f} -> {wp2_lon:.7f}")

        if skipped_red > 0:
            planner.log(f"  - Skipped {skipped_red} points inside red zones")

        # Add start point as first waypoint if specified
        if start_point and waypoints:
            start_wp = WP(cmd=CMD_WAYPOINT, lat=start_point[0], lon=start_point[1], alt=altitude)
            waypoints.insert(0, start_wp)
            planner.log(f"  - Added START point as first waypoint")

        # Add end point as last waypoint if specified
        if end_point and waypoints:
            end_wp = WP(cmd=CMD_WAYPOINT, lat=end_point[0], lon=end_point[1], alt=altitude)
            waypoints.append(end_wp)
            planner.log(f"  - Added END point as last waypoint")

        # Calculate statistics
        total_distance = 0.0
        for i in range(len(waypoints) - 1):
            wp1, wp2 = waypoints[i], waypoints[i + 1]
            total_distance += planner.haversine_distance(wp1.lat, wp1.lon, wp2.lat, wp2.lon)

        # Estimate flight time (assuming 5 m/s cruise speed)
        flight_speed = 5.0  # m/s
        estimated_time = total_distance / flight_speed if total_distance > 0 else 0

        stats = {
            "total_waypoints": len(waypoints),
            "total_lines": total_lines,
            "total_distance_m": total_distance,
            "estimated_time_s": estimated_time,
            "row_width_m": row_width_meters,
            "area_width_m": width_m,
            "area_height_m": height_m,
            "red_zones_count": len(red_zones),
            "skipped_red_zone_points": skipped_red,
            "has_start_point": start_point is not None,
            "has_end_point": end_point is not None,
        }

        planner.log("=" * 60)
        planner.log("PATH GENERATION COMPLETE")
        planner.log("=" * 60)
        planner.log(f"Summary:")
        planner.log(f"  - Total waypoints: {len(waypoints)}")
        planner.log(f"  - Total survey lines: {total_lines}")
        planner.log(f"  - Total distance: {total_distance:.1f} meters ({total_distance/1000:.2f} km)")
        planner.log(f"  - Estimated flight time: {estimated_time:.0f} seconds ({estimated_time/60:.1f} minutes)")
        planner.log(f"  - Red zones avoided: {len(red_zones)}")
        if start_point:
            planner.log(f"  - Path starts at: ({start_point[0]:.7f}, {start_point[1]:.7f})")
        if end_point:
            planner.log(f"  - Path ends at: ({end_point[0]:.7f}, {end_point[1]:.7f})")
        planner.log("=" * 60)

        return waypoints, stats


# ---------------- Map Widget (now emits waypointsChanged) ----------------
class MapWidget(QWidget):
    userInteracted = pyqtSignal(int)
    waypointsChanged = pyqtSignal(list)  # emits List[WP] whenever WPs change on this map
    boundaryChanged = pyqtSignal(list)   # emits boundary polygon when changed
    redZonesChanged = pyqtSignal(list)   # emits red zones when changed
    startEndChanged = pyqtSignal(object, object)  # emits (start_point, end_point)

    def __init__(self, status_provider=lambda: None, show_overlays: bool = True):
        super().__init__()
        self.setMouseTracking(True)
        self.zoom=5; self.min_zoom=1; self.max_zoom=19
        self.center_lat=20.5937; self.center_lon=78.9629
        self.cache = DiskCache(CACHE_DIR)
        self.http = HttpTileProvider(PROVIDERS["ESRI Satellite"]["template"], self.cache)
        self.http.tile_available.connect(lambda *_: self.update())
        self.http.tile_error.connect(self._status)
        self.mbtiles=None
        self.offline=False
        self._drag=False; self._last=None
        self._statusbar = status_provider

        self._wps: List[WP] = []
        self.add_mode = False

        # Boundary polygon for path planning (yellow zone)
        self._boundary: List[Tuple[float, float]] = []  # List of (lat, lon)
        self.boundary_mode = False  # Mode for drawing boundary

        # Red zones - no-fly areas (list of polygons)
        self._red_zones: List[List[Tuple[float, float]]] = []  # List of polygons
        self._current_red_zone: List[Tuple[float, float]] = []  # Currently being drawn
        self.red_zone_mode = False  # Mode for drawing red zones

        # Start and End points for path planning
        self._start_point: Optional[Tuple[float, float]] = None  # (lat, lon)
        self._end_point: Optional[Tuple[float, float]] = None  # (lat, lon)
        self.start_point_mode = False  # Mode for setting start point
        self.end_point_mode = False  # Mode for setting end point

        # Generated path (green lines)
        self._generated_path: List[WP] = []

        # overlays optional on plan map
        self.hud = HUDPanel() if show_overlays else None
        self.ahrs = AHRSWidget() if show_overlays else None
        if self.hud:
            self.hud.setParent(self); self.hud.move(10, 10); self.hud.show()
            self.hud.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        if self.ahrs:
            self.ahrs.setParent(self)
            # Position AHRS to the right of HUD
            ahrs_x = (self.hud.width() + 20) if self.hud else 10
            self.ahrs.move(ahrs_x, 10)
            self.ahrs.show()
            self.ahrs.setAttribute(Qt.WA_TransparentForMouseEvents, True)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        if self.hud: self.hud.move(10, 10)
        if self.ahrs:
            # Keep AHRS to the right of HUD
            ahrs_x = (self.hud.width() + 20) if self.hud else 10
            self.ahrs.move(ahrs_x, 10)

    def set_offline(self, val: bool): self.offline=val; self.update()
    def set_provider(self, key: str):
        self.max_zoom = PROVIDERS[key]["max_zoom"]; self.http.set_template(PROVIDERS[key]["template"]); self.update()
    def recenter(self, lat: float, lon: float):
        self.center_lat, self.center_lon = float(lat), float(lon); self.update()

    def set_add_mode(self, enabled: bool): self.add_mode = enabled

    def set_boundary_mode(self, enabled: bool):
        """Enable/disable boundary drawing mode"""
        self.boundary_mode = enabled
        if enabled:
            self._disable_other_modes('boundary')
            print("[Map] Boundary drawing mode ENABLED - Click to add boundary points")
        else:
            print("[Map] Boundary drawing mode DISABLED")

    def set_red_zone_mode(self, enabled: bool):
        """Enable/disable red zone (no-fly area) drawing mode"""
        self.red_zone_mode = enabled
        if enabled:
            self._disable_other_modes('red_zone')
            self._current_red_zone = []  # Start new red zone
            print("[Map] RED ZONE drawing mode ENABLED - Click to add no-fly area points")
            print("[Map] Click at least 3 points, then finish the zone")
        else:
            print("[Map] RED ZONE drawing mode DISABLED")

    def set_start_point_mode(self, enabled: bool):
        """Enable/disable start point setting mode"""
        self.start_point_mode = enabled
        if enabled:
            self._disable_other_modes('start')
            print("[Map] START POINT mode ENABLED - Click on map to set start position")
        else:
            print("[Map] START POINT mode DISABLED")

    def set_end_point_mode(self, enabled: bool):
        """Enable/disable end point setting mode"""
        self.end_point_mode = enabled
        if enabled:
            self._disable_other_modes('end')
            print("[Map] END POINT mode ENABLED - Click on map to set end position")
        else:
            print("[Map] END POINT mode DISABLED")

    def _disable_other_modes(self, keep_mode: str):
        """Disable all modes except the specified one"""
        if keep_mode != 'boundary':
            self.boundary_mode = False
        if keep_mode != 'red_zone':
            self.red_zone_mode = False
        if keep_mode != 'start':
            self.start_point_mode = False
        if keep_mode != 'end':
            self.end_point_mode = False
        if keep_mode != 'add':
            self.add_mode = False

    def set_boundary(self, boundary: List[Tuple[float, float]]):
        """Set the boundary polygon"""
        self._boundary = boundary[:]
        self.update()

    def get_boundary(self) -> List[Tuple[float, float]]:
        """Get the current boundary polygon"""
        return self._boundary[:]

    def clear_boundary(self):
        """Clear the boundary polygon"""
        self._boundary = []
        self._generated_path = []
        print("[Map] Boundary cleared")
        self.update()

    # Red Zone methods
    def set_red_zones(self, red_zones: List[List[Tuple[float, float]]]):
        """Set all red zones"""
        self._red_zones = [rz[:] for rz in red_zones]
        self.update()

    def get_red_zones(self) -> List[List[Tuple[float, float]]]:
        """Get all red zones"""
        return [rz[:] for rz in self._red_zones]

    def add_red_zone(self, red_zone: List[Tuple[float, float]]):
        """Add a completed red zone polygon"""
        if len(red_zone) >= 3:
            self._red_zones.append(red_zone[:])
            print(f"[Map] Red zone added with {len(red_zone)} points. Total red zones: {len(self._red_zones)}")
            self.update()
            self.redZonesChanged.emit(self.get_red_zones())

    def finish_current_red_zone(self):
        """Finish drawing the current red zone and add it to the list"""
        if len(self._current_red_zone) >= 3:
            self.add_red_zone(self._current_red_zone)
            self._current_red_zone = []
            return True
        else:
            print(f"[Map] Cannot finish red zone: need at least 3 points (have {len(self._current_red_zone)})")
            return False

    def clear_red_zones(self):
        """Clear all red zones"""
        self._red_zones = []
        self._current_red_zone = []
        print("[Map] All red zones cleared")
        self.update()
        self.redZonesChanged.emit([])

    def undo_last_red_zone(self):
        """Remove the last added red zone"""
        if self._red_zones:
            self._red_zones.pop()
            print(f"[Map] Last red zone removed. Remaining: {len(self._red_zones)}")
            self.update()
            self.redZonesChanged.emit(self.get_red_zones())

    # Start/End point methods
    def set_start_point(self, point: Optional[Tuple[float, float]]):
        """Set the start point for path planning"""
        self._start_point = point
        if point:
            print(f"[Map] START point set: ({point[0]:.7f}, {point[1]:.7f})")
        else:
            print("[Map] START point cleared")
        self.update()
        self.startEndChanged.emit(self._start_point, self._end_point)

    def set_end_point(self, point: Optional[Tuple[float, float]]):
        """Set the end point for path planning"""
        self._end_point = point
        if point:
            print(f"[Map] END point set: ({point[0]:.7f}, {point[1]:.7f})")
        else:
            print("[Map] END point cleared")
        self.update()
        self.startEndChanged.emit(self._start_point, self._end_point)

    def get_start_point(self) -> Optional[Tuple[float, float]]:
        """Get the start point"""
        return self._start_point

    def get_end_point(self) -> Optional[Tuple[float, float]]:
        """Get the end point"""
        return self._end_point

    def clear_start_end_points(self):
        """Clear both start and end points"""
        self._start_point = None
        self._end_point = None
        print("[Map] Start and End points cleared")
        self.update()
        self.startEndChanged.emit(None, None)

    def set_generated_path(self, path: List[WP]):
        """Set the generated coverage path (green lines)"""
        self._generated_path = path[:]
        print(f"[Map] Generated path set with {len(path)} waypoints")
        self.update()

    def clear_generated_path(self):
        """Clear the generated path"""
        self._generated_path = []
        self.update()

    def set_waypoints(self, wps: List[WP]):
        self._wps = wps[:]
        self.update()

    def get_waypoints(self) -> List[WP]:
        return self._wps[:]

    # conversions
    def latlon_to_pixels(self, lat, lon, z):
        lat = max(min(lat,85.05112878),-85.05112878)
        n = 2**z
        x = (lon+180.0)/360.0*n
        lat_rad = math.radians(lat)
        y = (1.0 - math.log(math.tan(lat_rad)+1/math.cos(lat_rad))/math.pi)/2.0*n
        return x*TILE_SIZE, y*TILE_SIZE
    def pixels_to_latlon(self, px, py, z):
        n=2**z
        lon = px/TILE_SIZE/n*360.0-180.0
        lat_rad = math.atan(math.sinh(math.pi*(1-2*(py/TILE_SIZE)/n)))
        return math.degrees(lat_rad), lon

    # interaction
    def wheelEvent(self, e: QtGui.QWheelEvent):
        self.userInteracted.emit(3500)
        dy = e.pixelDelta().y() if not e.pixelDelta().isNull() else e.angleDelta().y()
        if dy == 0: e.ignore(); return
        dz = 1 if dy > 0 else -1
        old_zoom = self.zoom
        new_zoom = max(self.min_zoom, min(self.max_zoom, self.zoom + dz))
        if new_zoom == old_zoom: e.accept(); return
        mx, my = e.pos().x(), e.pos().y()
        w, h = self.width(), self.height()
        cx, cy = self.latlon_to_pixels(self.center_lat, self.center_lon, old_zoom)
        tlx, tly = cx - w/2, cy - h/2
        mxw, myw = tlx + mx, tly + my
        scale = 2 ** (new_zoom - old_zoom)
        new_tlx, new_tly = mxw * scale - mx, myw * scale - my
        new_cx, new_cy = new_tlx + w/2, new_tly + h/2
        self.center_lat, self.center_lon = self.pixels_to_latlon(new_cx, new_cy, new_zoom)
        self.zoom = new_zoom
        self.update(); e.accept()

    def mousePressEvent(self, e: QtGui.QMouseEvent):
        if e.button() == Qt.LeftButton:
            # Convert click position to lat/lon
            w,h=self.width(),self.height()
            cx,cy=self.latlon_to_pixels(self.center_lat,self.center_lon,self.zoom)
            tlx,tly=cx-w/2,cy-h/2
            px,py=tlx+e.pos().x(),tly+e.pos().y()
            lat,lon=self.pixels_to_latlon(px,py,self.zoom)

            if self.start_point_mode:
                # Set start point for path planning
                self._start_point = (lat, lon)
                print(f"[Map] START point set: Lat={lat:.7f}, Lon={lon:.7f}")
                self.update()
                self.startEndChanged.emit(self._start_point, self._end_point)
                self.userInteracted.emit(3500)
                e.accept(); return

            if self.end_point_mode:
                # Set end point for path planning
                self._end_point = (lat, lon)
                print(f"[Map] END point set: Lat={lat:.7f}, Lon={lon:.7f}")
                self.update()
                self.startEndChanged.emit(self._start_point, self._end_point)
                self.userInteracted.emit(3500)
                e.accept(); return

            if self.red_zone_mode:
                # Add a red zone point (no-fly area)
                point_num = len(self._current_red_zone)
                self._current_red_zone.append((lat, lon))
                print(f"[Map] Added RED ZONE point R{point_num}: Lat={lat:.7f}, Lon={lon:.7f}")
                print(f"[Map] Current red zone points: {len(self._current_red_zone)} (need at least 3 to complete)")
                self.update()
                self.userInteracted.emit(3500)
                e.accept(); return

            if self.boundary_mode:
                # Add a boundary point (yellow zone)
                point_num = len(self._boundary)
                self._boundary.append((lat, lon))
                print(f"[Map] Added boundary point B{point_num}: Lat={lat:.7f}, Lon={lon:.7f}")
                print(f"[Map] Total boundary points: {len(self._boundary)} (need at least 3 to form polygon)")
                self.update()
                self.boundaryChanged.emit(self.get_boundary())
                self.userInteracted.emit(3500)
                e.accept(); return

            if self.add_mode:
                # Add a waypoint at click
                default_alt = 30.0 if not self._wps else self._wps[-1].alt
                cmd = CMD_WAYPOINT if self._wps else CMD_TAKEOFF
                wp_num = len(self._wps)
                self._wps.append(WP(cmd=cmd, lat=lat, lon=lon, alt=default_alt))
                print(f"[Map] Added WP{wp_num}: Lat={lat:.7f}, Lon={lon:.7f}, Alt={default_alt}m")
                self.update()
                self.waypointsChanged.emit(self.get_waypoints())
                self.userInteracted.emit(3500)
                e.accept(); return
            self._drag = True; self._last = e.pos()
            self.userInteracted.emit(3500); e.accept()
        else:
            e.ignore()

    def mouseMoveEvent(self, e: QtGui.QMouseEvent):
        if self._drag and self._last is not None:
            dx = e.pos().x() - self._last.x()
            dy = e.pos().y() - self._last.y()
            self._last = e.pos()
            cx, cy = self.latlon_to_pixels(self.center_lat, self.center_lon, self.zoom)
            self.center_lat, self.center_lon = self.pixels_to_latlon(cx - dx, cy - dy, self.zoom)
            self.update(); e.accept()
        else:
            sb = self._statusbar() if callable(self._statusbar) else None
            if sb:
                w,h=self.width(),self.height()
                cx,cy=self.latlon_to_pixels(self.center_lat,self.center_lon,self.zoom)
                tlx,tly=cx-w/2,cy-h/2
                px,py=tlx+e.pos().x(),tly+e.pos().y()
                lat,lon=self.pixels_to_latlon(px,py,self.zoom)
                sb.showMessage(f"Lat {lat:.6f}  Lon {lon:.6f}  Zoom {self.zoom}", 1000)
            e.ignore()

    def mouseReleaseEvent(self, e: QtGui.QMouseEvent):
        if e.button() == Qt.LeftButton:
            self._drag = False; self._last = None
            self.userInteracted.emit(3500); e.accept()
        else:
            e.ignore()

    def _fetch_tile(self,z,x,y):
        if self.mbtiles:
            pm=self.mbtiles.get_tile(z,x,y)
            if pm: return pm
        pm=self.cache.get(z,x,y)
        if pm: return pm
        return self.http.get_tile(z,x,y, allow_network=not self.offline)

    def paintEvent(self,_):
        p = QPainter(self); p.fillRect(self.rect(), Qt.white)
        w,h=self.width(),self.height()
        cx,cy=self.latlon_to_pixels(self.center_lat,self.center_lon,self.zoom)
        tlx,tly=cx-w/2,cy-h/2
        start_tx=int(math.floor(tlx/TILE_SIZE)); start_ty=int(math.floor(tly/TILE_SIZE))
        end_tx=int(math.floor((tlx+w-1)/TILE_SIZE)); end_ty=int(math.floor((tly+h-1)/TILE_SIZE))
        for ty in range(start_ty,end_ty+1):
            for tx in range(start_tx,end_tx+1):
                sx=int(tx*TILE_SIZE-tlx); sy=int(ty*TILE_SIZE-tly)
                fx=wrap_x(tx,self.zoom); fy=clamp_y(ty,self.zoom)
                pm=self._fetch_tile(self.zoom,fx,fy)
                if not isinstance(pm,QPixmap) or pm.isNull():
                    r=QRect(sx,sy,TILE_SIZE,TILE_SIZE)
                    p.fillRect(r, Qt.darkGray); p.setPen(Qt.black); p.drawRect(r)
                else:
                    p.drawPixmap(sx,sy,pm)

        # crosshair
        p.setPen(Qt.red); p.drawLine(w//2-10,h//2,w//2+10,h//2); p.drawLine(w//2,h//2-10,w//2,h//2+10)

        # draw boundary polygon (YELLOW ZONE - survey area)
        if self._boundary:
            boundary_pts = []
            for lat, lon in self._boundary:
                px, py = self.latlon_to_pixels(lat, lon, self.zoom)
                sx = int(px - tlx); sy = int(py - tly)
                boundary_pts.append((sx, sy))

            p.setRenderHint(QPainter.Antialiasing, True)

            # Draw filled polygon with transparent yellow
            if len(boundary_pts) >= 3:
                poly_points = [QtCore.QPoint(x, y) for x, y in boundary_pts]
                polygon = QtGui.QPolygon(poly_points)
                p.setBrush(QtGui.QColor(255, 255, 0, 50))  # Semi-transparent yellow fill
                p.setPen(QPen(QtGui.QColor(255, 200, 0), 3))  # Yellow border
                p.drawPolygon(polygon)

            # Draw boundary lines
            p.setPen(QPen(QtGui.QColor(255, 200, 0), 3))  # Bright yellow
            for i in range(len(boundary_pts)):
                p.drawLine(boundary_pts[i][0], boundary_pts[i][1],
                          boundary_pts[(i+1) % len(boundary_pts)][0],
                          boundary_pts[(i+1) % len(boundary_pts)][1])

            # Draw boundary points
            for i, (sx, sy) in enumerate(boundary_pts):
                # Yellow marker
                p.setBrush(QtGui.QColor(255, 200, 0))
                p.setPen(QPen(Qt.black, 2))
                p.drawEllipse(QtCore.QPoint(sx, sy), 6, 6)
                # Label
                label = f"B{i}"
                font = QFont("Consolas", 9); font.setBold(True); p.setFont(font)
                p.setPen(QPen(Qt.black, 3))
                p.drawText(sx + 8, sy - 8, label)
                p.setPen(QtGui.QColor(255, 200, 0))
                p.drawText(sx + 8, sy - 8, label)

        # draw RED ZONES (no-fly areas)
        for rz_idx, red_zone in enumerate(self._red_zones):
            if len(red_zone) >= 3:
                rz_pts = []
                for lat, lon in red_zone:
                    px, py = self.latlon_to_pixels(lat, lon, self.zoom)
                    sx = int(px - tlx); sy = int(py - tly)
                    rz_pts.append((sx, sy))

                p.setRenderHint(QPainter.Antialiasing, True)

                # Draw filled polygon with semi-transparent red
                poly_points = [QtCore.QPoint(x, y) for x, y in rz_pts]
                polygon = QtGui.QPolygon(poly_points)
                p.setBrush(QtGui.QColor(255, 0, 0, 80))  # Semi-transparent red fill
                p.setPen(QPen(QtGui.QColor(200, 0, 0), 2))  # Dark red border
                p.drawPolygon(polygon)

                # Draw red zone points
                for i, (sx, sy) in enumerate(rz_pts):
                    p.setBrush(QtGui.QColor(255, 50, 50))
                    p.setPen(QPen(Qt.black, 1))
                    p.drawEllipse(QtCore.QPoint(sx, sy), 4, 4)

                # Draw "NO-FLY" label at center
                if rz_pts:
                    center_x = sum(pt[0] for pt in rz_pts) // len(rz_pts)
                    center_y = sum(pt[1] for pt in rz_pts) // len(rz_pts)
                    font = QFont("Arial", 8, QFont.Bold); p.setFont(font)
                    p.setPen(QPen(Qt.white, 3))
                    p.drawText(center_x - 20, center_y, f"NO-FLY")
                    p.setPen(QtGui.QColor(200, 0, 0))
                    p.drawText(center_x - 20, center_y, f"NO-FLY")

        # draw current red zone being drawn (dashed red)
        if self._current_red_zone:
            curr_rz_pts = []
            for lat, lon in self._current_red_zone:
                px, py = self.latlon_to_pixels(lat, lon, self.zoom)
                sx = int(px - tlx); sy = int(py - tly)
                curr_rz_pts.append((sx, sy))

            p.setRenderHint(QPainter.Antialiasing, True)

            # Draw dashed lines
            pen = QPen(QtGui.QColor(255, 0, 0), 2, Qt.DashLine)
            p.setPen(pen)
            for i in range(len(curr_rz_pts) - 1):
                p.drawLine(curr_rz_pts[i][0], curr_rz_pts[i][1],
                          curr_rz_pts[i+1][0], curr_rz_pts[i+1][1])

            # Draw points
            for i, (sx, sy) in enumerate(curr_rz_pts):
                p.setBrush(QtGui.QColor(255, 100, 100))
                p.setPen(QPen(Qt.black, 2))
                p.drawEllipse(QtCore.QPoint(sx, sy), 5, 5)
                # Label
                label = f"R{i}"
                font = QFont("Consolas", 8); font.setBold(True); p.setFont(font)
                p.setPen(QPen(Qt.black, 2))
                p.drawText(sx + 6, sy - 6, label)
                p.setPen(QtGui.QColor(255, 50, 50))
                p.drawText(sx + 6, sy - 6, label)

        # draw START point (large green marker)
        if self._start_point:
            lat, lon = self._start_point
            px, py = self.latlon_to_pixels(lat, lon, self.zoom)
            sx = int(px - tlx); sy = int(py - tly)

            p.setRenderHint(QPainter.Antialiasing, True)
            # Green circle with white border
            p.setBrush(QtGui.QColor(0, 200, 0))
            p.setPen(QPen(Qt.white, 3))
            p.drawEllipse(QtCore.QPoint(sx, sy), 12, 12)
            p.setPen(QPen(Qt.black, 2))
            p.drawEllipse(QtCore.QPoint(sx, sy), 12, 12)
            # "START" label
            font = QFont("Arial", 9, QFont.Bold); p.setFont(font)
            p.setPen(QPen(Qt.black, 3))
            p.drawText(sx - 18, sy + 25, "START")
            p.setPen(QtGui.QColor(0, 180, 0))
            p.drawText(sx - 18, sy + 25, "START")
            # Arrow/play icon inside
            p.setPen(Qt.white)
            p.setBrush(Qt.white)
            triangle = QtGui.QPolygon([
                QtCore.QPoint(sx - 4, sy - 6),
                QtCore.QPoint(sx - 4, sy + 6),
                QtCore.QPoint(sx + 6, sy)
            ])
            p.drawPolygon(triangle)

        # draw END point (large red marker)
        if self._end_point:
            lat, lon = self._end_point
            px, py = self.latlon_to_pixels(lat, lon, self.zoom)
            ex = int(px - tlx); ey = int(py - tly)

            p.setRenderHint(QPainter.Antialiasing, True)
            # Red circle with white border
            p.setBrush(QtGui.QColor(220, 50, 50))
            p.setPen(QPen(Qt.white, 3))
            p.drawEllipse(QtCore.QPoint(ex, ey), 12, 12)
            p.setPen(QPen(Qt.black, 2))
            p.drawEllipse(QtCore.QPoint(ex, ey), 12, 12)
            # "END" label
            font = QFont("Arial", 9, QFont.Bold); p.setFont(font)
            p.setPen(QPen(Qt.black, 3))
            p.drawText(ex - 10, ey + 25, "END")
            p.setPen(QtGui.QColor(200, 0, 0))
            p.drawText(ex - 10, ey + 25, "END")
            # Square/stop icon inside
            p.setPen(Qt.white)
            p.setBrush(Qt.white)
            p.drawRect(ex - 5, ey - 5, 10, 10)

        # draw generated coverage path (GREEN LINES - drone flight path)
        if self._generated_path:
            gen_pts = []
            for wp in self._generated_path:
                px, py = self.latlon_to_pixels(wp.lat, wp.lon, self.zoom)
                sx = int(px - tlx); sy = int(py - tly)
                gen_pts.append((sx, sy))

            p.setRenderHint(QPainter.Antialiasing, True)

            # Draw green path lines
            p.setPen(QPen(QtGui.QColor(0, 255, 0), 2))  # Green color
            for i in range(len(gen_pts) - 1):
                p.drawLine(gen_pts[i][0], gen_pts[i][1], gen_pts[i+1][0], gen_pts[i+1][1])

            # Draw small green dots at waypoints
            p.setBrush(QtGui.QColor(0, 200, 0))
            p.setPen(QPen(Qt.black, 1))
            for sx, sy in gen_pts:
                p.drawEllipse(QtCore.QPoint(sx, sy), 3, 3)

            # Draw start and end markers
            if gen_pts:
                # Start marker (green circle with "S")
                sx, sy = gen_pts[0]
                p.setBrush(QtGui.QColor(0, 255, 0))
                p.setPen(QPen(Qt.black, 2))
                p.drawEllipse(QtCore.QPoint(sx, sy), 8, 8)
                font = QFont("Consolas", 8, QFont.Bold); p.setFont(font)
                p.setPen(Qt.black)
                p.drawText(sx - 4, sy + 4, "S")

                # End marker (red circle with "E")
                ex, ey = gen_pts[-1]
                p.setBrush(QtGui.QColor(255, 100, 100))
                p.setPen(QPen(Qt.black, 2))
                p.drawEllipse(QtCore.QPoint(ex, ey), 8, 8)
                p.setPen(Qt.black)
                p.drawText(ex - 4, ey + 4, "E")

        # draw planned waypoints/path (WHITE markers with numbers - manual waypoints)
        if self._wps:
            pts=[]
            for wp in self._wps:
                px,py=self.latlon_to_pixels(wp.lat,wp.lon,self.zoom)
                sx=int(px-tlx); sy=int(py-tly)
                pts.append((sx,sy))
            p.setRenderHint(QPainter.Antialiasing, True)
            p.setPen(QPen(Qt.cyan, 3))  # Cyan for manual waypoint path
            for i in range(len(pts)-1):
                p.drawLine(pts[i][0],pts[i][1],pts[i+1][0],pts[i+1][1])
            for i,(sx,sy) in enumerate(pts):
                # outer halo
                p.setBrush(Qt.white)
                p.setPen(QPen(Qt.black, 2))
                p.drawEllipse(QtCore.QPoint(sx,sy), 7, 7)
                # index label with halo
                label = str(i)
                font = QFont("Consolas", 10); font.setBold(True); p.setFont(font)
                # black outline then white text
                p.setPen(QPen(Qt.black, 4))
                p.drawText(sx+10, sy-10, label)
                p.setPen(Qt.white)
                p.drawText(sx+10, sy-10, label)

        p.setPen(Qt.black); p.drawText(10, self.height()-10, f"Lat {self.center_lat:.5f}  Lon {self.center_lon:.5f}  Zoom {self.zoom}")

    def _status(self, s: str):
        sb = self._statusbar() if callable(self._statusbar) else None
        if sb: sb.showMessage(s, 4000)

# ---------------- Planner Table ----------------
class PlannerTable(QTableWidget):
    def __init__(self):
        super().__init__(0, 4)
        self.setHorizontalHeaderLabels(["Command", "Latitude", "Longitude", "Altitude (m)"])
        self.horizontalHeader().setStretchLastSection(True)
        self.verticalHeader().setDefaultSectionSize(22)
        self.setSelectionBehavior(QTableWidget.SelectRows)
        self.setToolTip("Mission waypoint sequence - Click to select, edit values directly in cells")

    def load(self, wps: List[WP]):
        self.setRowCount(0)
        for wp in wps:
            self._append_row(wp)

    def _append_row(self, wp: WP):
        r = self.rowCount()
        self.insertRow(r)

        combo = QComboBox()
        for name, val in CMD_CHOICES:
            combo.addItem(name, val)
        idx = next((i for i,(n,v) in enumerate(CMD_CHOICES) if v==wp.cmd), 1)
        combo.setCurrentIndex(idx)
        self.setCellWidget(r, 0, combo)

        lat_item = QTableWidgetItem(f"{wp.lat:.7f}")
        lon_item = QTableWidgetItem(f"{wp.lon:.7f}")
        alt_item = QTableWidgetItem(f"{wp.alt:.1f}")
        self.setItem(r,1,lat_item); self.setItem(r,2,lon_item); self.setItem(r,3,alt_item)

        # when the CMD combo changes, treat as edit
        combo.currentIndexChanged.connect(lambda *_: self.itemChanged.emit(self.item(r,1)))

    def to_list(self) -> List[WP]:
        out=[]
        for r in range(self.rowCount()):
            combo: QComboBox = self.cellWidget(r,0)
            cmd = int(combo.currentData())
            try:
                lat = float(self.item(r,1).text()); lon = float(self.item(r,2).text()); alt = float(self.item(r,3).text())
            except Exception:
                lat=0.0; lon=0.0; alt=0.0
            out.append(WP(cmd=cmd, lat=lat, lon=lon, alt=alt))
        return out

    def del_selected(self):
        rows = sorted({idx.row() for idx in self.selectedIndexes()}, reverse=True)
        for r in rows:
            self.removeRow(r)

    def clear_all(self):
        self.setRowCount(0)

# ---------------- Map Dashboard with Planner Tab ----------------
class MapDashboard(QWidget):
    FOLLOW_SUSPEND_MS = 3500

    def __init__(self, worker: MavWorker, statusbar_provider):
        super().__init__()
        self.worker = worker

        # Path planning state
        self._generated_path: List[WP] = []  # Generated coverage path waypoints
        self._path_stats: dict = {}  # Statistics from path generation

        # Two maps: main (with HUD) and planner (no HUD)
        self.map = MapWidget(status_provider=statusbar_provider, show_overlays=True)
        self.plan_map = MapWidget(status_provider=statusbar_provider, show_overlays=False)

        # tabs
        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_map_tab(), "Map")
        self.tabs.addTab(self._build_plan_tab(), "Plan")

        lay = QVBoxLayout(self)
        lay.addWidget(self.tabs, 1)

        # Follow suppression
        self._suppress_until = 0
        self.map.userInteracted.connect(self._on_user_interaction)
        self.plan_map.userInteracted.connect(self._on_user_interaction)

        # Status updates -> HUD + autopan
        self.worker.status.connect(self._on_status)

        # mission signals
        self.worker.mission_read_ok.connect(self._on_mission_read)
        self.worker.mission_write_ok.connect(self._on_mission_write_ok)
        self.worker.mission_clear_ok.connect(self._on_mission_cleared)
        self.worker.mission_error.connect(self._on_mission_error)

        # Keep table and both maps in sync
        self.table.itemChanged.connect(self._on_table_changed)
        self.plan_map.waypointsChanged.connect(self._on_map_changed)
        self.map.waypointsChanged.connect(self._on_map_changed)

    def _build_map_tab(self):
        w = QWidget(); layout = QVBoxLayout(w)

        tb = QToolBar()
        layout.addWidget(tb)
        tb.addWidget(QLabel("Layer:"))
        self.layer = QComboBox(); self.layer.addItems(list(PROVIDERS.keys())); self.layer.setCurrentText("ESRI Satellite")
        tb.addWidget(self.layer)
        self.offline = QCheckBox("Offline (cache only)"); tb.addWidget(self.offline)
        self.autopan = QCheckBox("Auto Pan"); self.autopan.setChecked(True); tb.addWidget(self.autopan)
        open_mbt = QPushButton("Open MBTiles…"); tb.addWidget(open_mbt)
        tb.addSeparator(); tb.addWidget(QLabel(" Drag: pan • Wheel: zoom"))

        layout.addWidget(self.map, 1)

        self.layer.currentTextChanged.connect(lambda k: [self.map.set_provider(k), self.plan_map.set_provider(k)])
        self.offline.stateChanged.connect(lambda *_: [self.map.set_offline(self.offline.isChecked()), self.plan_map.set_offline(self.offline.isChecked())])
        open_mbt.clicked.connect(self._open_mbtiles)
        self.map.set_provider(self.layer.currentText())
        self.plan_map.set_provider(self.layer.currentText())
        return w

    def _build_plan_tab(self):
        w = QWidget(); v = QVBoxLayout(w)

        # ============ WAYPOINT TOOLBAR ============
        tb = QToolBar(); v.addWidget(tb)
        tb.addWidget(QLabel(" Manual: "))

        self.btn_add_mode = QCheckBox("Add Waypoint (Click)")
        self.btn_add_mode.setToolTip("Enable to add waypoints by clicking on the map")
        tb.addWidget(self.btn_add_mode)
        self.btn_set_home = QPushButton("Set Home")
        self.btn_set_home.setToolTip("Set the first waypoint (WP0) as home/takeoff position")
        tb.addWidget(self.btn_set_home)
        self.btn_delete = QPushButton("Delete WP")
        self.btn_delete.setToolTip("Delete selected waypoints from the sequence")
        tb.addWidget(self.btn_delete)
        self.btn_clear_local = QPushButton("Clear Plan")
        self.btn_clear_local.setToolTip("Clear all waypoints from map and table")
        tb.addWidget(self.btn_clear_local)

        tb.addSeparator()
        tb.addWidget(QLabel(" Vehicle: "))
        self.btn_read = QPushButton("READ")
        self.btn_read.setToolTip("Download mission from flight controller")
        tb.addWidget(self.btn_read)
        self.btn_write = QPushButton("WRITE")
        self.btn_write.setToolTip("Upload mission to flight controller")
        tb.addWidget(self.btn_write)
        self.btn_clear_vehicle = QPushButton("CLEAR")
        self.btn_clear_vehicle.setToolTip("Remove all waypoints from flight controller")
        tb.addWidget(self.btn_clear_vehicle)

        # ============ PATH PLANNING TOOLBAR 1 - ZONES ============
        path_tb = QToolBar(); v.addWidget(path_tb)

        # Path planning section header
        path_label = QLabel(" ZONES: ")
        path_label.setStyleSheet("font-weight: bold; color: #006600;")
        path_tb.addWidget(path_label)

        # Boundary drawing mode (Yellow Zone)
        self.btn_boundary_mode = QCheckBox("Draw Boundary")
        self.btn_boundary_mode.setToolTip(
            "Enable to draw survey boundary (YELLOW ZONE) by clicking on map.\n"
            "Click at least 3 points to define the area for path planning."
        )
        self.btn_boundary_mode.setStyleSheet("QCheckBox { color: #cc8800; font-weight: bold; }")
        path_tb.addWidget(self.btn_boundary_mode)

        # Red zone drawing mode (No-Fly Zone)
        self.btn_red_zone_mode = QCheckBox("Draw No-Fly Zone")
        self.btn_red_zone_mode.setToolTip(
            "Enable to draw NO-FLY areas (RED ZONES) by clicking on map.\n"
            "The drone will avoid these areas during path planning.\n"
            "Click at least 3 points, then click 'Finish Zone'."
        )
        self.btn_red_zone_mode.setStyleSheet("QCheckBox { color: #cc0000; font-weight: bold; }")
        path_tb.addWidget(self.btn_red_zone_mode)

        # Finish red zone button
        self.btn_finish_red_zone = QPushButton("Finish Zone")
        self.btn_finish_red_zone.setStyleSheet("QPushButton { background-color: #ffcccc; }")
        self.btn_finish_red_zone.setToolTip("Complete the current red zone polygon")
        path_tb.addWidget(self.btn_finish_red_zone)

        # Clear red zones button
        self.btn_clear_red_zones = QPushButton("Clear Red Zones")
        self.btn_clear_red_zones.setStyleSheet("QPushButton { background-color: #ff9999; }")
        self.btn_clear_red_zones.setToolTip("Remove all no-fly zones")
        path_tb.addWidget(self.btn_clear_red_zones)

        path_tb.addSeparator()

        # Start/End point controls
        path_tb.addWidget(QLabel(" START/END: "))

        self.btn_set_start = QCheckBox("Set Start")
        self.btn_set_start.setToolTip("Click on map to set where the drone should START the survey")
        self.btn_set_start.setStyleSheet("QCheckBox { color: #008800; font-weight: bold; }")
        path_tb.addWidget(self.btn_set_start)

        self.btn_set_end = QCheckBox("Set End")
        self.btn_set_end.setToolTip("Click on map to set where the drone should END the survey")
        self.btn_set_end.setStyleSheet("QCheckBox { color: #880000; font-weight: bold; }")
        path_tb.addWidget(self.btn_set_end)

        self.btn_clear_start_end = QPushButton("Clear S/E")
        self.btn_clear_start_end.setToolTip("Clear start and end points")
        path_tb.addWidget(self.btn_clear_start_end)

        # ============ PATH PLANNING TOOLBAR 2 - GENERATION ============
        path_tb2 = QToolBar(); v.addWidget(path_tb2)

        path_tb2.addWidget(QLabel(" GENERATE: "))

        # Row width input
        path_tb2.addWidget(QLabel(" Row Width:"))
        self.spin_row_width = QSpinBox()
        self.spin_row_width.setRange(1, 100)
        self.spin_row_width.setValue(10)
        self.spin_row_width.setSuffix(" m")
        self.spin_row_width.setToolTip(
            "Distance between parallel flight lines (in meters).\n"
            "Smaller values = more coverage but longer flight time.\n"
            "Typical values: 5-20m depending on camera/sensor."
        )
        path_tb2.addWidget(self.spin_row_width)

        # Altitude input
        path_tb2.addWidget(QLabel(" Alt:"))
        self.spin_path_alt = QSpinBox()
        self.spin_path_alt.setRange(5, 500)
        self.spin_path_alt.setValue(30)
        self.spin_path_alt.setSuffix(" m")
        self.spin_path_alt.setToolTip("Flight altitude for generated path (in meters)")
        path_tb2.addWidget(self.spin_path_alt)

        path_tb2.addSeparator()

        # Generate path button
        self.btn_generate_path = QPushButton("Generate Path")
        self.btn_generate_path.setStyleSheet("QPushButton { background-color: #90EE90; font-weight: bold; padding: 5px; }")
        self.btn_generate_path.setToolTip(
            "Generate coverage flight path (GREEN LINES) inside the boundary.\n"
            "Creates a lawnmower pattern to survey the entire area.\n"
            "Avoids red zones and respects start/end points.\n"
            "Requires at least 3 boundary points."
        )
        path_tb2.addWidget(self.btn_generate_path)

        # Apply path button
        self.btn_apply_path = QPushButton("Apply to Mission")
        self.btn_apply_path.setStyleSheet("QPushButton { background-color: #87CEEB; font-weight: bold; padding: 5px; }")
        self.btn_apply_path.setToolTip(
            "Convert generated path to mission waypoints.\n"
            "This will replace current waypoints with the generated path."
        )
        path_tb2.addWidget(self.btn_apply_path)

        # Clear boundary button
        self.btn_clear_boundary = QPushButton("Clear All")
        self.btn_clear_boundary.setStyleSheet("QPushButton { background-color: #FFB6C1; }")
        self.btn_clear_boundary.setToolTip("Clear boundary, red zones, start/end points, and generated path")
        path_tb2.addWidget(self.btn_clear_boundary)

        # Help button
        self.btn_path_help = QPushButton("?")
        self.btn_path_help.setFixedWidth(30)
        self.btn_path_help.setToolTip("Show path planning help")
        path_tb2.addWidget(self.btn_path_help)

        # ============ STATUS/INFO LABEL ============
        self.path_status_label = QLabel("Path Planning: Draw boundary (yellow) → Add no-fly zones (red) → Set start/end → Generate path (green)")
        self.path_status_label.setStyleSheet("QLabel { padding: 3px; background: #f5f5dc; border: 1px solid #ddd; }")
        v.addWidget(self.path_status_label)

        # ============ MAP ============
        map_label = QLabel("YELLOW = Survey Boundary | RED = No-Fly Zone | GREEN = Flight Path | CYAN = Waypoints | START/END = Path endpoints")
        map_label.setStyleSheet("QLabel { font-weight: bold; padding: 5px; background: #e8f4f8; font-size: 10px; }")
        v.addWidget(map_label)
        self.plan_map.setFixedHeight(350)
        v.addWidget(self.plan_map)

        # ============ TABLE ============
        seq_label = QLabel("Waypoint Sequence - WP0 is Home/Takeoff position")
        seq_label.setStyleSheet("QLabel { font-weight: bold; padding: 5px; background: #f0f0f0; }")
        v.addWidget(seq_label)
        self.table = PlannerTable()
        v.addWidget(self.table, 1)

        # ============ WIRING ============
        # Waypoint controls
        self.btn_add_mode.stateChanged.connect(self._on_add_mode_changed)
        self.btn_set_home.clicked.connect(self._set_home_wp0)
        self.btn_read.clicked.connect(self._read_mission)
        self.btn_write.clicked.connect(self._write_mission)
        self.btn_delete.clicked.connect(self._delete_selected)
        self.btn_clear_local.clicked.connect(self._clear_local)
        self.btn_clear_vehicle.clicked.connect(self._clear_vehicle)

        # Path planning controls - Zones
        self.btn_boundary_mode.stateChanged.connect(self._on_boundary_mode_changed)
        self.btn_red_zone_mode.stateChanged.connect(self._on_red_zone_mode_changed)
        self.btn_finish_red_zone.clicked.connect(self._finish_red_zone)
        self.btn_clear_red_zones.clicked.connect(self._clear_red_zones)

        # Path planning controls - Start/End
        self.btn_set_start.stateChanged.connect(self._on_start_mode_changed)
        self.btn_set_end.stateChanged.connect(self._on_end_mode_changed)
        self.btn_clear_start_end.clicked.connect(self._clear_start_end)

        # Path planning controls - Generation
        self.btn_generate_path.clicked.connect(self._generate_coverage_path)
        self.btn_apply_path.clicked.connect(self._apply_path_to_mission)
        self.btn_clear_boundary.clicked.connect(self._clear_all_path_planning)
        self.btn_path_help.clicked.connect(self._show_path_help)

        # Sync between maps
        self.plan_map.boundaryChanged.connect(self._on_boundary_changed)
        self.plan_map.redZonesChanged.connect(self._on_red_zones_changed)
        self.plan_map.startEndChanged.connect(self._on_start_end_changed)

        return w

    def _on_add_mode_changed(self):
        """Handle add waypoint mode toggle"""
        enabled = self.btn_add_mode.isChecked()
        self.plan_map.set_add_mode(enabled)
        self.map.set_add_mode(enabled)
        # Disable boundary mode if add mode is enabled
        if enabled and self.btn_boundary_mode.isChecked():
            self.btn_boundary_mode.setChecked(False)
        print(f"[UI] Add waypoint mode: {'ENABLED' if enabled else 'DISABLED'}")

    def _on_boundary_mode_changed(self):
        """Handle boundary drawing mode toggle"""
        enabled = self.btn_boundary_mode.isChecked()
        self.plan_map.set_boundary_mode(enabled)
        self.map.set_boundary_mode(enabled)
        # Disable add mode if boundary mode is enabled
        if enabled and self.btn_add_mode.isChecked():
            self.btn_add_mode.setChecked(False)

        if enabled:
            self._uncheck_other_modes('boundary')
            self.path_status_label.setText("BOUNDARY MODE: Click on map to add boundary points (need at least 3)")
            self.path_status_label.setStyleSheet("QLabel { padding: 3px; background: #fff3cd; border: 1px solid #ffc107; }")
            print("[UI] Boundary drawing mode ENABLED")
            print("[UI] Instructions: Click on map to add boundary points. Need at least 3 points to form polygon.")
        else:
            self._update_path_status()
            print("[UI] Boundary drawing mode DISABLED")

    def _on_red_zone_mode_changed(self):
        """Handle red zone drawing mode toggle"""
        enabled = self.btn_red_zone_mode.isChecked()
        self.plan_map.set_red_zone_mode(enabled)
        self.map.set_red_zone_mode(enabled)

        if enabled:
            self._uncheck_other_modes('red_zone')
            self.path_status_label.setText("RED ZONE MODE: Click on map to add no-fly area points. Click 'Finish Zone' when done.")
            self.path_status_label.setStyleSheet("QLabel { padding: 3px; background: #ffcccc; border: 1px solid #cc0000; }")
            print("[UI] Red zone drawing mode ENABLED")
            print("[UI] Instructions: Click on map to add no-fly area points. Need at least 3 points.")
        else:
            self._update_path_status()
            print("[UI] Red zone drawing mode DISABLED")

    def _on_start_mode_changed(self):
        """Handle start point setting mode toggle"""
        enabled = self.btn_set_start.isChecked()
        self.plan_map.set_start_point_mode(enabled)
        self.map.set_start_point_mode(enabled)

        if enabled:
            self._uncheck_other_modes('start')
            self.path_status_label.setText("START POINT MODE: Click on map to set where the drone will START the survey")
            self.path_status_label.setStyleSheet("QLabel { padding: 3px; background: #d4edda; border: 1px solid #28a745; }")
            print("[UI] Start point setting mode ENABLED")
        else:
            self._update_path_status()
            print("[UI] Start point setting mode DISABLED")

    def _on_end_mode_changed(self):
        """Handle end point setting mode toggle"""
        enabled = self.btn_set_end.isChecked()
        self.plan_map.set_end_point_mode(enabled)
        self.map.set_end_point_mode(enabled)

        if enabled:
            self._uncheck_other_modes('end')
            self.path_status_label.setText("END POINT MODE: Click on map to set where the drone will END the survey")
            self.path_status_label.setStyleSheet("QLabel { padding: 3px; background: #f8d7da; border: 1px solid #dc3545; }")
            print("[UI] End point setting mode ENABLED")
        else:
            self._update_path_status()
            print("[UI] End point setting mode DISABLED")

    def _uncheck_other_modes(self, keep_mode: str):
        """Uncheck all mode checkboxes except the specified one"""
        if keep_mode != 'add':
            self.btn_add_mode.blockSignals(True)
            self.btn_add_mode.setChecked(False)
            self.btn_add_mode.blockSignals(False)
        if keep_mode != 'boundary':
            self.btn_boundary_mode.blockSignals(True)
            self.btn_boundary_mode.setChecked(False)
            self.btn_boundary_mode.blockSignals(False)
        if keep_mode != 'red_zone':
            self.btn_red_zone_mode.blockSignals(True)
            self.btn_red_zone_mode.setChecked(False)
            self.btn_red_zone_mode.blockSignals(False)
        if keep_mode != 'start':
            self.btn_set_start.blockSignals(True)
            self.btn_set_start.setChecked(False)
            self.btn_set_start.blockSignals(False)
        if keep_mode != 'end':
            self.btn_set_end.blockSignals(True)
            self.btn_set_end.setChecked(False)
            self.btn_set_end.blockSignals(False)

    def _finish_red_zone(self):
        """Finish drawing the current red zone"""
        print("[UI] FINISH RED ZONE button clicked")
        if self.plan_map.finish_current_red_zone():
            self.path_status_label.setText(f"Red zone added! Total: {len(self.plan_map.get_red_zones())} no-fly zones")
            self.path_status_label.setStyleSheet("QLabel { padding: 3px; background: #d4edda; border: 1px solid #28a745; }")
            # Sync to other map
            self.map.set_red_zones(self.plan_map.get_red_zones())
            QMessageBox.information(self, "Red Zone Added",
                f"No-fly zone added successfully!\n\n"
                f"Total red zones: {len(self.plan_map.get_red_zones())}\n\n"
                f"The path planner will avoid this area.")
        else:
            QMessageBox.warning(self, "Cannot Finish Zone",
                "Need at least 3 points to create a red zone.\n\n"
                "Keep clicking on the map to add more points.")

    def _clear_red_zones(self):
        """Clear all red zones"""
        print("[UI] CLEAR RED ZONES button clicked")
        count = len(self.plan_map.get_red_zones())
        if count == 0:
            QMessageBox.information(self, "Clear Red Zones", "No red zones to clear.")
            return

        reply = QMessageBox.question(self, "Clear Red Zones",
            f"Remove all {count} red zone(s)?\n\nThis cannot be undone.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            self.plan_map.clear_red_zones()
            self.map.clear_red_zones()
            self._update_path_status()
            print(f"[UI] Cleared {count} red zones")

    def _clear_start_end(self):
        """Clear start and end points"""
        print("[UI] CLEAR START/END button clicked")
        self.plan_map.clear_start_end_points()
        self.map.clear_start_end_points()
        self._update_path_status()

    def _on_red_zones_changed(self, red_zones):
        """Handle red zones changes from map"""
        self.map.set_red_zones(red_zones)
        self.plan_map.set_red_zones(red_zones)
        if red_zones:
            self.path_status_label.setText(f"Red zones: {len(red_zones)} no-fly area(s) defined")

    def _on_start_end_changed(self, start_point, end_point):
        """Handle start/end point changes from map"""
        # Sync to both maps
        self.map.set_start_point(start_point)
        self.map.set_end_point(end_point)
        self.plan_map._start_point = start_point
        self.plan_map._end_point = end_point

        # Update status
        parts = []
        if start_point:
            parts.append(f"START: ({start_point[0]:.5f}, {start_point[1]:.5f})")
        if end_point:
            parts.append(f"END: ({end_point[0]:.5f}, {end_point[1]:.5f})")
        if parts:
            self.path_status_label.setText(" | ".join(parts))
            self.path_status_label.setStyleSheet("QLabel { padding: 3px; background: #d4edda; border: 1px solid #28a745; }")

    def _clear_all_path_planning(self):
        """Clear all path planning elements: boundary, red zones, start/end, generated path"""
        print("[UI] CLEAR ALL button clicked")

        has_data = (self.plan_map.get_boundary() or
                   self.plan_map.get_red_zones() or
                   self.plan_map.get_start_point() or
                   self.plan_map.get_end_point() or
                   hasattr(self, '_generated_path') and self._generated_path)

        if not has_data:
            QMessageBox.information(self, "Clear All", "Nothing to clear.")
            return

        reply = QMessageBox.question(self, "Clear All Path Planning",
            "Clear all path planning elements?\n\n"
            "This will remove:\n"
            "- Survey boundary (yellow zone)\n"
            "- All no-fly zones (red zones)\n"
            "- Start and end points\n"
            "- Generated path\n\n"
            "This will NOT affect mission waypoints.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            # Clear boundary
            self.plan_map.clear_boundary()
            self.map.clear_boundary()

            # Clear red zones
            self.plan_map.clear_red_zones()
            self.map.clear_red_zones()

            # Clear start/end
            self.plan_map.clear_start_end_points()
            self.map.clear_start_end_points()

            # Clear generated path
            self.plan_map.clear_generated_path()
            self.map.clear_generated_path()
            if hasattr(self, '_generated_path'):
                self._generated_path = []
            if hasattr(self, '_path_stats'):
                self._path_stats = {}

            self._update_path_status()
            print("[UI] All path planning elements cleared")

    def _on_boundary_changed(self, boundary):
        """Handle boundary polygon changes from map"""
        # Sync boundary to both maps
        self.map.set_boundary(boundary)
        self.plan_map.set_boundary(boundary)
        count = len(boundary)
        if count < 3:
            self.path_status_label.setText(f"Boundary: {count} points (need {3 - count} more to form polygon)")
            self.path_status_label.setStyleSheet("QLabel { padding: 3px; background: #fff3cd; border: 1px solid #ffc107; }")
        else:
            self.path_status_label.setText(f"Boundary: {count} points - Ready to generate path!")
            self.path_status_label.setStyleSheet("QLabel { padding: 3px; background: #d4edda; border: 1px solid #28a745; }")

    def _generate_coverage_path(self):
        """Generate coverage flight path inside the boundary"""
        print("\n" + "=" * 60)
        print("[UI] GENERATE PATH button clicked")
        print("=" * 60)

        boundary = self.plan_map.get_boundary()
        if len(boundary) < 3:
            print(f"[UI] ERROR: Not enough boundary points ({len(boundary)}). Need at least 3.")
            QMessageBox.warning(self, "Generate Path",
                f"Cannot generate path: Need at least 3 boundary points.\n\n"
                f"Current boundary points: {len(boundary)}\n\n"
                f"Instructions:\n"
                f"1. Enable 'Draw Boundary' checkbox\n"
                f"2. Click on the map to add boundary points\n"
                f"3. Add at least 3 points to form a polygon\n"
                f"4. Click 'Generate Path' again")
            return

        row_width = self.spin_row_width.value()
        altitude = self.spin_path_alt.value()

        # Get red zones (no-fly areas)
        red_zones = self.plan_map.get_red_zones()

        # Get start and end points
        start_point = self.plan_map.get_start_point()
        end_point = self.plan_map.get_end_point()

        print(f"[UI] Generating coverage path...")
        print(f"[UI] Parameters:")
        print(f"[UI]   - Row width: {row_width}m")
        print(f"[UI]   - Altitude: {altitude}m")
        print(f"[UI]   - Boundary points: {len(boundary)}")
        print(f"[UI]   - Red zones (no-fly): {len(red_zones)}")
        if start_point:
            print(f"[UI]   - Start point: ({start_point[0]:.7f}, {start_point[1]:.7f})")
        if end_point:
            print(f"[UI]   - End point: ({end_point[0]:.7f}, {end_point[1]:.7f})")

        # Generate the path with all parameters
        waypoints, stats = PathPlanner.generate_coverage_path(
            boundary=boundary,
            row_width_meters=row_width,
            altitude=altitude,
            red_zones=red_zones,
            start_point=start_point,
            end_point=end_point
        )

        if not waypoints:
            print("[UI] ERROR: Path generation failed - no waypoints generated")
            QMessageBox.warning(self, "Generate Path",
                "Failed to generate path. The boundary may be too small or invalid.\n\n"
                "Try:\n"
                "- Making the boundary larger\n"
                "- Reducing the row width\n"
                "- Checking that boundary points form a valid polygon\n"
                "- Ensuring red zones don't cover the entire area")
            return

        # Store generated path
        self._generated_path = waypoints
        self._path_stats = stats

        # Display on both maps
        self.plan_map.set_generated_path(waypoints)
        self.map.set_generated_path(waypoints)

        # Update status with comprehensive info
        dist_km = stats['total_distance_m'] / 1000
        time_min = stats['estimated_time_s'] / 60
        status_parts = [
            f"{stats['total_waypoints']} waypoints",
            f"{stats['total_lines']} lines",
            f"{dist_km:.2f} km",
            f"~{time_min:.1f} min"
        ]
        if stats.get('red_zones_count', 0) > 0:
            status_parts.append(f"{stats['red_zones_count']} zones avoided")
        if stats.get('has_start_point'):
            status_parts.append("START set")
        if stats.get('has_end_point'):
            status_parts.append("END set")

        self.path_status_label.setText("Path Generated: " + " | ".join(status_parts))
        self.path_status_label.setStyleSheet("QLabel { padding: 3px; background: #d4edda; border: 1px solid #28a745; font-weight: bold; }")

        print(f"\n[UI] Path generation successful!")
        print(f"[UI] Click 'Apply to Mission' to convert to mission waypoints")

        # Build info message
        msg_parts = [
            f"Coverage path generated successfully!\n\n",
            f"Statistics:\n",
            f"  - Waypoints: {stats['total_waypoints']}\n",
            f"  - Survey lines: {stats['total_lines']}\n",
            f"  - Total distance: {dist_km:.2f} km\n",
            f"  - Estimated flight time: {time_min:.1f} minutes\n",
            f"  - Row spacing: {stats['row_width_m']}m\n",
        ]
        if stats.get('red_zones_count', 0) > 0:
            msg_parts.append(f"  - Red zones avoided: {stats['red_zones_count']}\n")
        if stats.get('has_start_point'):
            msg_parts.append(f"  - Custom start point: YES\n")
        if stats.get('has_end_point'):
            msg_parts.append(f"  - Custom end point: YES\n")
        msg_parts.append(f"\nThe GREEN path is now displayed on the map.\n")
        msg_parts.append(f"Click 'Apply to Mission' to convert it to mission waypoints.")

        QMessageBox.information(self, "Path Generated", "".join(msg_parts))

    def _apply_path_to_mission(self):
        """Apply generated path to mission waypoints"""
        print("\n[UI] APPLY PATH TO MISSION button clicked")

        if not hasattr(self, '_generated_path') or not self._generated_path:
            print("[UI] ERROR: No generated path to apply")
            QMessageBox.warning(self, "Apply Path",
                "No generated path to apply.\n\n"
                "Please generate a path first by:\n"
                "1. Drawing a boundary\n"
                "2. Clicking 'Generate Path'")
            return

        # Confirm with user
        count = len(self._generated_path)
        reply = QMessageBox.question(self, "Apply Path to Mission",
            f"This will replace current waypoints with {count} generated waypoints.\n\n"
            f"The first waypoint will be set as TAKEOFF.\n"
            f"The last waypoint will be set as LAND.\n\n"
            f"Continue?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)

        if reply != QMessageBox.Yes:
            print("[UI] Apply path canceled by user")
            return

        # Convert generated path to mission waypoints
        wps = []
        for i, gp in enumerate(self._generated_path):
            if i == 0:
                cmd = CMD_TAKEOFF
            elif i == len(self._generated_path) - 1:
                cmd = CMD_LAND
            else:
                cmd = CMD_WAYPOINT
            wps.append(WP(cmd=cmd, lat=gp.lat, lon=gp.lon, alt=gp.alt))

        print(f"[UI] Applying {len(wps)} waypoints to mission")

        # Update table and maps
        self.table.blockSignals(True)
        self.table.load(wps)
        self.table.blockSignals(False)
        self.map.set_waypoints(wps)
        self.plan_map.set_waypoints(wps)

        self.path_status_label.setText(f"Path applied to mission: {len(wps)} waypoints ready to upload")
        self.path_status_label.setStyleSheet("QLabel { padding: 3px; background: #cce5ff; border: 1px solid #007bff; font-weight: bold; }")

        print(f"[UI] Path applied successfully! {len(wps)} waypoints in mission")
        self.worker.L(f"Generated path applied: {len(wps)} waypoints")

        QMessageBox.information(self, "Path Applied",
            f"Successfully applied {len(wps)} waypoints to mission!\n\n"
            f"The waypoints are now shown in the sequence table.\n"
            f"Click 'WRITE' to upload to the flight controller.")

    def _clear_boundary(self):
        """Clear boundary polygon and generated path"""
        print("[UI] CLEAR BOUNDARY button clicked")

        if not self.plan_map.get_boundary() and not hasattr(self, '_generated_path'):
            print("[UI] Nothing to clear")
            return

        reply = QMessageBox.question(self, "Clear Boundary",
            "Clear the boundary polygon and generated path?\n\n"
            "This will not affect mission waypoints.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply != QMessageBox.Yes:
            return

        # Clear boundary on both maps
        self.plan_map.clear_boundary()
        self.map.clear_boundary()
        self.plan_map.clear_generated_path()
        self.map.clear_generated_path()

        # Clear stored path
        if hasattr(self, '_generated_path'):
            self._generated_path = []
        if hasattr(self, '_path_stats'):
            self._path_stats = {}

        self._update_path_status()
        print("[UI] Boundary and path cleared")

    def _update_path_status(self):
        """Update path status label"""
        self.path_status_label.setText("Path Planning: Draw boundary (yellow) → Generate path (green) → Apply to mission")
        self.path_status_label.setStyleSheet("QLabel { padding: 3px; background: #f5f5dc; border: 1px solid #ddd; }")

    def _show_path_help(self):
        """Show path planning help dialog"""
        print("[UI] PATH HELP button clicked")
        help_text = """
<h2>Coverage Path Planning Guide</h2>

<h3>Overview</h3>
<p>This feature automatically generates a flight path to survey an area.
The drone will fly in a <b>lawnmower pattern</b> (back and forth) to cover the entire region,
while avoiding no-fly zones and respecting custom start/end points.</p>

<h3>Step-by-Step Instructions</h3>
<ol>
<li><b>Draw the Survey Boundary (Yellow Zone)</b>
   <ul>
   <li>Enable "Draw Boundary" checkbox</li>
   <li>Click on the map to add boundary points</li>
   <li>Add at least 3 points to form a polygon</li>
   <li>The boundary will be shown in <span style="color: #cc8800;">YELLOW</span></li>
   </ul>
</li>
<li><b>Add No-Fly Zones (Red Zones) - Optional</b>
   <ul>
   <li>Enable "Draw No-Fly Zone" checkbox</li>
   <li>Click on the map to add red zone points</li>
   <li>Add at least 3 points, then click "Finish Zone"</li>
   <li>The drone will AVOID these <span style="color: red;">RED</span> areas</li>
   <li>You can add multiple red zones</li>
   </ul>
</li>
<li><b>Set Start/End Points - Optional</b>
   <ul>
   <li>Enable "Set Start" and click on map to set start position</li>
   <li>Enable "Set End" and click on map to set end position</li>
   <li>The path will begin at START and end at END</li>
   <li>If not set, the planner will choose optimal points</li>
   </ul>
</li>
<li><b>Set Parameters</b>
   <ul>
   <li><b>Row Width:</b> Distance between parallel flight lines (meters)</li>
   <li>Smaller = better coverage, longer flight time</li>
   <li>Typical values: 5-20m depending on camera/sensor</li>
   <li><b>Altitude:</b> Flight height in meters</li>
   </ul>
</li>
<li><b>Generate Path</b>
   <ul>
   <li>Click "Generate Path" button</li>
   <li>The path will be shown in <span style="color: green;">GREEN</span></li>
   <li>Review the statistics (distance, time, waypoints)</li>
   <li>Path will avoid all red zones automatically</li>
   </ul>
</li>
<li><b>Apply to Mission</b>
   <ul>
   <li>Click "Apply to Mission" to convert path to waypoints</li>
   <li>Waypoints will appear in <span style="color: cyan;">CYAN</span></li>
   <li>First waypoint = TAKEOFF, Last = LAND</li>
   </ul>
</li>
<li><b>Upload to Drone</b>
   <ul>
   <li>Click "WRITE" to upload mission to flight controller</li>
   </ul>
</li>
</ol>

<h3>Map Legend</h3>
<ul>
<li><span style="color: #cc8800;">■ YELLOW</span> = Survey boundary (area to cover)</li>
<li><span style="color: red;">■ RED</span> = No-fly zones (drone will avoid)</li>
<li><span style="color: green;">■ GREEN</span> = Generated coverage path</li>
<li><span style="color: cyan;">■ CYAN</span> = Mission waypoints (uploaded to drone)</li>
<li><b>START</b> = Path start point (green marker)</li>
<li><b>END</b> = Path end point (red marker)</li>
</ul>

<h3>Tips</h3>
<ul>
<li>Larger row width = faster survey but less overlap</li>
<li>For photogrammetry, use 60-80% overlap (smaller row width)</li>
<li>Add red zones for obstacles, buildings, or restricted areas</li>
<li>Set start point near takeoff location for efficiency</li>
<li>Set end point near landing location</li>
<li>Use "Clear All" to reset everything and start over</li>
</ul>
"""
        msg = QMessageBox(self)
        msg.setWindowTitle("Path Planning Help")
        msg.setTextFormat(Qt.RichText)
        msg.setText(help_text)
        msg.setIcon(QMessageBox.Information)
        msg.exec_()

    # ------ map/table sync ------
    def _on_map_changed(self, wps: List[WP]):
        # A map added/changed waypoints: push to table and the other map
        self.table.blockSignals(True)
        self.table.load(wps)
        self.table.blockSignals(False)
        self.map.set_waypoints(wps)
        self.plan_map.set_waypoints(wps)

    def _on_table_changed(self, *_):
        # Any edit in the table should redraw both maps
        wps = self.table.to_list()
        self.map.set_waypoints(wps)
        self.plan_map.set_waypoints(wps)

    # ------ mission actions ------
    def _delete_selected(self):
        """Delete selected waypoints from table and update maps"""
        # Get selected rows
        selected_rows = sorted({idx.row() for idx in self.table.selectedIndexes()})

        if not selected_rows:
            print("[UI] No waypoints selected for deletion")
            QMessageBox.information(self, "Delete Waypoint",
                                   "Please select one or more waypoints to delete.\n\n"
                                   "Click on a row in the waypoint sequence table to select it.")
            return

        # Confirm deletion
        count = len(selected_rows)
        if count == 1:
            msg = f"Delete waypoint WP{selected_rows[0]}?"
        else:
            msg = f"Delete {count} selected waypoints?"

        reply = QMessageBox.question(self, "Delete Waypoints",
                                     f"{msg}\n\nThis cannot be undone.",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            print("[UI] Waypoint deletion canceled by user")
            return

        print(f"[UI] Deleting {count} waypoint(s) from sequence: {selected_rows}")
        self.table.del_selected()

        # Get updated waypoints and sync to maps
        wps = self.table.to_list()
        self.map.set_waypoints(wps)
        self.plan_map.set_waypoints(wps)

        # Renumber remaining waypoints if needed (table handles this automatically)
        print(f"[UI] Waypoints deleted. Remaining: {len(wps)}")

        # Show feedback
        if wps:
            self.worker.L(f"Deleted {count} waypoint(s). {len(wps)} remaining.")
        else:
            self.worker.L("All waypoints deleted. Plan is now empty.")

    def _read_mission(self):
        """Download mission from flight controller"""
        print("[UI] User requested READ mission from vehicle")
        reply = QMessageBox.information(self, "READ Mission",
                                        "This will download the mission from the flight controller.\n\n"
                                        "The current local waypoints on the map will be replaced.",
                                        QMessageBox.Ok | QMessageBox.Cancel)
        if reply == QMessageBox.Cancel:
            print("[UI] READ mission canceled by user")
            return
        print("[UI] Initiating mission READ from flight controller...")
        self.worker.request_read_mission()

    def _write_mission(self):
        """Upload mission to flight controller"""
        print("[UI] User requested WRITE mission to vehicle")
        items = self.table.to_list()
        if not items:
            print("[UI] WRITE aborted: No waypoints in sequence")
            QMessageBox.warning(self, "WRITE Mission", "No waypoints to upload.\n\nPlease add waypoints first.");
            return
        print(f"[UI] Preparing to upload {len(items)} waypoint(s)")
        self.map.set_waypoints(items); self.plan_map.set_waypoints(items)
        reply = QMessageBox.information(self, "WRITE Mission",
                                        f"Ready to upload {len(items)} waypoint(s) to the flight controller.\n\n"
                                        f"This will overwrite any existing mission on the vehicle.",
                                        QMessageBox.Ok | QMessageBox.Cancel)
        if reply == QMessageBox.Cancel:
            print("[UI] WRITE mission canceled by user")
            return
        print(f"[UI] Initiating mission WRITE to flight controller: {len(items)} waypoint(s)")
        self.worker.request_write_mission(items)

    def _clear_vehicle(self):
        """Clear all mission waypoints from flight controller"""
        print("[UI] User requested CLEAR vehicle mission")
        reply = QMessageBox.question(self, "CLEAR Vehicle Mission",
                                     "This will permanently remove ALL mission waypoints from the flight controller.\n\n"
                                     "Are you sure you want to continue?",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            print("[UI] CLEAR vehicle mission canceled by user")
            return
        print("[UI] Initiating CLEAR mission on flight controller...")
        self.worker.request_clear_mission()

    def _clear_local(self):
        """Clear local waypoints from map and table"""
        print("[UI] User requested CLEAR local plan")
        if self.table.rowCount() == 0:
            print("[UI] No waypoints to clear")
            QMessageBox.information(self, "CLEAR Local Plan", "No waypoints to clear.")
            return
        reply = QMessageBox.question(self, "CLEAR Local Plan",
                                     "This will clear all waypoints from the map and sequence table.\n\n"
                                     "The flight controller mission will NOT be affected.\n\n"
                                     "Continue?",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            print("[UI] CLEAR local plan canceled by user")
            return
        print("[UI] Clearing local waypoints from map and table...")
        self.table.clear_all()
        self.map.set_waypoints([])
        self.plan_map.set_waypoints([])
        print("[UI] Local plan cleared successfully")
        QMessageBox.information(self, "Local Plan Cleared", "Local waypoints cleared successfully.")

    # ------ mission results ------
    def _on_mission_read(self, wps: List[WP]):
        """Called when mission is successfully read from flight controller"""
        print(f"[UI] Mission READ completed: Received {len(wps)} waypoint(s)")
        if not wps:
            print("[UI] No waypoints on vehicle - mission is empty")
            QMessageBox.information(self, "READ Complete",
                                   "No waypoints found on the flight controller.\n\n"
                                   "The vehicle has no mission loaded.")
        else:
            print(f"[UI] Displaying {len(wps)} waypoint(s) on map and in sequence table")
            for i, wp in enumerate(wps):
                print(f"  WP{i}: Cmd={wp.cmd}, Lat={wp.lat:.7f}, Lon={wp.lon:.7f}, Alt={wp.alt:.1f}m")
            QMessageBox.information(self, "READ Complete",
                                   f"Successfully downloaded {len(wps)} waypoint(s) from the flight controller.\n\n"
                                   f"The waypoints are now displayed on the map and in the sequence table.")
        # Update table and maps with downloaded waypoints
        self.table.blockSignals(True)
        self.table.load(wps)
        self.table.blockSignals(False)
        self.map.set_waypoints(wps)
        self.plan_map.set_waypoints(wps)
        print("[UI] Map and table updated with downloaded waypoints")

    def _on_mission_write_ok(self):
        """Called when mission is successfully written to flight controller"""
        print("[UI] Mission WRITE completed successfully")
        QMessageBox.information(self, "WRITE Complete",
                               "Mission successfully uploaded to the flight controller!\n\n"
                               "The waypoints are now saved on the vehicle.")

    def _on_mission_cleared(self):
        """Called when mission is successfully cleared from flight controller"""
        print("[UI] Mission CLEAR completed - vehicle mission is now empty")
        # Clear local display to match vehicle state
        self.table.clear_all()
        self.map.set_waypoints([])
        self.plan_map.set_waypoints([])
        print("[UI] Local waypoints cleared to match vehicle state")
        QMessageBox.information(self, "CLEAR Complete",
                               "All mission waypoints have been removed from the flight controller.\n\n"
                               "Local waypoints have also been cleared.")

    def _on_mission_error(self, msg: str):
        """Called when a mission operation fails"""
        print(f"[UI] Mission operation ERROR: {msg}")
        QMessageBox.critical(self, "Mission Error",
                            f"Mission operation failed:\n\n{msg}\n\n"
                            f"Please check the connection and try again.")

    # ------ helpers ------
    def _set_home_wp0(self):
        """Set or create home/takeoff waypoint (WP0).

        If GPS position is available, offers to use current position.
        Otherwise uses map center or existing first waypoint position.
        """
        wps = self.table.to_list()
        current_lat = self.map.center_lat
        current_lon = self.map.center_lon
        default_alt = 30.0

        # Try to get GPS position if connected
        gps_lat = None
        gps_lon = None
        # Check if we have recent GPS data from HUD
        if self.map.hud and hasattr(self.map.hud, 'gps'):
            gps_text = self.map.hud.gps.text()
            if gps_text and "3D" in gps_text:
                # We have good GPS fix - use map center which should be tracking position
                gps_lat = current_lat
                gps_lon = current_lon

        if wps:
            # Already have waypoints - modify WP0 to be TAKEOFF
            old_wp = wps[0]

            # Ask user what to do
            options = []
            if gps_lat is not None:
                options.append(f"Use current GPS position ({gps_lat:.6f}, {gps_lon:.6f})")
            options.append(f"Keep existing position ({old_wp.lat:.6f}, {old_wp.lon:.6f})")
            options.append(f"Use map center ({current_lat:.6f}, {current_lon:.6f})")

            # Simple dialog - just set as takeoff
            wps[0].cmd = CMD_TAKEOFF
            print(f"[UI] Set WP0 as TAKEOFF at ({old_wp.lat:.7f}, {old_wp.lon:.7f})")
            self.worker.L(f"WP0 set as TAKEOFF position")

        else:
            # No waypoints yet - create new home waypoint
            if gps_lat is not None:
                # Use GPS position
                home_lat, home_lon = gps_lat, gps_lon
                print(f"[UI] Creating home waypoint at GPS position: ({home_lat:.7f}, {home_lon:.7f})")
            else:
                # Use map center
                home_lat, home_lon = current_lat, current_lon
                print(f"[UI] Creating home waypoint at map center: ({home_lat:.7f}, {home_lon:.7f})")

            wps = [WP(cmd=CMD_TAKEOFF, lat=home_lat, lon=home_lon, alt=default_alt)]
            self.worker.L(f"Created home waypoint (TAKEOFF) at ({home_lat:.6f}, {home_lon:.6f})")

        # Update table and maps
        self.table.blockSignals(True)
        self.table.load(wps)
        self.table.blockSignals(False)
        self.map.set_waypoints(wps)
        self.plan_map.set_waypoints(wps)

        # Center plan map on home if this is the first waypoint
        if len(wps) == 1:
            self.plan_map.recenter(wps[0].lat, wps[0].lon)
            self.plan_map.zoom = max(15, self.plan_map.zoom)
            self.plan_map.update()

    def _open_mbtiles(self):
        fn, _ = QFileDialog.getOpenFileName(self, "Open MBTiles", "", "MBTiles (*.mbtiles)")
        if not fn: return
        try:
            if self.map.mbtiles: self.map.mbtiles.close()
            if self.plan_map.mbtiles: self.plan_map.mbtiles.close()
            prov = MBTilesProvider(Path(fn))
            if not prov.valid:
                QMessageBox.warning(self,"MBTiles","Invalid file."); return
            # Use separate providers so their sqlite connections don't fight
            self.map.mbtiles = prov
            self.plan_map.mbtiles = MBTilesProvider(Path(fn))
            self.map.update(); self.plan_map.update()
        except Exception as e:
            QMessageBox.critical(self,"MBTiles",str(e))

    # ------ autopan + HUD ------
    def _on_user_interaction(self, ms: int):
        now = time.time() * 1000.0
        ms = max(ms, self.FOLLOW_SUSPEND_MS)
        self._suppress_until = max(self._suppress_until, now + ms)

    def _on_status(self, d: dict):
        if self.autopan.isChecked():
            now = time.time() * 1000.0
            if now >= self._suppress_until:
                lat = d.get("_gps_lat")
                lon = d.get("_gps_lon")
                if lat is not None and lon is not None:
                    self.map.recenter(lat, lon)
                    # keep the plan map near the same area but do not fight user during planning

        # HUD overlay on main map
        if self.map.hud:
            # Update mode and armed status with visual indicator
            if "mode" in d and "armed" in d:
                self.map.hud.set_mode_armed(d['mode'], d['armed'])

            # GPS status
            if d.get("_fix") is not None or d.get("_sats") is not None:
                fix = d.get('_fix', '?')
                sats = d.get('_sats', '?')
                # Color code GPS status
                if fix in ['3D', 'RTK_FIXED', 'RTK_FLOAT', 'DGPS']:
                    self.map.hud.gps.setStyleSheet("color: #00ff00; font-size: 12px;")
                elif fix == '2D':
                    self.map.hud.gps.setStyleSheet("color: #ffff00; font-size: 12px;")
                else:
                    self.map.hud.gps.setStyleSheet("color: #ff6666; font-size: 12px;")
                self.map.hud.gps.setText(f"{fix} / {sats} sats")

            # Altitude
            if d.get("_alt") is not None:
                self.map.hud.alt.setText(f"{d['_alt']:.1f} m")

            # Ground speed
            if d.get("_gs") is not None:
                self.map.hud.gs.setText(f"{d['_gs']:.1f} m/s")

            # Vertical speed with color coding
            if d.get("_vz") is not None:
                vz = d['_vz']
                if vz > 0.5:
                    self.map.hud.vz.setStyleSheet("color: #00ff00; font-size: 12px;")
                elif vz < -0.5:
                    self.map.hud.vz.setStyleSheet("color: #ff6666; font-size: 12px;")
                else:
                    self.map.hud.vz.setStyleSheet("color: #ffffff; font-size: 12px;")
                self.map.hud.vz.setText(f"{vz:+.1f} m/s")

            # Yaw/Heading
            if d.get("_yaw") is not None:
                self.map.hud.yaw.setText(f"{d['_yaw']:.1f}°")

            # Battery with voltage warning
            if "battery" in d:
                vb = d.get("_vb")
                cpu = d.get("_cpu")
                if vb is not None and cpu is not None:
                    # Color code battery voltage (adjust thresholds as needed)
                    if vb < 21.0:  # Critical
                        self.map.hud.bat.setStyleSheet("color: #ff0000; font-size: 12px; font-weight: bold;")
                    elif vb < 22.5:  # Warning
                        self.map.hud.bat.setStyleSheet("color: #ffff00; font-size: 12px;")
                    else:  # OK
                        self.map.hud.bat.setStyleSheet("color: #00ff00; font-size: 12px;")
                    self.map.hud.bat.setText(f"{vb:.2f} V | CPU {cpu:.0f}%")
                else:
                    self.map.hud.bat.setText(d["battery"])

        # Update AHRS/Artificial Horizon
        if self.map.ahrs:
            r = d.get("_roll")
            p = d.get("_pitch")
            y = d.get("_yaw")
            if (r is not None) or (p is not None) or (y is not None):
                self.map.ahrs.set_attitude(r or 0.0, p or 0.0, y or 0.0)

# ---------------- Main window ----------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("rifm (with Planner)")
        self.resize(1200, 780)
        self.statusbar = QStatusBar(self); self.setStatusBar(self.statusbar)

        self.worker = MavWorker(demo_mode=False)

        self.stack = QStackedWidget()
        self.connect_page = ConnectPage(self.worker)
        self.map_page    = MapDashboard(self.worker, statusbar_provider=lambda: self.statusBar())
        self.stack.addWidget(self.connect_page)  # 0
        self.stack.addWidget(self.map_page)      # 1
        self.setCentralWidget(self.stack)
        self.stack.setCurrentIndex(0)

        self.connect_page.request_go_map.connect(lambda: self.stack.setCurrentIndex(1))

        if not QSslSocket.supportsSsl():
            self.statusBar().showMessage("SSL not available: HTTPS tiles cannot load. Install OpenSSL 1.1.x for Qt5.", 8000)

    def closeEvent(self, e):
        try:
            if self.worker.isRunning():
                self.worker.stop(); self.worker.wait(1500)
        except Exception:
            pass
        e.accept()

# ---------------- Entrypoint ----------------
if __name__ == "__main__":
    print("\n" + "="*60)
    print("   Ground Control Station (GCS) - Mission Planner")
    print("   Version: UI10 - Coverage Path Planning + No-Fly Zones")
    print("="*60)
    print("[System] Initializing application...")

    # Print path planning help on startup
    print("\n" + "-"*60)
    print("   COVERAGE PATH PLANNING FEATURES")
    print("-"*60)
    print("   This version includes automatic path generation with:")
    print("   - Survey boundary (YELLOW zone)")
    print("   - No-fly zones (RED zones) - drone avoids these areas")
    print("   - Custom START and END points")
    print("   - Configurable row width and altitude")
    print("")
    print("   HOW TO USE:")
    print("   1. Go to the 'Plan' tab")
    print("   2. Draw survey boundary (YELLOW) - at least 3 points")
    print("   3. [Optional] Draw no-fly zones (RED) - obstacles to avoid")
    print("   4. [Optional] Set START and END points")
    print("   5. Set Row Width (meters between flight lines)")
    print("   6. Click 'Generate Path' - creates GREEN path")
    print("   7. Click 'Apply to Mission' to convert to waypoints")
    print("   8. Click 'WRITE' to upload to drone")
    print("")
    print("   MAP LEGEND:")
    print("   - YELLOW = Survey boundary (area to cover)")
    print("   - RED    = No-fly zones (drone avoids these)")
    print("   - GREEN  = Generated flight path")
    print("   - CYAN   = Mission waypoints")
    print("   - START  = Path starting point (green marker)")
    print("   - END    = Path ending point (red marker)")
    print("-"*60 + "\n")

    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    print("[System] Creating main window...")
    w = MainWindow()
    w.show()
    print("[System] Application ready. Connect to flight controller to begin.\n")
    sys.exit(app.exec_())
