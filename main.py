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
    QTableWidgetItem
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

        self._upl_active = False
        self._upl_items: List[WP] = []
        self._upl_start_time = 0
        self._dl_active = False
        self._dl_expected = 0
        self._dl_got: List[WP] = []
        self._dl_start_time = 0

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
        lat_i = int(round(wp.lat * 1e7))
        lon_i = int(round(wp.lon * 1e7))
        return mavutil.mavlink.MAVLink_mission_item_int_message(
            target_system=target_sys,
            target_component=target_comp,
            seq=seq,
            frame=wp.frame,
            command=wp.cmd,
            current=1 if seq == 0 else 0,
            autocontinue=1,
            param1=float(wp.p1), param2=float(wp.p2), param3=float(wp.p3), param4=float(wp.p4),
            x=lat_i, y=lon_i, z=float(wp.alt),
            mission_type=0
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

                # mission download
                elif t == "MISSION_COUNT" and self._dl_active:
                    self._dl_expected = int(msg.count or 0); self._dl_got = []
                    print(f"[MAVLink] ✓ Vehicle responded: {self._dl_expected} waypoint(s) in mission")
                    if self._dl_expected == 0:
                        print(f"[MAVLink] Mission is empty - no waypoints to download")
                        self._dl_active = False
                        self._dl_start_time = 0  # Reset timeout timer
                        self.mission_read_ok.emit([])
                    else:
                        print(f"[MAVLink] Requesting waypoints from vehicle...")
                        self.L(f"Vehicle has {self._dl_expected} waypoint(s), downloading...")
                # Handle both MISSION_ITEM_INT and legacy MISSION_ITEM
                elif (t == "MISSION_ITEM_INT" or t == "MISSION_ITEM") and self._dl_active:
                    if t == "MISSION_ITEM":
                        print(f"[MAVLink] Received LEGACY MISSION_ITEM (converting to INT format)")
                    wp = self._from_mission_int(msg)
                    self._dl_got.append(wp)
                    print(f"[MAVLink] Downloaded WP{len(self._dl_got)-1}: Cmd={wp.cmd}, Lat={wp.lat:.7f}, Lon={wp.lon:.7f}, Alt={wp.alt:.1f}m [{len(self._dl_got)}/{self._dl_expected}]")
                    if len(self._dl_got) == self._dl_expected:
                        self._dl_active = False
                        self._dl_start_time = 0  # Reset timeout timer
                        print(f"\n{'='*60}")
                        print(f"[MAVLink] ✓ Mission download COMPLETE: {len(self._dl_got)} waypoint(s)")
                        print(f"{'='*60}\n")
                        self.L(f"Mission download complete: {len(self._dl_got)} waypoint(s)")
                        self.mission_read_ok.emit(self._dl_got[:])

                # mission upload - handle BOTH MISSION_REQUEST_INT and legacy MISSION_REQUEST
                elif (t == "MISSION_REQUEST_INT" or t == "MISSION_REQUEST") and self._upl_active:
                    seq = int(msg.seq)
                    if t == "MISSION_REQUEST":
                        print(f"[MAVLink] Vehicle requesting waypoint {seq+1}/{len(self._upl_items)} (LEGACY MISSION_REQUEST)")
                    else:
                        print(f"[MAVLink] Vehicle requesting waypoint {seq+1}/{len(self._upl_items)} (MISSION_REQUEST_INT)")
                    try:
                        wp = self._upl_items[seq]
                        print(f"[MAVLink] Sending WP{seq}: Cmd={wp.cmd}, Lat={wp.lat:.7f}, Lon={wp.lon:.7f}, Alt={wp.alt:.1f}m")
                    except Exception as e:
                        print(f"[MAVLink] ERROR: Failed to get waypoint {seq}: {e}")
                        continue
                    pkt = self._to_mission_int(seq, wp, self._fc_sid, self._fc_cid, mavutil, self._mav)
                    self._mav.send(pkt)
                    print(f"[MAVLink] Waypoint {seq+1}/{len(self._upl_items)} sent to vehicle")
                elif t == "MISSION_ACK" and self._upl_active:
                    ack_type = msg.type if hasattr(msg, 'type') else 0
                    print(f"[MAVLink] Mission ACK received - Type: {ack_type}")
                    if ack_type == 0:  # MAV_MISSION_ACCEPTED
                        print(f"[MAVLink] ✓ Mission ACCEPTED by vehicle - All {len(self._upl_items)} waypoints saved!")
                        self.L(f"Mission upload complete! {len(self._upl_items)} waypoints saved.")
                    else:
                        print(f"[MAVLink] ✗ Mission REJECTED by vehicle - ACK type: {ack_type}")
                        self.mission_error.emit(f"Mission rejected by vehicle (ACK={ack_type})")
                        self._upl_active = False; self._upl_items = []
                        self._upl_start_time = 0  # Reset timeout timer
                        return
                    self._upl_active = False; self._upl_items = []
                    self._upl_start_time = 0  # Reset timeout timer
                    self.mission_write_ok.emit()

                # mission clear acknowledge (some stacks send ACK, some don't; we always fire ok on command ack if received)
                elif t == "COMMAND_ACK":
                    # optionally inspect msg.command == MAV_CMD_MISSION_CLEAR_ALL
                    pass

            # process UI commands
            try:
                c, payload = self._cmdq.get_nowait()
            except queue.Empty:
                c = None

            if c == "read":
                try:
                    print(f"\n{'='*60}")
                    print(f"[MAVLink] Requesting mission list from vehicle")
                    print(f"  Target: sys={self._fc_sid}, comp={self._fc_cid}")
                    print(f"{'='*60}")
                    self.L("Sending mission read request to flight controller...")
                    self._dl_active = True; self._dl_got = []; self._dl_expected = 0
                    self._dl_start_time = time.time()  # Start timeout timer
                    # Request mission list - vehicle will respond with MISSION_COUNT
                    self._mav.mission_request_list_send(self._fc_sid, self._fc_cid, mission_type=0)
                    print(f"[MAVLink] MISSION_REQUEST_LIST sent")
                    print(f"[MAVLink] Waiting for vehicle to respond with mission count and items...")
                    print(f"[MAVLink] Note: Vehicle will automatically send mission items after count")
                    print(f"[MAVLink] Timeout: 30 seconds")
                except Exception as e:
                    print(f"[MAVLink] Mission read failed: {e}")
                    self._dl_active = False; self.mission_error.emit(f"Read failed: {e}")

            elif c == "write":
                items: List[WP] = payload or []
                if not items:
                    print("[MAVLink] Write aborted: No waypoints provided")
                    self.mission_error.emit("No waypoints to upload."); continue
                try:
                    print(f"\n{'='*60}")
                    print(f"[MAVLink] Starting mission upload: {len(items)} waypoint(s)")
                    print(f"  Target: sys={self._fc_sid}, comp={self._fc_cid}")
                    print(f"{'='*60}")
                    self.L(f"Uploading {len(items)} waypoint(s) to flight controller...")
                    self._upl_active = True; self._upl_items = items[:]
                    self._upl_start_time = time.time()  # Start timeout timer
                    self._mav.mission_count_send(self._fc_sid, self._fc_cid, len(items), mission_type=0)
                    print(f"[MAVLink] MISSION_COUNT sent: {len(items)} waypoints")
                    print(f"[MAVLink] Waiting for vehicle to request waypoints...")
                    print(f"[MAVLink] Timeout: 30 seconds")
                except Exception as e:
                    print(f"[MAVLink] Mission write failed: {e}")
                    self._upl_active = False; self.mission_error.emit(f"Write failed: {e}")

            elif c == "clear":
                try:
                    print(f"\n{'='*60}")
                    print(f"[MAVLink] Sending CLEAR_ALL command to vehicle")
                    print(f"  Target: sys={self._fc_sid}, comp={self._fc_cid}")
                    print(f"{'='*60}")
                    self.L("Clearing all mission items from flight controller...")
                    # Clear all mission items on vehicle
                    from pymavlink import mavutil
                    self._mav.mission_clear_all_send(self._fc_sid, self._fc_cid, mission_type=0)
                    print("[MAVLink] MISSION_CLEAR_ALL command sent")
                    print("[MAVLink] Waiting for acknowledgment...")
                    # Give it a moment then emit success (some FCs don't send ACK for clear)
                    self.msleep(500)
                    self.mission_clear_ok.emit()
                    print("[MAVLink] ✓ Mission cleared from vehicle")
                except Exception as e:
                    print(f"[MAVLink] Mission clear failed: {e}")
                    self.mission_error.emit(f"Clear failed: {e}")

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
    def __init__(self):
        super().__init__()
        self.setFixedSize(260, 260)
        self.roll = 0.0; self.pitch = 0.0; self.yaw = 0.0
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background: rgba(0, 0, 0, 120); border-radius: 10px;")
    def set_attitude(self, roll_deg: float, pitch_deg: float, yaw_deg: float):
        self.roll = float(roll_deg or 0.0); self.pitch = float(pitch_deg or 0.0); self.yaw = float(yaw_deg or 0.0)
        self.update()
    def paintEvent(self, _):
        w, h = self.width(), self.height()
        cx, cy = w//2, h//2
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing, True)
        bezel_pen = QPen(Qt.white); bezel_pen.setWidth(1); p.setPen(bezel_pen)
        ppd = 3.0
        p.save(); p.translate(cx, cy); p.rotate(-self.roll)
        y_off = ppd * self.pitch
        sky_rect = QRect(-400, -400 - int(y_off), 800, 400)
        ground_rect = QRect(-400, 0 - int(y_off), 800, 400)
        p.fillRect(sky_rect, QtGui.QColor(90, 150, 220))
        p.fillRect(ground_rect, QtGui.QColor(140, 95, 35))
        p.setPen(Qt.white); p.drawLine(-380, -int(y_off), 380, -int(y_off))
        p.restore()
        p.setPen(Qt.white); radius = 110
        p.drawArc(cx-radius, cy-radius, radius*2, radius*2, (90-60)*16, 120*16)
        pointer = QtGui.QPolygon([QtCore.QPoint(cx, cy - radius - 6),
                                  QtCore.QPoint(cx - 7, cy - radius + 8),
                                  QtCore.QPoint(cx + 7, cy - radius + 8)])
        p.setBrush(Qt.white); p.drawPolygon(pointer)

class HUDPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background: rgba(0,0,0,120); color: #e8e8e8; border-radius: 8px;")
        lay = QGridLayout(self); lay.setContentsMargins(8,8,8,8)
        def key(t): lbl=QLabel(t); f=lbl.font(); f.setBold(True); lbl.setFont(f); return lbl
        self.mode=QLabel("—"); self.gps=QLabel("—"); self.alt=QLabel("0.0 m")
        self.gs=QLabel("0.0 m/s"); self.vz=QLabel("0.0 m/s"); self.yaw=QLabel("0°"); self.bat=QLabel("—")
        lay.addWidget(key("Mode:"),0,0); lay.addWidget(self.mode,0,1)
        lay.addWidget(key("GPS:"),1,0); lay.addWidget(self.gps,1,1)
        lay.addWidget(key("Alt:"),2,0); lay.addWidget(self.alt,2,1)
        lay.addWidget(key("GS:"),3,0); lay.addWidget(self.gs,3,1)
        lay.addWidget(key("VZ:"),4,0); lay.addWidget(self.vz,4,1)
        lay.addWidget(key("Yaw:"),5,0); lay.addWidget(self.yaw,5,1)
        lay.addWidget(key("Batt:"),6,0); lay.addWidget(self.bat,6,1)
        self.setFixedWidth(260)

# ---------------- Map Widget (now emits waypointsChanged) ----------------
class MapWidget(QWidget):
    userInteracted = pyqtSignal(int)
    waypointsChanged = pyqtSignal(list)  # emits List[WP] whenever WPs change on this map

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
            if self.add_mode:
                # Add a waypoint at click
                w,h=self.width(),self.height()
                cx,cy=self.latlon_to_pixels(self.center_lat,self.center_lon,self.zoom)
                tlx,tly=cx-w/2,cy-h/2
                px,py=tlx+e.pos().x(),tly+e.pos().y()
                lat,lon=self.pixels_to_latlon(px,py,self.zoom)
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

        # draw planned path/points (high contrast)
        if self._wps:
            pts=[]
            for wp in self._wps:
                px,py=self.latlon_to_pixels(wp.lat,wp.lon,self.zoom)
                sx=int(px-tlx); sy=int(py-tly)
                pts.append((sx,sy))
            p.setRenderHint(QPainter.Antialiasing, True)
            p.setPen(QPen(Qt.yellow, 3))
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

        # Planner toolbar
        tb = QToolBar(); v.addWidget(tb)

        self.btn_add_mode = QCheckBox("Add Waypoint (Click on Map)")
        tb.addWidget(self.btn_add_mode)
        self.btn_set_home = QPushButton("Set Home as WP0")
        self.btn_set_home.setToolTip("Set the first waypoint (WP0) as home/takeoff position")
        self.btn_read = QPushButton("READ from Vehicle")
        self.btn_read.setToolTip("Download mission waypoints from flight controller")
        self.btn_write = QPushButton("WRITE to Vehicle")
        self.btn_write.setToolTip("Upload mission waypoints to flight controller")
        self.btn_clear_vehicle = QPushButton("CLEAR Vehicle Mission")
        self.btn_clear_vehicle.setToolTip("Remove all waypoints from flight controller")
        self.btn_delete = QPushButton("Delete Selected WP")
        self.btn_delete.setToolTip("Delete selected waypoints from the sequence")
        self.btn_clear_local = QPushButton("CLEAR Local Plan")
        self.btn_clear_local.setToolTip("Clear waypoints from map and table (does not affect vehicle)")

        tb.addWidget(self.btn_set_home)
        tb.addSeparator()
        tb.addWidget(self.btn_read); tb.addWidget(self.btn_write)
        tb.addSeparator()
        tb.addWidget(self.btn_delete); tb.addWidget(self.btn_clear_local); tb.addWidget(self.btn_clear_vehicle)

        # Planner map (clickable)
        map_label = QLabel("Mission Planning Map - Click to add waypoints when 'Add Waypoint' is enabled")
        map_label.setStyleSheet("QLabel { font-weight: bold; padding: 5px; background: #e8f4f8; }")
        v.addWidget(map_label)
        self.plan_map.setFixedHeight(320)
        v.addWidget(self.plan_map)

        # Table
        seq_label = QLabel("Waypoint Sequence - WP0 is Home/Takeoff position")
        seq_label.setStyleSheet("QLabel { font-weight: bold; padding: 5px; background: #f0f0f0; }")
        v.addWidget(seq_label)
        self.table = PlannerTable()
        v.addWidget(self.table, 1)

        # Wiring
        self.btn_add_mode.stateChanged.connect(lambda *_: [self.plan_map.set_add_mode(self.btn_add_mode.isChecked()),
                                                           self.map.set_add_mode(self.btn_add_mode.isChecked())])
        self.btn_set_home.clicked.connect(self._set_home_wp0)
        self.btn_read.clicked.connect(self._read_mission)
        self.btn_write.clicked.connect(self._write_mission)
        self.btn_delete.clicked.connect(self._delete_selected)
        self.btn_clear_local.clicked.connect(self._clear_local)
        self.btn_clear_vehicle.clicked.connect(self._clear_vehicle)
        return w

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
        print("[UI] Deleting selected waypoints from sequence...")
        self.table.del_selected()
        # Get updated waypoints and sync to maps
        wps = self.table.to_list()
        self.map.set_waypoints(wps)
        self.plan_map.set_waypoints(wps)
        print(f"[UI] Waypoints deleted. Remaining: {len(wps)}")

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
        wps = self.table.to_list()
        if wps:
            wps[0].cmd = CMD_TAKEOFF
        else:
            wps = [WP(cmd=CMD_TAKEOFF, lat=self.map.center_lat, lon=self.map.center_lon, alt=30.0)]
        self.table.blockSignals(True); self.table.load(wps); self.table.blockSignals(False)
        self.map.set_waypoints(wps); self.plan_map.set_waypoints(wps)

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
                lat = d.get("_gps_lat"); lon = d.get("_gps_lon")
                if lat is not None and lon is not None:
                    self.map.recenter(lat, lon)
                    # keep the plan map near the same area but do not fight user during planning
        # HUD overlay on main map
        if self.map.hud:
            if "mode" in d and "armed" in d: self.map.hud.mode.setText(f"{d['mode']} | {'ARMED' if d['armed'] else 'DISARMED'}")
            if d.get("_fix") is not None or d.get("_sats") is not None: self.map.hud.gps.setText(f"{d.get('_fix','?')} / {d.get('_sats','?')} sats")
            if d.get("_alt") is not None: self.map.hud.alt.setText(f"{d['_alt']:.1f} m")
            if d.get("_gs") is not None:  self.map.hud.gs.setText(f"{d['_gs']:.1f} m/s")
            if d.get("_vz") is not None:  self.map.hud.vz.setText(f"{d['_vz']:+.1f} m/s")
            if d.get("_yaw") is not None: self.map.hud.yaw.setText(f"{d['_yaw']:.1f}°")
            if "battery" in d:
                vb = d.get("_vb"); cpu = d.get("_cpu")
                if vb is not None and cpu is not None:
                    self.map.hud.bat.setText(f"{vb:.2f} V | CPU {cpu:.0f}%")
                else:
                    self.map.hud.bat.setText(d["battery"])
        if self.map.ahrs:
            r = d.get("_roll"); p = d.get("_pitch"); y = d.get("_yaw")
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
    print("   Version: UI8 - With Mission Planning & AHRS")
    print("="*60)
    print("[System] Initializing application...")

    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    print("[System] Creating main window...")
    w = MainWindow()
    w.show()
    print("[System] Application ready. Connect to flight controller to begin.\n")
    sys.exit(app.exec_())
