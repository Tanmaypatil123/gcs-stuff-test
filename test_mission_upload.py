#!/usr/bin/env python3
"""
Mission Upload Test Script
Tests waypoint upload to flight controller with detailed diagnostics
"""

import sys
import time
from pymavlink import mavutil
from dataclasses import dataclass

@dataclass
class WP:
    """Waypoint data structure"""
    cmd: int
    lat: float
    lon: float
    alt: float
    p1: float = 0.0
    p2: float = 0.0
    p3: float = 0.0
    p4: float = 0.0
    frame: int = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT

# MAVLink command IDs
CMD_TAKEOFF = 22
CMD_WAYPOINT = 16
CMD_LAND = 21
CMD_RTL = 20

# Dummy waypoints for testing (around Pune, India area)
DUMMY_WAYPOINTS = [
    WP(cmd=CMD_TAKEOFF, lat=18.5204, lon=73.8567, alt=30.0, p1=15.0),  # Takeoff with 15° pitch
    WP(cmd=CMD_WAYPOINT, lat=18.5214, lon=73.8577, alt=50.0),
    WP(cmd=CMD_WAYPOINT, lat=18.5224, lon=73.8587, alt=50.0),
    WP(cmd=CMD_WAYPOINT, lat=18.5234, lon=73.8597, alt=50.0),
    WP(cmd=CMD_LAND, lat=18.5244, lon=73.8607, alt=0.0),
]

def decode_ack_type(ack_type):
    """Decode MAVLink mission ACK types"""
    ack_types = {
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
        10: "MAV_MISSION_INVALID_PARAM5_X - x/param5 invalid",
        11: "MAV_MISSION_INVALID_PARAM6_Y - y/param6 invalid",
        12: "MAV_MISSION_INVALID_PARAM7 - z/param7 invalid",
        13: "MAV_MISSION_INVALID_SEQUENCE - Received waypoint out of sequence",
        14: "MAV_MISSION_DENIED - Not accepting any mission commands",
        15: "MAV_MISSION_OPERATION_CANCELLED - Mission operation cancelled",
    }
    return ack_types.get(ack_type, f"UNKNOWN ACK TYPE: {ack_type}")

def to_mission_int(seq, wp, target_sys, target_comp):
    """Convert waypoint to MISSION_ITEM_INT message"""
    lat_int = int(round(wp.lat * 1e7))
    lon_int = int(round(wp.lon * 1e7))

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
        x=lat_int,
        y=lon_int,
        z=float(wp.alt),
        mission_type=0
    )

def connect_to_vehicle(connection_string):
    """Connect to vehicle and return MAVLink connection"""
    print(f"\n{'='*60}")
    print(f"Connecting to vehicle: {connection_string}")
    print(f"{'='*60}\n")

    try:
        import serial
        import re

        # Windows COM port handling (same as main.py)
        def win_path(port: str) -> str:
            m = re.match(r"^COM(\d+)$", port.upper())
            if m and int(m.group(1)) >= 10:
                return r"\\.\%s" % port.upper()
            return port

        # Open serial connection directly
        port = win_path(connection_string)
        print(f"Opening serial port: {port}")
        ser = serial.Serial(port=port, baudrate=115200, timeout=0.05,
                           write_timeout=0.5, rtscts=False, dsrdtr=False, xonxoff=False)

        # Create MAVLink connection
        from pymavlink.dialects.v20 import ardupilotmega as mavlink2
        mav = mavlink2.MAVLink(ser)
        mav.robust_parsing = True
        mav.srcSystem = 255
        mav.srcComponent = 190

        print("Waiting for heartbeat...")

        # Wait for heartbeat (same logic as main.py)
        start = time.time()
        fc_sid = None
        fc_cid = None

        while (time.time() - start < 12.0):
            data = ser.read(512)
            if not data:
                time.sleep(0.01)
                continue

            for byte in data:
                msg = mav.parse_char(bytes([byte]))
                if not msg:
                    continue

                if msg.get_type() == "HEARTBEAT":
                    sid = msg.get_srcSystem()
                    cid = msg.get_srcComponent()

                    if sid != 255:  # Not from GCS
                        fc_sid = sid
                        fc_cid = cid

                        print(f"✓ Connected!")
                        print(f"  System ID: {fc_sid}")
                        print(f"  Component ID: {fc_cid}")
                        print(f"  Vehicle Type: {msg.type}")
                        print(f"  Autopilot: {msg.autopilot}")
                        print(f"  Base Mode: {msg.base_mode}")
                        print(f"  System Status: {msg.system_status}")

                        # Create a wrapper object with the connection info
                        class MAVConnection:
                            def __init__(self, serial_conn, mavlink_obj, sys_id, comp_id):
                                self.serial = serial_conn
                                self.mav = mavlink_obj
                                self.target_system = sys_id
                                self.target_component = comp_id

                            def recv_match(self, type=None, blocking=False, timeout=None):
                                """Receive and parse messages"""
                                start_time = time.time()
                                while True:
                                    if timeout and (time.time() - start_time > timeout):
                                        return None

                                    data = self.serial.read(512)
                                    if not data:
                                        if not blocking:
                                            return None
                                        time.sleep(0.01)
                                        continue

                                    for byte in data:
                                        msg = self.mav.parse_char(bytes([byte]))
                                        if msg:
                                            if type is None or msg.get_type() == type:
                                                return msg

                        return MAVConnection(ser, mav, fc_sid, fc_cid)

        print("✗ No heartbeat received")
        ser.close()
        return None

    except Exception as e:
        print(f"✗ Connection failed: {e}")
        import traceback
        traceback.print_exc()
        return None

def upload_mission(mav, waypoints):
    """Upload mission to vehicle with detailed logging"""
    target_sys = mav.target_system
    target_comp = mav.target_component

    print(f"\n{'='*60}")
    print(f"MISSION UPLOAD TEST")
    print(f"  Waypoints: {len(waypoints)}")
    print(f"  Target: sys={target_sys}, comp={target_comp}")
    print(f"{'='*60}\n")

    # Print waypoints
    print("Waypoints to upload:")
    for i, wp in enumerate(waypoints):
        cmd_name = {22: "TAKEOFF", 16: "WAYPOINT", 21: "LAND", 20: "RTL"}.get(wp.cmd, f"CMD_{wp.cmd}")
        print(f"  WP{i}: {cmd_name:10} | Frame={wp.frame} | Lat={wp.lat:.7f}, Lon={wp.lon:.7f}, Alt={wp.alt:.1f}m | P1={wp.p1}")

    # Step 1: Send mission count
    print(f"\n[1] Sending MISSION_COUNT: {len(waypoints)} waypoints")
    mav.mav.mission_count_send(target_sys, target_comp, len(waypoints), mission_type=0)

    # Step 2: Wait for MISSION_REQUEST_INT or MISSION_ACK
    print(f"[2] Waiting for vehicle to request waypoints...\n")

    uploaded = 0
    timeout = time.time() + 30  # 30 second timeout

    while uploaded < len(waypoints) and time.time() < timeout:
        msg = mav.recv_match(blocking=True, timeout=1.0)

        if not msg:
            continue

        msg_type = msg.get_type()

        if msg_type == "MISSION_REQUEST_INT":
            seq = msg.seq
            print(f"[3] Vehicle requesting waypoint {seq+1}/{len(waypoints)}")

            if seq < len(waypoints):
                wp = waypoints[seq]
                pkt = to_mission_int(seq, wp, target_sys, target_comp)

                cmd_name = {22: "TAKEOFF", 16: "WAYPOINT", 21: "LAND"}.get(wp.cmd, f"CMD_{wp.cmd}")
                print(f"    Sending WP{seq}: {cmd_name} | Frame={wp.frame} | Lat={wp.lat:.7f}, Lon={wp.lon:.7f}, Alt={wp.alt:.1f}m")

                mav.mav.send(pkt)
                uploaded = seq + 1
            else:
                print(f"    ✗ ERROR: Requested waypoint {seq} out of range!")

        elif msg_type == "MISSION_REQUEST":
            # Handle legacy MISSION_REQUEST (non-INT version)
            seq = msg.seq
            print(f"[3] Vehicle requesting waypoint {seq+1}/{len(waypoints)} (legacy MISSION_REQUEST)")
            print(f"    ⚠ Vehicle is using MISSION_REQUEST instead of MISSION_REQUEST_INT")
            print(f"    This might be a compatibility issue!")

        elif msg_type == "MISSION_ACK":
            ack_type = msg.type
            print(f"\n[4] MISSION_ACK received:")
            print(f"    ACK Type: {ack_type}")
            print(f"    Meaning: {decode_ack_type(ack_type)}")

            if ack_type == 0:
                print(f"\n{'='*60}")
                print(f"✓✓✓ MISSION UPLOAD SUCCESSFUL ✓✓✓")
                print(f"{'='*60}\n")
                return True
            else:
                print(f"\n{'='*60}")
                print(f"✗✗✗ MISSION UPLOAD FAILED ✗✗✗")
                print(f"{'='*60}\n")

                # Provide debugging hints
                if ack_type == 2:
                    print("💡 HINT: Try changing frame type from 3 to 0 or 6")
                    print("   Frame 0 = MAV_FRAME_GLOBAL (absolute altitude)")
                    print("   Frame 3 = MAV_FRAME_GLOBAL_RELATIVE_ALT (relative)")
                    print("   Frame 6 = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT")
                elif ack_type == 3:
                    print("💡 HINT: One of the commands is not supported by this vehicle")
                    print("   Try using only WAYPOINT commands (CMD=16)")
                elif ack_type == 15:
                    print("💡 HINT: Mission operation was cancelled")
                    print("   This might happen if:")
                    print("   - Vehicle is armed")
                    print("   - Another mission operation is in progress")
                    print("   - Vehicle doesn't support mission protocol")

                return False

    print(f"\n✗ Timeout waiting for mission upload to complete")
    print(f"   Uploaded: {uploaded}/{len(waypoints)} waypoints")
    return False

def read_mission(mav):
    """Read mission from vehicle"""
    target_sys = mav.target_system
    target_comp = mav.target_component

    print(f"\n{'='*60}")
    print(f"MISSION DOWNLOAD TEST")
    print(f"  Target: sys={target_sys}, comp={target_comp}")
    print(f"{'='*60}\n")

    print("[1] Sending MISSION_REQUEST_LIST")
    mav.mav.mission_request_list_send(target_sys, target_comp, mission_type=0)

    print("[2] Waiting for MISSION_COUNT...\n")

    msg = mav.recv_match(type='MISSION_COUNT', blocking=True, timeout=5.0)

    if not msg:
        print("✗ No response from vehicle")
        return []

    count = msg.count
    print(f"[3] Vehicle reports {count} waypoint(s)\n")

    if count == 0:
        print("Mission is empty")
        return []

    waypoints = []
    for i in range(count):
        msg = mav.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5.0)
        if msg:
            lat = msg.x / 1e7
            lon = msg.y / 1e7
            alt = msg.z
            cmd = msg.command
            cmd_name = {22: "TAKEOFF", 16: "WAYPOINT", 21: "LAND"}.get(cmd, f"CMD_{cmd}")
            print(f"[4] Downloaded WP{i}: {cmd_name:10} | Lat={lat:.7f}, Lon={lon:.7f}, Alt={alt:.1f}m")
            waypoints.append(msg)

    print(f"\n✓ Mission download complete: {len(waypoints)} waypoints")
    return waypoints

def clear_mission(mav):
    """Clear mission from vehicle"""
    target_sys = mav.target_system
    target_comp = mav.target_component

    print(f"\n{'='*60}")
    print(f"MISSION CLEAR TEST")
    print(f"  Target: sys={target_sys}, comp={target_comp}")
    print(f"{'='*60}\n")

    print("[1] Sending MISSION_CLEAR_ALL")
    mav.mav.mission_clear_all_send(target_sys, target_comp, mission_type=0)

    print("[2] Waiting for acknowledgment...")
    time.sleep(0.5)

    print("✓ Clear command sent\n")

def main():
    """Main test function"""
    print("\n" + "="*60)
    print("   MAVLink Mission Upload Diagnostic Tool")
    print("="*60)

    # Connection string - modify as needed
    # First, let's detect available ports
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())

    if not ports:
        print("\n✗ No COM ports found!")
        print("Please connect your flight controller and try again.")
        return 1

    print("\nAvailable COM ports:")
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {port.device} - {port.description}")

    # Auto-select first port or let user choose
    if len(ports) == 1:
        CONNECTION_STRING = ports[0].device
        print(f"\nAuto-selected: {CONNECTION_STRING}")
    else:
        choice = input(f"\nSelect port (1-{len(ports)}) or press Enter for {ports[0].device}: ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            CONNECTION_STRING = ports[int(choice)-1].device
        else:
            CONNECTION_STRING = ports[0].device
        print(f"Using: {CONNECTION_STRING}")

    # Connect to vehicle
    mav = connect_to_vehicle(CONNECTION_STRING)
    if not mav:
        print("\n✗ Failed to connect to vehicle")
        return 1

    # Test menu
    while True:
        print("\n" + "="*60)
        print("SELECT TEST:")
        print("  1. Upload mission (5 waypoints)")
        print("  2. Read mission from vehicle")
        print("  3. Clear mission from vehicle")
        print("  4. Try different frame types")
        print("  5. Exit")
        print("="*60)

        choice = input("\nEnter choice (1-5): ").strip()

        if choice == "1":
            upload_mission(mav, DUMMY_WAYPOINTS)

        elif choice == "2":
            read_mission(mav)

        elif choice == "3":
            clear_mission(mav)

        elif choice == "4":
            # Test different frame types
            print("\nTesting different frame types...")
            for frame_id, frame_name in [(0, "GLOBAL"), (3, "GLOBAL_RELATIVE_ALT"), (6, "GLOBAL_RELATIVE_ALT_INT")]:
                print(f"\n--- Testing Frame {frame_id}: {frame_name} ---")
                test_wps = [WP(cmd=wp.cmd, lat=wp.lat, lon=wp.lon, alt=wp.alt, p1=wp.p1, frame=frame_id)
                           for wp in DUMMY_WAYPOINTS]
                if upload_mission(mav, test_wps):
                    print(f"✓ Frame {frame_id} works!")
                    break
                time.sleep(1)

        elif choice == "5":
            print("\nExiting...")
            break

        else:
            print("Invalid choice")

    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(1)
