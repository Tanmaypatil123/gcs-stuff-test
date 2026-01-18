#!/usr/bin/env python3
"""
Mission Upload Test Script for Pixhawk Cube Orange
Tests waypoint upload to flight controller via Bluetooth serial with robust handling
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

# System status codes
SYSTEM_STATUS = {
    0: "UNINIT - Uninitialized",
    1: "BOOT - Booting up",
    2: "CALIBRATING - Calibrating sensors",
    3: "STANDBY - Ready, motors off",
    4: "ACTIVE - Motors engaged",
    5: "CRITICAL - Critical state",
    6: "EMERGENCY - Emergency",
    7: "POWEROFF - Powering off",
    8: "FLIGHT_TERMINATION - Flight termination"
}

# Autopilot types
AUTOPILOT_TYPE = {
    0: "Generic",
    3: "ArduPilot",
    4: "OpenPilot",
    8: "Invalid",
    12: "PX4",
}

# Vehicle types
VEHICLE_TYPE = {
    0: "Generic",
    1: "Fixed Wing",
    2: "Quadrotor",
    3: "Coaxial",
    4: "Helicopter",
    6: "GCS",
    10: "Ground Rover",
    11: "Surface Boat",
    12: "Submarine",
    13: "Hexarotor",
    14: "Octorotor",
    15: "Tricopter",
}

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


class MAVConnection:
    """Robust MAVLink connection handler for Bluetooth serial"""

    def __init__(self, serial_conn, mavlink_obj, sys_id, comp_id):
        self.serial = serial_conn
        self.mav = mavlink_obj
        self.target_system = sys_id
        self.target_component = comp_id
        self._message_buffer = []
        self._last_heartbeat = None
        self._connected = True

    def _read_messages(self, timeout=0.1):
        """Read all available messages from serial buffer"""
        messages = []
        start_time = time.time()

        while (time.time() - start_time) < timeout:
            try:
                data = self.serial.read(1024)
                if not data:
                    time.sleep(0.005)  # Small delay for Bluetooth
                    continue

                for byte in data:
                    msg = self.mav.parse_char(bytes([byte]))
                    if msg:
                        messages.append(msg)
                        # Track heartbeats
                        if msg.get_type() == "HEARTBEAT":
                            self._last_heartbeat = time.time()
            except Exception as e:
                print(f"Read error: {e}")
                break

        return messages

    def recv_match(self, type=None, blocking=False, timeout=None):
        """Receive and parse messages with robust handling"""
        if timeout is None:
            timeout = 5.0 if blocking else 0.1

        start_time = time.time()
        type_list = [type] if isinstance(type, str) else (type or [])

        while True:
            elapsed = time.time() - start_time
            if elapsed > timeout:
                return None

            # Check buffered messages first
            for i, msg in enumerate(self._message_buffer):
                msg_type = msg.get_type()
                if not type_list or msg_type in type_list:
                    self._message_buffer.pop(i)
                    return msg

            # Read new messages
            remaining_timeout = max(0.1, timeout - elapsed)
            messages = self._read_messages(min(0.2, remaining_timeout))

            for msg in messages:
                msg_type = msg.get_type()
                if not type_list or msg_type in type_list:
                    return msg
                else:
                    # Buffer other messages for later
                    self._message_buffer.append(msg)

            if not blocking:
                return None

            time.sleep(0.01)

    def recv_any(self, timeout=0.5):
        """Receive any message (for draining buffer)"""
        messages = []
        start_time = time.time()

        while (time.time() - start_time) < timeout:
            msg = self.recv_match(blocking=False, timeout=0.1)
            if msg:
                messages.append(msg)
            else:
                break

        return messages

    def is_connected(self):
        """Check if connection is still alive"""
        if self._last_heartbeat:
            return (time.time() - self._last_heartbeat) < 5.0
        return self._connected

    def close(self):
        """Close the connection"""
        self._connected = False
        try:
            self.serial.close()
        except:
            pass


def connect_to_vehicle(connection_string, timeout=15):
    """
    Connect to vehicle via Bluetooth serial and return MAVLink connection
    Returns connection object with verified Pixhawk status
    """
    print(f"\n{'='*60}")
    print(f"CONNECTING TO PIXHAWK VIA BLUETOOTH")
    print(f"Port: {connection_string}")
    print(f"{'='*60}\n")

    try:
        import serial
        import re

        # Windows COM port handling
        def win_path(port: str) -> str:
            m = re.match(r"^COM(\d+)$", port.upper())
            if m and int(m.group(1)) >= 10:
                return r"\\.\%s" % port.upper()
            return port

        port = win_path(connection_string)
        print(f"[1/4] Opening serial port: {port}")

        # Bluetooth serial settings - optimized for reliability
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=0.1,           # Read timeout
            write_timeout=1.0,     # Write timeout
            rtscts=False,
            dsrdtr=False,
            xonxoff=False
        )

        # Clear any stale data
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("     Serial port opened successfully")

        # Create MAVLink parser
        print(f"[2/4] Initializing MAVLink protocol...")
        from pymavlink.dialects.v20 import ardupilotmega as mavlink2
        mav = mavlink2.MAVLink(ser)
        mav.robust_parsing = True
        mav.srcSystem = 255
        mav.srcComponent = 190
        print("     MAVLink initialized (System ID: 255, Component: 190)")

        # Wait for heartbeat
        print(f"[3/4] Waiting for heartbeat from Pixhawk...")
        print(f"     (Timeout: {timeout} seconds)")

        start = time.time()
        fc_sid = None
        fc_cid = None
        heartbeat_msg = None

        while (time.time() - start < timeout):
            try:
                data = ser.read(512)
                if not data:
                    time.sleep(0.02)
                    continue

                for byte in data:
                    msg = mav.parse_char(bytes([byte]))
                    if not msg:
                        continue

                    if msg.get_type() == "HEARTBEAT":
                        sid = msg.get_srcSystem()
                        cid = msg.get_srcComponent()

                        # Ignore GCS heartbeats
                        if sid != 255:
                            fc_sid = sid
                            fc_cid = cid
                            heartbeat_msg = msg
                            break

                if fc_sid is not None:
                    break

            except Exception as e:
                print(f"     Warning: Read error - {e}")
                time.sleep(0.1)

        if fc_sid is None:
            print("\n[FAILED] No heartbeat received from Pixhawk")
            print("         Possible causes:")
            print("         - Bluetooth not paired/connected")
            print("         - Wrong serial port selected")
            print("         - Pixhawk not powered on")
            print("         - Baud rate mismatch")
            ser.close()
            return None

        # Parse and display connection status
        print(f"\n[4/4] CONNECTION ESTABLISHED!")
        print(f"\n     ┌─────────────────────────────────────────────┐")
        print(f"     │           PIXHAWK STATUS                    │")
        print(f"     ├─────────────────────────────────────────────┤")
        print(f"     │  System ID:     {fc_sid:<27} │")
        print(f"     │  Component ID:  {fc_cid:<27} │")

        vehicle_type = VEHICLE_TYPE.get(heartbeat_msg.type, f"Unknown ({heartbeat_msg.type})")
        print(f"     │  Vehicle Type:  {vehicle_type:<27} │")

        autopilot = AUTOPILOT_TYPE.get(heartbeat_msg.autopilot, f"Unknown ({heartbeat_msg.autopilot})")
        print(f"     │  Autopilot:     {autopilot:<27} │")

        sys_status = SYSTEM_STATUS.get(heartbeat_msg.system_status, f"Unknown ({heartbeat_msg.system_status})")
        print(f"     │  System Status: {sys_status:<27} │")

        # Decode base mode
        base_mode = heartbeat_msg.base_mode
        armed = "ARMED" if (base_mode & 128) else "DISARMED"
        print(f"     │  Armed State:   {armed:<27} │")
        print(f"     └─────────────────────────────────────────────┘")

        # Create connection wrapper
        conn = MAVConnection(ser, mav, fc_sid, fc_cid)

        # Verify connection with additional heartbeats
        print(f"\n     Verifying connection stability...")
        hb_count = 0
        verify_start = time.time()
        while (time.time() - verify_start < 3.0) and hb_count < 2:
            msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.5)
            if msg and msg.get_srcSystem() == fc_sid:
                hb_count += 1
                print(f"     Heartbeat {hb_count}/2 received")

        if hb_count >= 2:
            print(f"\n     CONNECTION VERIFIED - Ready for mission upload!")
        else:
            print(f"\n     Warning: Connection may be unstable")

        return conn

    except serial.SerialException as e:
        print(f"\n[FAILED] Serial port error: {e}")
        print("         Check if the port is in use by another application")
        return None
    except Exception as e:
        print(f"\n[FAILED] Connection failed: {e}")
        import traceback
        traceback.print_exc()
        return None


def upload_mission(mav, waypoints, max_retries=3):
    """
    Upload mission to Pixhawk with robust retry handling
    Returns True if upload successful with confirmation
    """
    target_sys = mav.target_system
    target_comp = mav.target_component

    print(f"\n{'='*60}")
    print(f"MISSION UPLOAD TO PIXHAWK")
    print(f"{'='*60}")
    print(f"  Waypoints:  {len(waypoints)}")
    print(f"  Target:     System={target_sys}, Component={target_comp}")
    print(f"  Retries:    {max_retries}")
    print(f"{'='*60}\n")

    # Display waypoints to upload
    print("WAYPOINTS TO UPLOAD:")
    print("─" * 70)
    for i, wp in enumerate(waypoints):
        cmd_name = {22: "TAKEOFF", 16: "WAYPOINT", 21: "LAND", 20: "RTL"}.get(wp.cmd, f"CMD_{wp.cmd}")
        print(f"  WP{i}: {cmd_name:10} | Frame={wp.frame} | "
              f"Lat={wp.lat:.7f}, Lon={wp.lon:.7f}, Alt={wp.alt:.1f}m")
    print("─" * 70)

    # Clear any pending messages
    print("\n[STEP 1/5] Clearing message buffer...")
    mav.recv_any(timeout=0.3)
    print("           Buffer cleared")

    # Send mission count with retry
    for attempt in range(max_retries):
        print(f"\n[STEP 2/5] Sending MISSION_COUNT ({len(waypoints)} waypoints)...")
        if attempt > 0:
            print(f"           Retry attempt {attempt + 1}/{max_retries}")

        mav.mav.mission_count_send(target_sys, target_comp, len(waypoints), mission_type=0)
        time.sleep(0.1)  # Give Pixhawk time to process

        # Wait for first request
        print(f"\n[STEP 3/5] Waiting for Pixhawk to request waypoints...")

        request_received = False
        uploaded = set()
        last_request_time = time.time()
        upload_timeout = 45  # Total upload timeout
        request_timeout = 8  # Timeout waiting for each request

        start_time = time.time()

        while time.time() - start_time < upload_timeout:
            # Check for timeout waiting for requests
            if time.time() - last_request_time > request_timeout:
                if not request_received:
                    print(f"           No request received, retrying...")
                    break
                else:
                    # Resend last waypoint
                    print(f"           Request timeout, checking status...")

            msg = mav.recv_match(blocking=True, timeout=1.0)

            if not msg:
                continue

            msg_type = msg.get_type()

            if msg_type == "MISSION_REQUEST_INT":
                request_received = True
                last_request_time = time.time()
                seq = msg.seq

                print(f"\n[STEP 4/5] Pixhawk requesting waypoint {seq + 1}/{len(waypoints)}")

                if seq < len(waypoints):
                    wp = waypoints[seq]
                    pkt = to_mission_int(seq, wp, target_sys, target_comp)

                    cmd_name = {22: "TAKEOFF", 16: "WAYPOINT", 21: "LAND"}.get(wp.cmd, f"CMD_{wp.cmd}")
                    print(f"           Sending WP{seq}: {cmd_name} | "
                          f"Lat={wp.lat:.7f}, Lon={wp.lon:.7f}, Alt={wp.alt:.1f}m")

                    mav.mav.send(pkt)
                    uploaded.add(seq)
                    time.sleep(0.05)  # Small delay for Bluetooth
                else:
                    print(f"           ERROR: Requested waypoint {seq} out of range!")

            elif msg_type == "MISSION_REQUEST":
                # Legacy request handling
                request_received = True
                last_request_time = time.time()
                seq = msg.seq

                print(f"\n[STEP 4/5] Pixhawk requesting waypoint {seq + 1}/{len(waypoints)} (legacy)")

                if seq < len(waypoints):
                    wp = waypoints[seq]
                    # Send as legacy MISSION_ITEM
                    lat_int = int(round(wp.lat * 1e7))
                    lon_int = int(round(wp.lon * 1e7))

                    mav.mav.mission_item_send(
                        target_sys, target_comp, seq, wp.frame, wp.cmd,
                        1 if seq == 0 else 0, 1,
                        float(wp.p1), float(wp.p2), float(wp.p3), float(wp.p4),
                        wp.lat, wp.lon, float(wp.alt), 0
                    )
                    uploaded.add(seq)
                    print(f"           Sent WP{seq} (legacy format)")
                    time.sleep(0.05)

            elif msg_type == "MISSION_ACK":
                ack_type = msg.type

                print(f"\n[STEP 5/5] MISSION_ACK RECEIVED")
                print(f"           ACK Code: {ack_type}")
                print(f"           Status:   {decode_ack_type(ack_type)}")

                if ack_type == 0:
                    # SUCCESS!
                    print(f"\n{'='*60}")
                    print(f"  MISSION UPLOAD SUCCESSFUL!")
                    print(f"  {len(waypoints)} waypoints uploaded to Pixhawk")
                    print(f"{'='*60}")

                    # Verify by requesting mission count
                    print(f"\n  Verifying upload...")
                    time.sleep(0.2)
                    mav.mav.mission_request_list_send(target_sys, target_comp, mission_type=0)

                    verify_msg = mav.recv_match(type="MISSION_COUNT", blocking=True, timeout=3.0)
                    if verify_msg:
                        print(f"  Pixhawk reports {verify_msg.count} waypoints stored")
                        if verify_msg.count == len(waypoints):
                            print(f"  VERIFICATION PASSED!")
                        else:
                            print(f"  Warning: Count mismatch (expected {len(waypoints)})")
                    else:
                        print(f"  Could not verify (no response)")

                    return True
                else:
                    # Upload failed
                    print(f"\n{'='*60}")
                    print(f"  MISSION UPLOAD FAILED")
                    print(f"{'='*60}")

                    # Provide debugging hints
                    if ack_type == 2:
                        print("\n  HINT: Frame type not supported")
                        print("        Try frame=0 (GLOBAL) or frame=6 (GLOBAL_RELATIVE_ALT_INT)")
                    elif ack_type == 3:
                        print("\n  HINT: Command not supported by this vehicle")
                    elif ack_type == 14:
                        print("\n  HINT: Vehicle not accepting missions")
                        print("        Check if vehicle is armed or in a mode that blocks uploads")
                    elif ack_type == 15:
                        print("\n  HINT: Operation cancelled")
                        print("        Another operation may be in progress")

                    return False

            elif msg_type == "HEARTBEAT":
                # Connection still alive
                pass

        if not request_received:
            print(f"           No response from Pixhawk, will retry...")
            time.sleep(1.0)
            continue

    print(f"\n[FAILED] Mission upload timed out after {max_retries} attempts")
    print(f"         Waypoints acknowledged: {len(uploaded)}/{len(waypoints)}")
    return False


def read_mission(mav):
    """Read mission from Pixhawk with status feedback"""
    target_sys = mav.target_system
    target_comp = mav.target_component

    print(f"\n{'='*60}")
    print(f"READING MISSION FROM PIXHAWK")
    print(f"{'='*60}\n")

    print("[1/3] Requesting mission list...")
    mav.mav.mission_request_list_send(target_sys, target_comp, mission_type=0)

    msg = mav.recv_match(type='MISSION_COUNT', blocking=True, timeout=5.0)

    if not msg:
        print("[FAILED] No response from Pixhawk")
        return []

    count = msg.count
    print(f"[2/3] Pixhawk reports {count} waypoint(s)\n")

    if count == 0:
        print("       Mission is empty")
        return []

    waypoints = []
    print("[3/3] Downloading waypoints...")
    print("─" * 70)

    for i in range(count):
        # Request each waypoint
        mav.mav.mission_request_int_send(target_sys, target_comp, i, mission_type=0)

        msg = mav.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=3.0)
        if msg:
            lat = msg.x / 1e7
            lon = msg.y / 1e7
            alt = msg.z
            cmd = msg.command
            cmd_name = {22: "TAKEOFF", 16: "WAYPOINT", 21: "LAND", 20: "RTL"}.get(cmd, f"CMD_{cmd}")
            print(f"  WP{i}: {cmd_name:10} | Lat={lat:.7f}, Lon={lon:.7f}, Alt={alt:.1f}m")
            waypoints.append(msg)
        else:
            print(f"  WP{i}: [TIMEOUT - No response]")

    print("─" * 70)
    print(f"\nMission download complete: {len(waypoints)}/{count} waypoints retrieved")

    # Send ACK
    mav.mav.mission_ack_send(target_sys, target_comp, 0, mission_type=0)

    return waypoints


def clear_mission(mav):
    """Clear mission from Pixhawk with confirmation"""
    target_sys = mav.target_system
    target_comp = mav.target_component

    print(f"\n{'='*60}")
    print(f"CLEARING MISSION FROM PIXHAWK")
    print(f"{'='*60}\n")

    print("[1/2] Sending MISSION_CLEAR_ALL...")
    mav.mav.mission_clear_all_send(target_sys, target_comp, mission_type=0)

    print("[2/2] Waiting for acknowledgment...")
    msg = mav.recv_match(type='MISSION_ACK', blocking=True, timeout=3.0)

    if msg:
        if msg.type == 0:
            print("\n       MISSION CLEARED SUCCESSFULLY")
        else:
            print(f"\n       Clear failed: {decode_ack_type(msg.type)}")
    else:
        print("\n       No acknowledgment received (mission may still be cleared)")

    # Verify
    time.sleep(0.3)
    mav.mav.mission_request_list_send(target_sys, target_comp, mission_type=0)
    verify = mav.recv_match(type='MISSION_COUNT', blocking=True, timeout=2.0)
    if verify:
        print(f"       Verification: Pixhawk now has {verify.count} waypoints")


def get_pixhawk_status(mav):
    """Get detailed status from Pixhawk"""
    print(f"\n{'='*60}")
    print(f"PIXHAWK STATUS")
    print(f"{'='*60}\n")

    # Wait for heartbeat
    print("Waiting for status messages...")

    start = time.time()
    heartbeat = None
    sys_status = None
    gps_raw = None

    while (time.time() - start) < 5.0:
        msg = mav.recv_match(blocking=True, timeout=1.0)
        if not msg:
            continue

        msg_type = msg.get_type()

        if msg_type == "HEARTBEAT" and msg.get_srcSystem() == mav.target_system:
            heartbeat = msg
        elif msg_type == "SYS_STATUS":
            sys_status = msg
        elif msg_type == "GPS_RAW_INT":
            gps_raw = msg

        if heartbeat and sys_status:
            break

    if heartbeat:
        print(f"┌─────────────────────────────────────────────┐")
        print(f"│           HEARTBEAT STATUS                  │")
        print(f"├─────────────────────────────────────────────┤")

        vehicle = VEHICLE_TYPE.get(heartbeat.type, f"Unknown ({heartbeat.type})")
        print(f"│  Vehicle:     {vehicle:<29} │")

        autopilot = AUTOPILOT_TYPE.get(heartbeat.autopilot, f"Unknown ({heartbeat.autopilot})")
        print(f"│  Autopilot:   {autopilot:<29} │")

        status = SYSTEM_STATUS.get(heartbeat.system_status, f"Unknown")
        print(f"│  Status:      {status:<29} │")

        armed = "ARMED" if (heartbeat.base_mode & 128) else "DISARMED"
        print(f"│  Armed:       {armed:<29} │")

        print(f"└─────────────────────────────────────────────┘")

    if sys_status:
        print(f"\n┌─────────────────────────────────────────────┐")
        print(f"│           SYSTEM STATUS                     │")
        print(f"├─────────────────────────────────────────────┤")
        voltage = sys_status.voltage_battery / 1000.0
        print(f"│  Battery:     {voltage:.2f}V                          │")
        print(f"│  CPU Load:    {sys_status.load / 10.0:.1f}%                          │")
        print(f"└─────────────────────────────────────────────┘")

    if not heartbeat:
        print("No status received from Pixhawk")


def main():
    """Main test function"""
    print("\n" + "="*60)
    print("   PIXHAWK CUBE ORANGE - MISSION UPLOAD TOOL")
    print("   Bluetooth Serial Connection")
    print("="*60)

    # Detect available ports
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())

    if not ports:
        print("\n[ERROR] No serial ports found!")
        print("        Please check:")
        print("        - Bluetooth is paired and connected")
        print("        - Pixhawk telemetry is powered on")
        return 1

    print("\nAvailable serial ports:")
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {port.device} - {port.description}")

    # Port selection
    if len(ports) == 1:
        CONNECTION_STRING = ports[0].device
        print(f"\nAuto-selected: {CONNECTION_STRING}")
    else:
        choice = input(f"\nSelect port (1-{len(ports)}) or Enter for {ports[0].device}: ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            CONNECTION_STRING = ports[int(choice)-1].device
        else:
            CONNECTION_STRING = ports[0].device
        print(f"Using: {CONNECTION_STRING}")

    # Connect to Pixhawk
    mav = connect_to_vehicle(CONNECTION_STRING, timeout=15)
    if not mav:
        print("\n[ERROR] Failed to connect to Pixhawk")
        return 1

    # Main menu
    while True:
        print("\n" + "="*60)
        print("MENU:")
        print("  1. Upload mission (5 waypoints)")
        print("  2. Read mission from Pixhawk")
        print("  3. Clear mission")
        print("  4. Get Pixhawk status")
        print("  5. Test different frame types")
        print("  6. Exit")
        print("="*60)

        choice = input("\nEnter choice (1-6): ").strip()

        if choice == "1":
            result = upload_mission(mav, DUMMY_WAYPOINTS)
            if result:
                print("\nMission ready for flight!")
            else:
                print("\nPlease check connection and try again")

        elif choice == "2":
            read_mission(mav)

        elif choice == "3":
            clear_mission(mav)

        elif choice == "4":
            get_pixhawk_status(mav)

        elif choice == "5":
            print("\nTesting different frame types...")
            for frame_id, frame_name in [(3, "GLOBAL_RELATIVE_ALT"), (0, "GLOBAL"), (6, "GLOBAL_RELATIVE_ALT_INT")]:
                print(f"\n--- Testing Frame {frame_id}: {frame_name} ---")
                test_wps = [WP(cmd=wp.cmd, lat=wp.lat, lon=wp.lon, alt=wp.alt, p1=wp.p1, frame=frame_id)
                           for wp in DUMMY_WAYPOINTS]
                if upload_mission(mav, test_wps, max_retries=1):
                    print(f"\nFrame {frame_id} ({frame_name}) works!")
                    break
                time.sleep(1)

        elif choice == "6":
            print("\nClosing connection...")
            mav.close()
            print("Goodbye!")
            break

        else:
            print("Invalid choice, please try again")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(1)
