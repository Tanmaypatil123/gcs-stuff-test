#!/usr/bin/env python3
"""
Standalone Mission Planner for ArduPilot/PX4
Features: Read, Write, Delete missions
Connection logic from ui6.py (robust auto-detection)
FIXED: Uses mavutil.mavlink_connection for proper MAVLink API access
"""

import sys
import time
import re
from typing import Optional, Dict, Any, List, Tuple

# Try importing required libraries
try:
    import serial
    import serial.tools.list_ports
except Exception:
    print("[ERROR] pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

try:
    from pymavlink import mavutil
except Exception:
    print("[ERROR] pymavlink not installed. Run: pip install pymavlink")
    sys.exit(1)

# ============================================================================
# CONNECTION CONFIGURATION (from ui6.py)
# ============================================================================

PREFERRED_PORTS = ["COM5", "COM4"]
BAUDS = [115200, 57600, 38400, 9600]

def win_path(port: str) -> str:
    """Convert COM port to Windows device path if needed."""
    m = re.match(r"^COM(\d+)$", port.upper())
    if m and int(m.group(1)) >= 10:
        return r"\\.\%s" % port.upper()
    return port

def list_candidate_ports() -> List[str]:
    """List serial ports, prioritizing preferred ports and Bluetooth SPP."""
    all_ports = list(serial.tools.list_ports.comports())
    names = [p.device for p in all_ports]
    out = []

    # Add preferred ports first
    for want in PREFERRED_PORTS:
        if want in names:
            out.append(want)

    # Add Bluetooth SPP ports
    for p in all_ports:
        if "standard serial over bluetooth" in (p.description or "").lower() and p.device not in out:
            out.append(p.device)

    # Add remaining ports
    for p in all_ports:
        if p.device not in out:
            out.append(p.device)

    return out

def enum_name(table: str, value: int) -> str:
    """Get MAVLink enum name from value."""
    try:
        return mavutil.mavlink.enums[table][int(value)].name
    except Exception:
        return f"UNKNOWN({value})"

def probe_port(port: str, baud: int, seconds: float = 2.0) -> Dict[str, Any]:
    """
    Probe a serial port for MAVLink traffic using mavutil.
    Returns dict with: ok, total_msgs, heartbeats, fc_candidates, err
    """
    result = dict(ok=False, total_msgs=0, heartbeats=0, fc_candidates=[], err=None)

    try:
        # Use mavutil for probing (temporary connection)
        connection_string = f"{win_path(port)}"
        master = mavutil.mavlink_connection(connection_string, baud=baud, source_system=255, source_component=190)
        master.setup_logfile(None, None)  # Disable logging
    except Exception as e:
        result["err"] = f"open_error: {e}"
        return result

    start = time.time()

    try:
        while time.time() - start < seconds:
            msg = master.recv_match(blocking=False, timeout=0.05)
            if not msg:
                continue

            result["total_msgs"] += 1

            if msg.get_type() == "HEARTBEAT":
                sid, cid = msg.get_srcSystem(), msg.get_srcComponent()
                if sid != 255 and msg.get_type() == "HEARTBEAT":
                    # Check if it's not a GCS
                    if hasattr(msg, 'type') and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                        result["heartbeats"] += 1
                        result["fc_candidates"].append((sid, cid, msg.type, msg.autopilot))
    except Exception as e:
        result["err"] = f"read_error: {e}"
    finally:
        try:
            master.close()
        except Exception:
            pass

    result["ok"] = result["total_msgs"] > 0
    return result

def auto_detect_connection() -> Optional[Tuple[str, int]]:
    """
    Auto-detect MAVLink connection by scanning ports.
    Returns (port, baud) tuple or None if not found.
    """
    ports = list_candidate_ports()

    if not ports:
        print("[ERROR] No serial ports found. Check connections.")
        return None

    print("\n[INFO] Scanning for MAVLink connections...")
    best_fc = best_any = None

    for port in ports:
        print(f"  Scanning {port}...")
        for baud in BAUDS:
            r = probe_port(port, baud, 2.0)

            if r["err"]:
                continue

            if r["total_msgs"] > 0:
                print(f"    {baud} baud: msgs={r['total_msgs']} heartbeats={r['heartbeats']} fc_candidates={len(r['fc_candidates'])}")

            fc_score = len(r["fc_candidates"]) * 100 + r["heartbeats"] * 2 + r["total_msgs"]
            any_score = r["total_msgs"]

            if r["fc_candidates"] and (best_fc is None or fc_score > best_fc[0]):
                best_fc = (fc_score, port, baud)

            if r["ok"] and (best_any is None or any_score > best_any[0]):
                best_any = (any_score, port, baud)

    if best_fc:
        return best_fc[1], best_fc[2]
    if best_any:
        return best_any[1], best_any[2]

    return None

def connect_to_autopilot(port: Optional[str] = None, baud: Optional[int] = None, auto: bool = True) -> Optional[Any]:
    """
    Connect to autopilot using mavutil.mavlink_connection (FIXED).
    Returns mavutil connection object with proper methods.
    """
    # Auto-detect if requested or no port/baud specified
    if auto or not (port and baud):
        result = auto_detect_connection()
        if not result:
            print("[ERROR] No MAVLink connection found.")
            return None
        port, baud = result

    print(f"\n[CONNECT] Opening {port} @ {baud} baud...")

    try:
        # FIXED: Use mavutil.mavlink_connection instead of raw MAVLink object
        connection_string = f"{win_path(port)}"
        master = mavutil.mavlink_connection(
            connection_string,
            baud=baud,
            source_system=255,
            source_component=190
        )

        print("[CONNECT] Waiting for autopilot heartbeat...")
        master.wait_heartbeat(timeout=10)

        print(f"[OK] Connected to autopilot:")
        print(f"     System ID: {master.target_system}, Component ID: {master.target_component}")

        # Get vehicle info from first heartbeat
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            print(f"     Type: {enum_name('MAV_TYPE', msg.type)}")
            print(f"     Autopilot: {enum_name('MAV_AUTOPILOT', msg.autopilot)}")
            print(f"     Mode: {mode}")
            print(f"     Armed: {bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}")

        return master

    except Exception as e:
        print(f"[ERROR] Connection failed: {e}")
        import traceback
        traceback.print_exc()
        return None

# ============================================================================
# MISSION PLANNING FUNCTIONS
# ============================================================================

class Waypoint:
    """Waypoint data structure."""
    def __init__(self, lat: float, lon: float, alt: float = 15.0, cmd: int = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT):
        self.lat = float(lat)
        self.lon = float(lon)
        self.alt = float(alt)
        self.cmd = cmd
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.param1 = 15.0 if cmd == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF else 0.0
        self.param2 = 0.0
        self.param3 = 0.0
        self.param4 = 0.0

def set_home_location(master, lat: float, lon: float, alt: float = 0.0):
    """Set home location."""
    print(f"\n[SET HOME] Setting home to: {lat:.7f}, {lon:.7f}, {alt:.1f}m")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        1,  # set position
        0, 0, 0, 0,  # params 1-4
        lat, lon, alt  # lat, lon, alt
    )

    msg = master.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=3.0)
    if msg:
        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("[OK] Home location set successfully")
            return True
        else:
            print(f"[WARNING] Home set result: {msg.result}")
    else:
        print("[WARNING] No ACK received for set home")

    return False

def get_home_location(master) -> Optional[Tuple[float, float, float]]:
    """Get current home location."""
    print("\n[GET HOME] Requesting home position...")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0, 0
    )

    msg = master.recv_match(type=['HOME_POSITION'], blocking=True, timeout=3.0)
    if msg:
        lat = msg.latitude / 1e7
        lon = msg.longitude / 1e7
        alt = msg.altitude / 1000.0
        print(f"[OK] Home position: {lat:.7f}, {lon:.7f}, {alt:.1f}m")
        return (lat, lon, alt)
    else:
        print("[ERROR] Failed to get home position")
        return None

def write_mission(master, waypoints: List[Waypoint]) -> bool:
    """
    Write/Upload mission to autopilot (FIXED).
    Returns True if successful.
    """
    if not waypoints:
        print("[ERROR] No waypoints to upload")
        return False

    print(f"\n[WRITE MISSION] Uploading {len(waypoints)} waypoints...")

    target_sys = master.target_system
    target_comp = master.target_component

    # 1. Clear existing mission
    master.mav.mission_clear_all_send(target_sys, target_comp, 0)
    time.sleep(0.5)

    # 2. Send waypoint count with retry logic (like ui12.py)
    print("[2] Sending mission count...")
    req_received = False

    for attempt in range(5):
        master.mav.mission_count_send(target_sys, target_comp, len(waypoints), 0)

        msg = master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=2.0)

        if msg:
            if msg.get_type() == 'MISSION_ACK' and len(waypoints) == 0:
                print("[OK] Empty mission accepted")
                return True
            if msg.get_type() in ['MISSION_REQUEST', 'MISSION_REQUEST_INT']:
                req_received = True
                print(f"[OK] Autopilot ready, requested item: {msg.seq}")
                break

    if not req_received:
        print("[ERROR] Autopilot did not respond to mission count")
        return False

    # 3. Upload items loop
    start_time = time.time()
    while True:
        # Timeout check
        if time.time() - start_time > 60:
            print("[ERROR] Timeout during upload")
            return False

        # Wait for request
        msg = master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=3.0)

        if not msg:
            print("  (Waiting for autopilot...)")
            continue

        msg_type = msg.get_type()

        # Check for completion
        if msg_type == 'MISSION_ACK':
            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print("[OK] Mission uploaded successfully!")
                return True
            else:
                print(f"[ERROR] Mission upload failed with ACK type: {msg.type}")
                return False

        # Handle waypoint request
        seq = msg.seq
        if seq >= len(waypoints):
            print(f"[WARNING] Autopilot requested invalid seq: {seq}")
            continue

        wp = waypoints[seq]
        current = 1 if seq == 0 else 0

        if msg_type == 'MISSION_REQUEST_INT':
            # Send as MISSION_ITEM_INT (modern protocol)
            master.mav.mission_item_int_send(
                target_sys, target_comp,
                seq,
                wp.frame,
                wp.cmd,
                current,
                1,  # autocontinue
                wp.param1, wp.param2, wp.param3, wp.param4,
                int(wp.lat * 1e7),  # latitude as int
                int(wp.lon * 1e7),  # longitude as int
                wp.alt,
                0  # mission_type
            )
            print(f"  Sent WP {seq} (INT): {wp.lat:.7f}, {wp.lon:.7f}, {wp.alt:.1f}m")
        else:
            # Send as MISSION_ITEM (legacy protocol)
            master.mav.mission_item_send(
                target_sys, target_comp,
                seq,
                wp.frame,
                wp.cmd,
                current,
                1,  # autocontinue
                wp.param1, wp.param2, wp.param3, wp.param4,
                wp.lat, wp.lon, wp.alt
            )
            print(f"  Sent WP {seq} (FLOAT): {wp.lat:.7f}, {wp.lon:.7f}, {wp.alt:.1f}m")

def read_mission(master) -> Optional[List[Dict[str, Any]]]:
    """
    Read/Download mission from autopilot (FIXED).
    Returns list of waypoint dicts or None.
    """
    print("\n[READ MISSION] Requesting mission from autopilot...")

    target_sys = master.target_system
    target_comp = master.target_component

    # Request mission list
    master.mav.mission_request_list_send(target_sys, target_comp, 0)

    # Get waypoint count
    msg = master.recv_match(type=['MISSION_COUNT'], blocking=True, timeout=5.0)

    if not msg:
        print("[ERROR] Timeout waiting for mission count")
        return None

    waypoint_count = msg.count
    print(f"[INFO] Autopilot has {waypoint_count} waypoints")

    if waypoint_count == 0:
        print("[INFO] No waypoints in autopilot")
        return []

    # Request each waypoint
    waypoints = []
    for i in range(waypoint_count):
        # Request waypoint (try INT first, fallback to regular)
        master.mav.mission_request_int_send(target_sys, target_comp, i, 0)

        msg = master.recv_match(type=['MISSION_ITEM', 'MISSION_ITEM_INT'], blocking=True, timeout=5.0)

        if not msg:
            print(f"[ERROR] Timeout waiting for waypoint {i}")
            return None

        # Handle both INT and regular format
        if msg.get_type() == 'MISSION_ITEM_INT':
            lat = msg.x / 1e7
            lon = msg.y / 1e7
        else:
            lat = msg.x
            lon = msg.y

        wp_data = {
            'seq': msg.seq,
            'frame': msg.frame,
            'command': msg.command,
            'current': msg.current,
            'autocontinue': msg.autocontinue,
            'param1': msg.param1,
            'param2': msg.param2,
            'param3': msg.param3,
            'param4': msg.param4,
            'lat': lat,
            'lon': lon,
            'alt': msg.z
        }

        waypoints.append(wp_data)

        # Get command name
        cmd_name = "UNKNOWN"
        if msg.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            cmd_name = "WAYPOINT"
        elif msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            cmd_name = "TAKEOFF"
        elif msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
            cmd_name = "LAND"
        elif msg.command == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            cmd_name = "RTL"

        print(f"  WP {msg.seq}: {cmd_name} | {lat:.7f}, {lon:.7f}, {msg.z:.1f}m")

    # Send ACK
    master.mav.mission_ack_send(target_sys, target_comp, mavutil.mavlink.MAV_MISSION_ACCEPTED, 0)

    print(f"[OK] Successfully read {len(waypoints)} waypoints")
    return waypoints

def delete_mission(master) -> bool:
    """
    Delete/Clear all waypoints from autopilot (FIXED).
    Returns True if successful.
    """
    print("\n[DELETE MISSION] Clearing all waypoints...")

    target_sys = master.target_system
    target_comp = master.target_component

    master.mav.mission_clear_all_send(target_sys, target_comp, 0)

    msg = master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=5.0)

    if msg:
        if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print("[OK] Mission cleared successfully!")
            return True
        else:
            print(f"[ERROR] Mission clear failed with type: {msg.type}")
            return False
    else:
        print("[WARNING] No ACK received, mission may be cleared")
        return True

def set_mode(master, mode_name: str) -> bool:
    """Set flight mode (e.g., 'GUIDED', 'AUTO', 'RTL') - FIXED."""
    print(f"\n[SET MODE] Changing mode to {mode_name}...")

    # Get mode ID from mavutil
    if mode_name.upper() not in master.mode_mapping():
        print(f"[ERROR] Unknown mode: {mode_name}")
        print(f"Available modes: {list(master.mode_mapping().keys())}")
        return False

    mode_id = master.mode_mapping()[mode_name.upper()]

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    msg = master.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=3.0)

    if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print(f"[OK] Mode changed to {mode_name}")
        return True
    else:
        print(f"[WARNING] Mode change may have failed")
        return False

def arm_disarm(master, arm: bool) -> bool:
    """Arm or disarm the vehicle - FIXED."""
    action = "ARM" if arm else "DISARM"
    print(f"\n[{action}] {'Arming' if arm else 'Disarming'} vehicle...")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1 if arm else 0,  # 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0
    )

    msg = master.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=3.0)

    if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print(f"[OK] Vehicle {'armed' if arm else 'disarmed'}")
        return True
    else:
        print(f"[WARNING] {action} may have failed")
        return False

# ============================================================================
# DEMO WAYPOINTS (Same as your example)
# ============================================================================

DEMO_WAYPOINTS = [
    (37.5090904347, 127.045094298),
    (37.509070898, 127.048905867),
    (37.5063678607, 127.048960654),
    (37.5061713129, 127.044741936),
    (37.5078823794, 127.046914506)
]

def create_demo_mission() -> List[Waypoint]:
    """Create demo mission from waypoint coordinates."""
    mission = []

    for i, (lat, lon) in enumerate(DEMO_WAYPOINTS):
        if i == 0:
            # First waypoint: TAKEOFF
            wp = Waypoint(lat, lon, alt=15.0, cmd=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
            wp.param1 = 15.0  # minimum pitch
        elif i == len(DEMO_WAYPOINTS) - 1:
            # Last waypoint: LAND
            wp = Waypoint(lat, lon, alt=0.0, cmd=mavutil.mavlink.MAV_CMD_NAV_LAND)
        else:
            # Regular waypoint
            wp = Waypoint(lat, lon, alt=15.0, cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)

        mission.append(wp)

    return mission

# ============================================================================
# MAIN MENU
# ============================================================================

def print_menu():
    """Print main menu."""
    print("\n" + "="*60)
    print("  MISSION PLANNER - Main Menu")
    print("="*60)
    print("  1. Write Mission (Upload demo waypoints)")
    print("  2. Read Mission (Download from autopilot)")
    print("  3. Delete Mission (Clear all waypoints)")
    print("  4. Set Home Location")
    print("  5. Get Home Location")
    print("  6. Set Flight Mode")
    print("  7. Arm/Disarm")
    print("  8. Reconnect")
    print("  9. Exit")
    print("="*60)

def main():
    """Main program loop."""
    print("\n" + "="*60)
    print("  ArduPilot/PX4 Mission Planner")
    print("  Connection logic from ui6.py (FIXED)")
    print("="*60)

    # Connect to autopilot
    master = connect_to_autopilot(auto=True)

    if not master:
        print("\n[FATAL] Could not connect to autopilot.")
        print("Please check:")
        print("  - Flight controller is powered")
        print("  - USB/Bluetooth connection is active")
        print("  - Correct COM port in Device Manager")
        sys.exit(1)

    # Main loop
    while True:
        try:
            print_menu()
            choice = input("\nEnter choice (1-9): ").strip()

            if choice == "1":
                # Write Mission
                mission = create_demo_mission()
                print(f"\nDemo mission has {len(mission)} waypoints:")
                for i, wp in enumerate(mission):
                    cmd_name = "TAKEOFF" if wp.cmd == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF else \
                               "LAND" if wp.cmd == mavutil.mavlink.MAV_CMD_NAV_LAND else "WAYPOINT"
                    print(f"  {i}: {cmd_name} | {wp.lat:.7f}, {wp.lon:.7f}, {wp.alt:.1f}m")

                confirm = input("\nUpload this mission? (y/n): ").strip().lower()
                if confirm == 'y':
                    write_mission(master, mission)

            elif choice == "2":
                # Read Mission
                waypoints = read_mission(master)
                if waypoints:
                    print(f"\n[SUMMARY] Read {len(waypoints)} waypoints from autopilot")

            elif choice == "3":
                # Delete Mission
                confirm = input("\nAre you sure you want to delete all waypoints? (y/n): ").strip().lower()
                if confirm == 'y':
                    delete_mission(master)

            elif choice == "4":
                # Set Home
                if DEMO_WAYPOINTS:
                    lat, lon = DEMO_WAYPOINTS[0]
                    print(f"\nUsing first demo waypoint as home: {lat:.7f}, {lon:.7f}")
                    set_home_location(master, lat, lon, 0.0)
                else:
                    print("[ERROR] No demo waypoints available")

            elif choice == "5":
                # Get Home
                get_home_location(master)

            elif choice == "6":
                # Set Mode
                mode = input("\nEnter mode (e.g., GUIDED, AUTO, RTL, LOITER): ").strip().upper()
                set_mode(master, mode)

            elif choice == "7":
                # Arm/Disarm
                action = input("\nArm or Disarm? (arm/disarm): ").strip().lower()
                if action == "arm":
                    arm_disarm(master, True)
                elif action == "disarm":
                    arm_disarm(master, False)
                else:
                    print("[ERROR] Invalid choice. Enter 'arm' or 'disarm'")

            elif choice == "8":
                # Reconnect
                print("\n[RECONNECT] Closing current connection...")
                try:
                    master.close()
                except Exception:
                    pass

                master = connect_to_autopilot(auto=True)
                if not master:
                    print("[FATAL] Reconnection failed.")
                    break

            elif choice == "9":
                # Exit
                print("\n[EXIT] Closing connection...")
                try:
                    master.close()
                except Exception:
                    pass
                print("Goodbye!")
                break

            else:
                print("[ERROR] Invalid choice. Please enter 1-9.")

        except KeyboardInterrupt:
            print("\n\n[EXIT] Interrupted by user.")
            try:
                master.close()
            except Exception:
                pass
            break
        except Exception as e:
            print(f"\n[ERROR] Unexpected error: {e}")
            import traceback
            traceback.print_exc()

if __name__ == "__main__":
    main()
