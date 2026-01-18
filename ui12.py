#!/usr/bin/env python3
"""
Mission Upload Script - FIXED
Solved: AttributeError and Serial Buffer Timeouts
"""

import sys
import time
from pymavlink import mavutil
from dataclasses import dataclass

# --- DATA STRUCTURES ---

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

# Dummy waypoints for testing
DUMMY_WAYPOINTS = [
    WP(cmd=CMD_TAKEOFF, lat=18.5204, lon=73.8567, alt=30.0, p1=15.0),
    WP(cmd=CMD_WAYPOINT, lat=18.5214, lon=73.8577, alt=50.0),
    WP(cmd=CMD_WAYPOINT, lat=18.5224, lon=73.8587, alt=50.0),
    WP(cmd=CMD_WAYPOINT, lat=18.5234, lon=73.8597, alt=50.0),
    WP(cmd=CMD_LAND, lat=18.5244, lon=73.8607, alt=0.0),
]

def decode_ack_type(ack_type):
    ack_types = {
        0: "ACCEPTED", 1: "ERROR", 2: "UNSUPPORTED_FRAME", 3: "UNSUPPORTED",
        4: "NO_SPACE", 5: "INVALID", 13: "INVALID_SEQUENCE", 14: "DENIED",
        15: "CANCELLED"
    }
    return ack_types.get(ack_type, f"UNKNOWN({ack_type})")

# --- CONNECTION HANDLER ---

def connect_to_vehicle(connection_string):
    print(f"\n{'='*60}")
    print(f"Connecting to vehicle: {connection_string}")
    print(f"{'='*60}\n")

    try:
        # mavutil.mavlink_connection handles all serial parsing/buffering automatically
        # This fixes the "Timeout" and packet loss issues.
        master = mavutil.mavlink_connection(connection_string, baud=115200)
        
        print("Waiting for heartbeat...")
        # wait_heartbeat() is a built-in blocking function that is much safer
        master.wait_heartbeat(timeout=10)
        
        print(f"✓ Connected to SysID: {master.target_system} CompID: {master.target_component}")
        return master

    except Exception as e:
        print(f"Connection Error: {e}")
        return None

# --- MESSAGE CONVERTERS ---

def to_mission_int(seq, wp, target_sys, target_comp):
    """Convert to MISSION_ITEM_INT (Modern Protocol - Integer scaling)"""
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
        x=int(round(wp.lat * 1e7)), # Convert to integer scaling (deg * 10^7)
        y=int(round(wp.lon * 1e7)),
        z=float(wp.alt),
        mission_type=0
    )

def to_mission_item(seq, wp, target_sys, target_comp):
    """Convert to MISSION_ITEM (Legacy Protocol - Float)"""
    return mavutil.mavlink.MAVLink_mission_item_message(
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
        x=float(wp.lat),
        y=float(wp.lon),
        z=float(wp.alt)
    )

# --- UPLOAD LOGIC ---

def upload_mission(master, waypoints):
    """
    Uploads mission using the standard State Machine:
    1. Clear All
    2. Send Count
    3. Wait for Request -> Send Item
    4. Wait for Ack
    """
    target_sys = master.target_system
    target_comp = master.target_component

    print(f"\n[1] Starting Mission Upload ({len(waypoints)} items)")
    
    # 1. Clear previous mission
    master.mav.mission_clear_all_send(target_sys, target_comp, mission_type=0)
    
    # 2. Send Mission Count
    # We loop briefly to ensure the drone receives the count request
    ack_received = False
    req_received = False
    
    print("[2] Sending Count...")
    
    # Try sending count up to 5 times if no response
    for attempt in range(5):
        master.mav.mission_count_send(target_sys, target_comp, len(waypoints), mission_type=0)
        
        # Listen for a response (Request or Ack)
        msg = master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=1.5)
        
        if msg:
            if msg.get_type() == 'MISSION_ACK' and len(waypoints) == 0:
                print("✓ Empty mission accepted")
                return True
            if msg.get_type() in ['MISSION_REQUEST', 'MISSION_REQUEST_INT']:
                req_received = True
                print(f"✓ Sync established. Drone requested item: {msg.seq}")
                break # We are connected and ready to upload
    
    if not req_received:
        print("✗ ERROR: Drone did not respond to MISSION_COUNT. Check connection.")
        return False

    # 3. Upload Items Loop
    # The drone dictates the speed. We respond to what the drone asks for.
    
    start_time = time.time()
    while True:
        # Check overall timeout (prevent infinite loops)
        if time.time() - start_time > 60:
            print("✗ Timeout during upload process")
            return False
            
        # Keep GCS heartbeat alive during upload
        if time.time() % 1 < 0.1:
            master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

        # Wait for the next request
        msg = master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=2.0)
        
        if not msg:
            # If we timed out waiting for a request, re-send the last item or count?
            # Usually just continue and wait.
            print("   (Waiting for drone...)")
            continue
            
        msg_type = msg.get_type()
        
        # SUCCESS
        if msg_type == 'MISSION_ACK':
            print(f"\n[Finished] ACK Received: {decode_ack_type(msg.type)}")
            return msg.type == 0 # Return True if Accepted (0)
            
        # REQUEST HANDLING
        seq = msg.seq
        if seq >= len(waypoints):
            print(f"⚠ Drone requested invalid sequence: {seq}")
            continue
            
        wp = waypoints[seq]
        
        if msg_type == 'MISSION_REQUEST_INT':
            print(f"   -> Sending WP {seq} (INT)")
            # Create message
            pkt = to_mission_int(seq, wp, target_sys, target_comp)
            # Send using the mavutil connection
            master.mav.send(pkt)
            
        elif msg_type == 'MISSION_REQUEST':
            print(f"   -> Sending WP {seq} (FLOAT)")
            pkt = to_mission_item(seq, wp, target_sys, target_comp)
            master.mav.send(pkt)

# --- MAIN ---

def main():
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    
    print("\nMAVLink Mission Uploader (FIXED)")
    if not ports:
        print("No ports found.")
        return
        
    for i, p in enumerate(ports):
        print(f"{i+1}. {p.device} ({p.description})")
        
    sel = input("\nSelect Port (default 1): ")
    try:
        if not sel:
            port = ports[0].device
        else:
            port = ports[int(sel)-1].device
    except:
        print("Invalid selection")
        return
    
    # CONNECT
    master = connect_to_vehicle(port)
    if not master: return
    
    print("\nSelected Operation:")
    print("1. Upload Mission")
    print("2. Read Mission Count")
    print("3. Clear Mission")
    
    op = input("Choice: ")
    
    if op == "1":
        upload_mission(master, DUMMY_WAYPOINTS)
        
    elif op == "2":
        master.mav.mission_request_list_send(master.target_system, master.target_component, mission_type=0)
        msg = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
        if msg:
            print(f"Vehicle has {msg.count} waypoints")
        else:
            print("Read timeout")
            
    elif op == "3":
        master.mav.mission_clear_all_send(master.target_system, master.target_component, mission_type=0)
        ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
        if ack:
            print(f"Clear Result: {decode_ack_type(ack.type)}")
        else:
            print("Clear sent (no ACK)")

if __name__ == "__main__":
    main()