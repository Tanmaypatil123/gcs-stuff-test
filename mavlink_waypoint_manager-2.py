#!/usr/bin/env python3
"""
MAVLink Waypoint Manager with Auto Port Detection
==================================================
Automatically scans and connects to available COM ports,
then provides simple waypoint read/write functionality.

Requirements:
    pip install pymavlink pyserial

Usage:
    python mavlink_waypoint_manager.py
    
    # Or specify port manually:
    python mavlink_waypoint_manager.py --port COM5
"""

from pymavlink import mavutil, mavwp
import serial.tools.list_ports
import time


class MAVLinkWaypointManager:
    """
    Manages waypoint upload/download via MAVLink with auto port detection.
    """
    
    def __init__(self, port=None, baud=57600):
        """
        Initialize connection - auto-detect port if not specified.
        
        Args:
            port: COM port (e.g., 'COM5') or None for auto-detect
            baud: Baud rate (default 57600)
        """
        if port is None:
            port = self.auto_detect_port()
        
        if port is None:
            raise Exception("No COM port found! Please connect your device.")
        
        print(f"Connecting to {port} at {baud} baud...")
        self.mav = mavutil.mavlink_connection(port, baud=baud)
        
        print("Waiting for heartbeat...")
        self.mav.wait_heartbeat()
        print(f"HEARTBEAT OK - System: {self.mav.target_system}, Component: {self.mav.target_component}\n")
        
        # Waypoint loader helper
        self.wp = mavwp.MAVWPLoader()
    
    def auto_detect_port(self):
        """
        Automatically detect available COM ports.
        
        Returns:
            str: Port name (e.g., 'COM5') or None if not found
        """
        print("Scanning for available COM ports...")
        ports = serial.tools.list_ports.comports()
        
        if not ports:
            print("No COM ports found!")
            return None
        
        print("\nAvailable ports:")
        for i, port in enumerate(ports):
            print(f"  [{i}] {port.device} - {port.description}")
        
        # Common MAVLink device identifiers
        mavlink_keywords = ['px4', 'ardupilot', 'cube', 'pixhawk', 'mavlink', 
                           'usb', 'serial', 'uart', 'ftdi', 'silicon labs', 'ch340']
        
        # Try to find a MAVLink device
        for port in ports:
            desc_lower = port.description.lower()
            for keyword in mavlink_keywords:
                if keyword in desc_lower:
                    print(f"\nAuto-selected: {port.device} ({port.description})")
                    return port.device
        
        # If only one port, use it
        if len(ports) == 1:
            print(f"\nUsing only available port: {ports[0].device}")
            return ports[0].device
        
        # Let user choose
        print("\nMultiple ports found. Enter port number or name:")
        choice = input("> ").strip()
        
        try:
            idx = int(choice)
            return ports[idx].device
        except ValueError:
            # User entered port name directly
            return choice if choice else ports[0].device
    
    # =========================================================================
    # SET HOME POSITION
    # =========================================================================
    
    def set_home(self, lat, lon, alt=0):
        """Set home location."""
        print(f'Setting home: {lat}, {lon}, {alt}')
        
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1,  # set position
            0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            lat,  # latitude
            lon,  # longitude
            alt   # altitude
        )
        
        msg = self.mav.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=5)
        print(f'Set home ACK: {msg}')
        return msg
    
    def get_home(self):
        """Get home location."""
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        msg = self.mav.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=5)
        print(f'Get home ACK: {msg}')
        
        msg = self.mav.recv_match(type=['HOME_POSITION'], blocking=True, timeout=5)
        if msg:
            lat = msg.latitude / 1e7
            lon = msg.longitude / 1e7
            alt = msg.altitude / 1000.0
            print(f'Home position: {lat}, {lon}, {alt}')
            return (lat, lon, alt)
        return None
    
    # =========================================================================
    # WRITE WAYPOINTS TO DRONE
    # =========================================================================
    
    def write_waypoints(self, waypoints, altitude=15):
        """
        Upload waypoints to the drone.
        
        Args:
            waypoints: List of (lat, lon) tuples or (lat, lon, alt) tuples
            altitude: Default altitude if not specified in waypoint
            
        Returns:
            bool: True if successful
        """
        print("\n" + "="*50)
        print("WRITING WAYPOINTS TO DRONE")
        print("="*50)
        
        # Clear waypoint loader
        self.wp.clear()
        
        # Build waypoints
        for seq, waypoint in enumerate(waypoints):
            lat = waypoint[0]
            lon = waypoint[1]
            alt = waypoint[2] if len(waypoint) > 2 else altitude
            
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            autocontinue = 1
            current = 1 if seq == 0 else 0
            
            # First waypoint = TAKEOFF, Last = LAND, Others = WAYPOINT
            if seq == 0:
                cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                param1 = 15.0  # minimum pitch
            elif seq == len(waypoints) - 1:
                cmd = mavutil.mavlink.MAV_CMD_NAV_LAND
                param1 = 0
            else:
                cmd = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
                param1 = 0
            
            p = mavutil.mavlink.MAVLink_mission_item_message(
                self.mav.target_system,
                self.mav.target_component,
                seq,
                frame,
                cmd,
                current,
                autocontinue,
                param1, 0, 0, 0,  # params 1-4
                lat, lon, alt     # x, y, z
            )
            self.wp.add(p)
            print(f'Added WP {seq}: ({lat}, {lon}, {alt}) cmd={cmd}')
        
        # Clear existing mission
        print("\nClearing existing waypoints...")
        self.mav.waypoint_clear_all_send()
        time.sleep(0.5)
        
        # Send waypoint count
        print(f"Sending waypoint count: {self.wp.count()}")
        self.mav.waypoint_count_send(self.wp.count())
        
        # Send each waypoint when requested
        for i in range(self.wp.count()):
            msg = self.mav.recv_match(type=['MISSION_REQUEST'], blocking=True, timeout=10)
            if msg is None:
                print(f"ERROR: Timeout waiting for MISSION_REQUEST")
                return False
            
            print(f'Sending waypoint {msg.seq}')
            self.mav.mav.send(self.wp.wp(msg.seq))
        
        # Wait for ACK
        msg = self.mav.recv_match(type=['MISSION_ACK'], blocking=True, timeout=10)
        if msg:
            print(f'Mission ACK: {msg.type}')
            if msg.type == 0:  # MAV_MISSION_ACCEPTED
                print("WAYPOINTS UPLOADED SUCCESSFULLY!")
                return True
        
        print("UPLOAD FAILED!")
        return False
    
    # =========================================================================
    # READ WAYPOINTS FROM DRONE
    # =========================================================================
    
    def read_waypoints(self):
        """
        Download waypoints from the drone.
        
        Returns:
            list: List of waypoint dictionaries
        """
        print("\n" + "="*50)
        print("READING WAYPOINTS FROM DRONE")
        print("="*50)
        
        waypoints = []
        
        # Request waypoint list
        print("Requesting waypoint list...")
        self.mav.waypoint_request_list_send()
        
        # Get count
        msg = self.mav.recv_match(type=['MISSION_COUNT'], blocking=True, timeout=10)
        if msg is None:
            print("ERROR: No response from drone")
            return []
        
        waypoint_count = msg.count
        print(f'Waypoint count: {waypoint_count}')
        
        if waypoint_count == 0:
            print("No waypoints on drone")
            return []
        
        # Request each waypoint
        for i in range(waypoint_count):
            self.mav.waypoint_request_send(i)
            
            msg = self.mav.recv_match(type=['MISSION_ITEM'], blocking=True, timeout=10)
            if msg is None:
                print(f"ERROR: Timeout getting waypoint {i}")
                break
            
            wp_data = {
                'seq': msg.seq,
                'lat': msg.x,
                'lon': msg.y,
                'alt': msg.z,
                'command': msg.command,
                'frame': msg.frame,
                'param1': msg.param1,
                'param2': msg.param2,
                'param3': msg.param3,
                'param4': msg.param4,
            }
            waypoints.append(wp_data)
            
            print(f'Received WP {msg.seq}: lat={msg.x}, lon={msg.y}, alt={msg.z}, cmd={msg.command}')
        
        # Send ACK
        self.mav.mav.mission_ack_send(
            self.mav.target_system,
            self.mav.target_component,
            0  # MAV_MISSION_ACCEPTED
        )
        
        print(f"\nTotal waypoints read: {len(waypoints)}")
        return waypoints
    
    # =========================================================================
    # CLEAR WAYPOINTS
    # =========================================================================
    
    def clear_waypoints(self):
        """Clear all waypoints from drone."""
        print("\nClearing all waypoints...")
        self.mav.waypoint_clear_all_send()
        
        msg = self.mav.recv_match(type=['MISSION_ACK'], blocking=True, timeout=5)
        if msg:
            print(f'Clear ACK: {msg.type}')
            return msg.type == 0
        return False
    
    # =========================================================================
    # GET CURRENT POSITION
    # =========================================================================
    
    def get_current_position(self, timeout=5):
        """Get drone's current GPS position."""
        print("\nGetting current position...")
        
        msg = self.mav.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=timeout)
        
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            print(f'Current position: lat={lat}, lon={lon}, alt={alt}')
            return (lat, lon, alt)
        
        print("No position data received")
        return None
    
    def close(self):
        """Close connection."""
        self.mav.close()
        print("Connection closed")


# =============================================================================
# MAIN - DEMO
# =============================================================================

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='MAVLink Waypoint Manager')
    parser.add_argument('--port', default=None, help='COM port (e.g., COM5). Auto-detect if not specified.')
    parser.add_argument('--baud', type=int, default=57600, help='Baud rate')
    args = parser.parse_args()
    
    print("="*60)
    print("MAVLink Waypoint Manager")
    print("="*60)
    
    try:
        # Connect (auto-detect port if not specified)
        manager = MAVLinkWaypointManager(port=args.port, baud=args.baud)
        
        # Example waypoints (lat, lon) or (lat, lon, alt)
        waypoints = [
            (37.5090904347, 127.045094298),       # Takeoff point
            (37.509070898, 127.048905867, 20),    # WP 1
            (37.5063678607, 127.048960654, 25),   # WP 2
            (37.5061713129, 127.044741936, 20),   # WP 3
            (37.5078823794, 127.046914506)        # Land point
        ]
        
        home = waypoints[0]
        
        # Set home
        manager.set_home(home[0], home[1], 0)
        time.sleep(1)
        
        # Get home
        manager.get_home()
        time.sleep(1)
        
        # Write waypoints
        manager.write_waypoints(waypoints, altitude=15)
        time.sleep(1)
        
        # Read waypoints back
        read_wps = manager.read_waypoints()
        
        # Get current position
        manager.get_current_position()
        
        manager.close()
        
    except KeyboardInterrupt:
        print("\nCancelled by user")
    except Exception as e:
        print(f"\nError: {e}")
        raise


if __name__ == '__main__':
    main()
