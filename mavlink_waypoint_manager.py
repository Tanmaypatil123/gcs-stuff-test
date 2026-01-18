#!/usr/bin/env python3
"""
MAVLink Waypoint/Mission Manager
================================
A script demonstrating how to upload and download waypoints/coordinates 
to/from a drone using the MAVLink protocol.

This is similar to how Mission Planner (and other GCS software) handles 
mission/waypoint operations.

The MAVLink Mission Protocol:
- Upload: GCS sends MISSION_COUNT -> Drone requests items one by one -> GCS sends items -> Drone ACKs
- Download: GCS requests list -> Drone sends count -> GCS requests items one by one -> Drone sends items -> GCS ACKs

Requirements:
    pip install pymavlink

Usage:
    python mavlink_waypoint_manager.py --connect <connection_string>
    
    Examples:
        python mavlink_waypoint_manager.py --connect /dev/ttyUSB0 --baud 57600
        python mavlink_waypoint_manager.py --connect udp:127.0.0.1:14550
        python mavlink_waypoint_manager.py --connect tcp:127.0.0.1:5760

Author: Based on ArduPilot/Mission Planner MAVLink implementation
License: GPL-3.0 (same as Mission Planner)
"""

from pymavlink import mavutil, mavwp
import time
import argparse


class MAVLinkWaypointManager:
    """
    Manages waypoint/mission upload and download operations via MAVLink.
    
    This class implements the MAVLink Mission Protocol as used by 
    Mission Planner and other GCS software.
    """
    
    def __init__(self, connection_string, baud=57600):
        """
        Initialize connection to the drone.
        
        Args:
            connection_string: MAVLink connection string (serial port, UDP, TCP)
            baud: Baud rate for serial connections
        """
        print(f"Connecting to: {connection_string}")
        
        # Establish MAVLink connection
        if connection_string.startswith('udp:') or connection_string.startswith('tcp:'):
            self.mav = mavutil.mavlink_connection(connection_string)
        else:
            self.mav = mavutil.mavlink_connection(connection_string, baud=baud)
        
        # Wait for heartbeat to confirm connection
        print("Waiting for heartbeat...")
        self.mav.wait_heartbeat()
        print(f"Connected! System ID: {self.mav.target_system}, Component ID: {self.mav.target_component}")
        
        # Initialize waypoint loader (helper class for managing waypoints)
        self.wp_loader = mavwp.MAVWPLoader()
    
    def send_heartbeat(self):
        """Send a heartbeat to keep connection alive (GCS should send heartbeats)."""
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
    
    # =========================================================================
    # READING WAYPOINTS FROM DRONE (Mission Download)
    # =========================================================================
    
    def download_mission(self, timeout=10):
        """
        Download the current mission/waypoints from the drone.
        
        This implements the MAVLink Mission Download Protocol:
        1. Send MISSION_REQUEST_LIST
        2. Receive MISSION_COUNT
        3. For each waypoint: Send MISSION_REQUEST_INT, Receive MISSION_ITEM_INT
        4. Send MISSION_ACK
        
        Args:
            timeout: Timeout in seconds for each message
            
        Returns:
            list: List of waypoint dictionaries
        """
        print("\n" + "="*60)
        print("DOWNLOADING MISSION FROM DRONE")
        print("="*60)
        
        waypoints = []
        
        # Step 1: Request the mission list
        print("Requesting mission list...")
        self.mav.waypoint_request_list_send()
        
        # Step 2: Wait for MISSION_COUNT response
        msg = self.mav.recv_match(type=['MISSION_COUNT'], blocking=True, timeout=timeout)
        if msg is None:
            print("ERROR: No response from drone (timeout waiting for MISSION_COUNT)")
            return []
        
        waypoint_count = msg.count
        print(f"Drone reports {waypoint_count} waypoints")
        
        if waypoint_count == 0:
            print("No waypoints stored on drone")
            return []
        
        # Step 3: Request each waypoint one by one
        for i in range(waypoint_count):
            print(f"Requesting waypoint {i}...")
            
            # Request waypoint using MISSION_REQUEST_INT (preferred) or fallback
            self.mav.waypoint_request_send(i)
            
            # Wait for MISSION_ITEM or MISSION_ITEM_INT response
            msg = self.mav.recv_match(
                type=['MISSION_ITEM', 'MISSION_ITEM_INT'], 
                blocking=True, 
                timeout=timeout
            )
            
            if msg is None:
                print(f"ERROR: Timeout waiting for waypoint {i}")
                break
            
            # Parse waypoint data
            wp_data = {
                'seq': msg.seq,
                'frame': msg.frame,
                'command': msg.command,
                'command_name': self._get_command_name(msg.command),
                'current': msg.current,
                'autocontinue': msg.autocontinue,
                'param1': msg.param1,
                'param2': msg.param2,
                'param3': msg.param3,
                'param4': msg.param4,
                'latitude': msg.x if hasattr(msg, 'x') else msg.x / 1e7,
                'longitude': msg.y if hasattr(msg, 'y') else msg.y / 1e7,
                'altitude': msg.z
            }
            
            # Handle MISSION_ITEM_INT which has lat/lon as integers (degrees * 1e7)
            if msg.get_type() == 'MISSION_ITEM_INT':
                wp_data['latitude'] = msg.x / 1e7
                wp_data['longitude'] = msg.y / 1e7
            
            waypoints.append(wp_data)
            print(f"  WP {i}: Lat={wp_data['latitude']:.7f}, "
                  f"Lon={wp_data['longitude']:.7f}, "
                  f"Alt={wp_data['altitude']:.1f}m, "
                  f"Cmd={wp_data['command_name']}")
        
        # Step 4: Send acknowledgment
        self.mav.mav.mission_ack_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_MISSION_ACCEPTED
        )
        
        print(f"\nSuccessfully downloaded {len(waypoints)} waypoints")
        return waypoints
    
    def get_current_position(self, timeout=5):
        """
        Get the drone's current GPS position.
        
        Returns:
            dict: Current position with lat, lon, alt, or None if unavailable
        """
        print("\nRequesting current position...")
        
        # Request GPS position data
        msg = self.mav.recv_match(
            type=['GLOBAL_POSITION_INT'], 
            blocking=True, 
            timeout=timeout
        )
        
        if msg is None:
            print("No position data received")
            return None
        
        position = {
            'latitude': msg.lat / 1e7,
            'longitude': msg.lon / 1e7,
            'altitude_msl': msg.alt / 1000.0,  # mm to meters
            'altitude_rel': msg.relative_alt / 1000.0,  # mm to meters
            'heading': msg.hdg / 100.0 if msg.hdg != 65535 else None  # cdeg to deg
        }
        
        print(f"Current Position: Lat={position['latitude']:.7f}, "
              f"Lon={position['longitude']:.7f}, "
              f"Alt(rel)={position['altitude_rel']:.1f}m")
        
        return position
    
    def get_home_position(self, timeout=5):
        """
        Request and return the home position from the drone.
        
        Returns:
            dict: Home position with lat, lon, alt, or None if unavailable
        """
        print("\nRequesting home position...")
        
        # Send command to request home position
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
            0,  # confirmation
            0, 0, 0, 0, 0, 0, 0  # params (unused)
        )
        
        # Wait for HOME_POSITION message
        msg = self.mav.recv_match(type=['HOME_POSITION'], blocking=True, timeout=timeout)
        
        if msg is None:
            print("No home position received")
            return None
        
        home = {
            'latitude': msg.latitude / 1e7,
            'longitude': msg.longitude / 1e7,
            'altitude': msg.altitude / 1000.0  # mm to meters
        }
        
        print(f"Home Position: Lat={home['latitude']:.7f}, "
              f"Lon={home['longitude']:.7f}, "
              f"Alt={home['altitude']:.1f}m")
        
        return home
    
    # =========================================================================
    # WRITING WAYPOINTS TO DRONE (Mission Upload)
    # =========================================================================
    
    def upload_mission(self, waypoints, timeout=10):
        """
        Upload a mission/waypoints to the drone.
        
        This implements the MAVLink Mission Upload Protocol:
        1. Send MISSION_COUNT
        2. For each waypoint: Receive MISSION_REQUEST_INT, Send MISSION_ITEM_INT
        3. Receive MISSION_ACK
        
        Args:
            waypoints: List of waypoint dictionaries or tuples (lat, lon, alt)
            timeout: Timeout in seconds for each message
            
        Returns:
            bool: True if upload successful, False otherwise
        """
        print("\n" + "="*60)
        print("UPLOADING MISSION TO DRONE")
        print("="*60)
        
        # Clear the waypoint loader and add new waypoints
        self.wp_loader.clear()
        
        # Convert waypoints to MAVLink mission items
        for i, wp in enumerate(waypoints):
            mission_item = self._create_mission_item(i, wp)
            self.wp_loader.add(mission_item)
        
        count = self.wp_loader.count()
        print(f"Preparing to upload {count} waypoints")
        
        # Step 1: Clear existing mission (optional but recommended)
        print("Clearing existing mission...")
        self.mav.waypoint_clear_all_send()
        msg = self.mav.recv_match(type=['MISSION_ACK'], blocking=True, timeout=timeout)
        if msg:
            print(f"Clear ACK received: {msg.type}")
        
        # Step 2: Send mission count
        print(f"Sending mission count: {count}")
        self.mav.waypoint_count_send(count)
        
        # Step 3: Send each waypoint when requested
        for i in range(count):
            # Wait for drone to request the waypoint
            msg = self.mav.recv_match(
                type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], 
                blocking=True, 
                timeout=timeout
            )
            
            if msg is None:
                print(f"ERROR: Timeout waiting for MISSION_REQUEST for waypoint {i}")
                return False
            
            seq = msg.seq
            print(f"Drone requested waypoint {seq}, sending...")
            
            # Send the requested waypoint
            wp_msg = self.wp_loader.wp(seq)
            self.mav.mav.send(wp_msg)
        
        # Step 4: Wait for final acknowledgment
        msg = self.mav.recv_match(type=['MISSION_ACK'], blocking=True, timeout=timeout)
        
        if msg is None:
            print("ERROR: No acknowledgment received")
            return False
        
        if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print(f"\nMission upload SUCCESSFUL! ({count} waypoints)")
            return True
        else:
            print(f"\nMission upload FAILED! Error code: {msg.type}")
            return False
    
    def _create_mission_item(self, seq, waypoint):
        """
        Create a MAVLink mission item from waypoint data.
        
        Args:
            seq: Sequence number (0-indexed)
            waypoint: dict with keys (lat, lon, alt, command) or tuple (lat, lon, alt)
            
        Returns:
            MAVLink_mission_item_message
        """
        # Handle different waypoint formats
        if isinstance(waypoint, dict):
            lat = waypoint.get('latitude', waypoint.get('lat', 0))
            lon = waypoint.get('longitude', waypoint.get('lon', 0))
            alt = waypoint.get('altitude', waypoint.get('alt', 10))
            command = waypoint.get('command', mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)
            frame = waypoint.get('frame', mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
            param1 = waypoint.get('param1', 0)
            param2 = waypoint.get('param2', 0)
            param3 = waypoint.get('param3', 0)
            param4 = waypoint.get('param4', 0)
        elif isinstance(waypoint, (list, tuple)):
            lat, lon, alt = waypoint[0], waypoint[1], waypoint[2] if len(waypoint) > 2 else 10
            command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            param1 = param2 = param3 = param4 = 0
        else:
            raise ValueError(f"Invalid waypoint format: {waypoint}")
        
        # First waypoint should have current=1
        current = 1 if seq == 0 else 0
        autocontinue = 1
        
        # Create mission item message
        mission_item = mavutil.mavlink.MAVLink_mission_item_message(
            self.mav.target_system,
            self.mav.target_component,
            seq,
            frame,
            command,
            current,
            autocontinue,
            param1,
            param2,
            param3,
            param4,
            lat,
            lon,
            alt
        )
        
        return mission_item
    
    def set_home_position(self, lat, lon, alt=0):
        """
        Set the home position on the drone.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters
        """
        print(f"\nSetting home position to: Lat={lat}, Lon={lon}, Alt={alt}")
        
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0,  # confirmation
            0,  # param1: use specified location (0) vs current location (1)
            0,  # param2: unused
            0,  # param3: unused
            0,  # param4: unused
            lat,  # param5: latitude
            lon,  # param6: longitude
            alt   # param7: altitude
        )
        
        # Wait for acknowledgment
        msg = self.mav.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=5)
        if msg:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Home position set successfully!")
            else:
                print(f"Failed to set home position. Result: {msg.result}")
    
    def clear_mission(self, timeout=5):
        """
        Clear all waypoints from the drone's mission.
        
        Returns:
            bool: True if successful
        """
        print("\nClearing all waypoints from drone...")
        
        self.mav.waypoint_clear_all_send()
        
        msg = self.mav.recv_match(type=['MISSION_ACK'], blocking=True, timeout=timeout)
        
        if msg and msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print("Mission cleared successfully!")
            return True
        else:
            print("Failed to clear mission")
            return False
    
    def set_current_waypoint(self, seq):
        """
        Set the current mission waypoint.
        
        Args:
            seq: Waypoint sequence number to set as current
        """
        print(f"\nSetting current waypoint to {seq}")
        
        self.mav.waypoint_set_current_send(seq)
        
        # Wait for MISSION_CURRENT message
        msg = self.mav.recv_match(type=['MISSION_CURRENT'], blocking=True, timeout=5)
        if msg:
            print(f"Current waypoint is now: {msg.seq}")
    
    # =========================================================================
    # HELPER METHODS
    # =========================================================================
    
    def _get_command_name(self, command_id):
        """Get human-readable name for a MAV_CMD."""
        cmd_names = {
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT: 'WAYPOINT',
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM: 'LOITER_UNLIM',
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS: 'LOITER_TURNS',
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME: 'LOITER_TIME',
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH: 'RTL',
            mavutil.mavlink.MAV_CMD_NAV_LAND: 'LAND',
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF: 'TAKEOFF',
            mavutil.mavlink.MAV_CMD_DO_SET_HOME: 'SET_HOME',
            16: 'WAYPOINT',
            17: 'LOITER_UNLIM',
            18: 'LOITER_TURNS',
            19: 'LOITER_TIME',
            20: 'RTL',
            21: 'LAND',
            22: 'TAKEOFF',
        }
        return cmd_names.get(command_id, f'CMD_{command_id}')
    
    def create_simple_mission(self, home_lat, home_lon, waypoints_relative):
        """
        Create a simple mission with takeoff, waypoints, and RTL.
        
        Args:
            home_lat: Home latitude
            home_lon: Home longitude
            waypoints_relative: List of (dlat, dlon, alt) tuples relative to home
            
        Returns:
            list: Mission items ready for upload
        """
        mission = []
        
        # Waypoint 0: Takeoff
        mission.append({
            'latitude': home_lat,
            'longitude': home_lon,
            'altitude': 10,
            'command': mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            'param1': 15  # Pitch angle
        })
        
        # Add navigation waypoints
        for dlat, dlon, alt in waypoints_relative:
            mission.append({
                'latitude': home_lat + dlat,
                'longitude': home_lon + dlon,
                'altitude': alt,
                'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            })
        
        # Final waypoint: Return to Launch
        mission.append({
            'latitude': home_lat,
            'longitude': home_lon,
            'altitude': 0,
            'command': mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        })
        
        return mission
    
    def close(self):
        """Close the MAVLink connection."""
        self.mav.close()
        print("\nConnection closed")


# =============================================================================
# EXAMPLE USAGE AND DEMO
# =============================================================================

def demo_read_mission(manager):
    """Demonstrate reading/downloading mission from drone."""
    print("\n" + "="*70)
    print("DEMO: Reading Mission from Drone")
    print("="*70)
    
    # Get current position
    manager.get_current_position()
    
    # Get home position
    manager.get_home_position()
    
    # Download current mission
    waypoints = manager.download_mission()
    
    if waypoints:
        print("\n--- Downloaded Waypoints Summary ---")
        for wp in waypoints:
            print(f"  [{wp['seq']}] {wp['command_name']}: "
                  f"({wp['latitude']:.6f}, {wp['longitude']:.6f}) @ {wp['altitude']}m")
    
    return waypoints


def demo_write_mission(manager):
    """Demonstrate writing/uploading mission to drone."""
    print("\n" + "="*70)
    print("DEMO: Writing Mission to Drone")
    print("="*70)
    
    # Example waypoints (latitude, longitude, altitude)
    # Using coordinates near a common test location
    example_waypoints = [
        # (lat, lon, alt) - simple format
        (37.5091, 127.0451, 15),  # Waypoint 1
        (37.5093, 127.0455, 20),  # Waypoint 2
        (37.5095, 127.0460, 25),  # Waypoint 3
        (37.5091, 127.0451, 10),  # Return to start
    ]
    
    # Alternative: Using dictionary format with more options
    detailed_waypoints = [
        {
            'latitude': 37.5091,
            'longitude': 127.0451,
            'altitude': 10,
            'command': mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            'param1': 15  # Minimum pitch
        },
        {
            'latitude': 37.5093,
            'longitude': 127.0455,
            'altitude': 20,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        },
        {
            'latitude': 37.5095,
            'longitude': 127.0460,
            'altitude': 25,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        },
        {
            'latitude': 37.5091,
            'longitude': 127.0451,
            'altitude': 0,
            'command': mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        }
    ]
    
    print("\nWaypoints to upload:")
    for i, wp in enumerate(detailed_waypoints):
        cmd_name = manager._get_command_name(wp.get('command', 16))
        print(f"  [{i}] {cmd_name}: ({wp['latitude']}, {wp['longitude']}) @ {wp['altitude']}m")
    
    # Upload the mission
    success = manager.upload_mission(detailed_waypoints)
    
    return success


def main():
    """Main function demonstrating the waypoint manager."""
    parser = argparse.ArgumentParser(
        description='MAVLink Waypoint Manager - Upload/Download waypoints to/from drone'
    )
    parser.add_argument(
        '--connect', 
        default='udp:127.0.0.1:14550',
        help='Connection string (e.g., /dev/ttyUSB0, udp:127.0.0.1:14550, tcp:127.0.0.1:5760)'
    )
    parser.add_argument(
        '--baud', 
        type=int, 
        default=57600,
        help='Baud rate for serial connections'
    )
    parser.add_argument(
        '--action',
        choices=['read', 'write', 'both', 'clear'],
        default='both',
        help='Action to perform: read, write, both, or clear'
    )
    
    args = parser.parse_args()
    
    print("="*70)
    print("MAVLink Waypoint Manager")
    print("Based on Mission Planner / ArduPilot MAVLink implementation")
    print("="*70)
    
    try:
        # Initialize the waypoint manager
        manager = MAVLinkWaypointManager(args.connect, args.baud)
        
        if args.action == 'read':
            demo_read_mission(manager)
            
        elif args.action == 'write':
            demo_write_mission(manager)
            
        elif args.action == 'clear':
            manager.clear_mission()
            
        else:  # 'both'
            # First read current mission
            demo_read_mission(manager)
            
            # Then upload a new mission
            demo_write_mission(manager)
            
            # Verify by reading back
            print("\n--- Verifying uploaded mission ---")
            demo_read_mission(manager)
        
        manager.close()
        
    except KeyboardInterrupt:
        print("\n\nOperation cancelled by user")
    except Exception as e:
        print(f"\nError: {e}")
        raise


if __name__ == '__main__':
    main()
