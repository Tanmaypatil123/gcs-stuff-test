# Mission Planner Standalone Script

## Overview

This is a standalone Python script for **mission planning** with ArduPilot/PX4 autopilots. It uses the **robust connection logic from ui6.py** combined with comprehensive mission planning capabilities.

## Features

### ✅ Connection (from ui6.py)
- **Auto-detection** of serial ports and baud rates
- **Intelligent scanning** with priority for Bluetooth SPP and preferred ports
- **Robust MAVLink parsing** with heartbeat detection
- **Windows COM port support** (handles COM10+ correctly)

### ✅ Mission Planning Functions
1. **Write Mission** - Upload waypoints to autopilot
2. **Read Mission** - Download waypoints from autopilot
3. **Delete Mission** - Clear all waypoints
4. **Set Home Location** - Configure home position
5. **Get Home Location** - Retrieve current home position
6. **Set Flight Mode** - Change modes (GUIDED, AUTO, RTL, etc.)
7. **Arm/Disarm** - Arm or disarm the vehicle

## Installation

### Prerequisites

```bash
pip install pyserial pymavlink
```

## Usage

### Quick Start

```bash
python mission_planner_standalone.py
```

The script will:
1. **Auto-detect** your flight controller connection
2. Show connection details
3. Present an interactive menu

### Menu Options

```
1. Write Mission (Upload demo waypoints)
2. Read Mission (Download from autopilot)
3. Delete Mission (Clear all waypoints)
4. Set Home Location
5. Get Home Location
6. Set Flight Mode
7. Arm/Disarm
8. Reconnect
9. Exit
```

## How It Works

### Connection Logic (from ui6.py)

The script uses the **exact same connection logic** as ui6.py:

1. **Port Discovery**
   - Lists all serial ports
   - Prioritizes: COM5, COM4, Bluetooth SPP, then others

2. **Auto-Detection**
   - Probes each port at multiple baud rates (115200, 57600, 38400, 9600)
   - Looks for MAVLink heartbeat messages
   - Scores candidates based on flight controller probability

3. **Connection Establishment**
   - Opens serial connection with robust settings
   - Waits for autopilot heartbeat (10 second timeout)
   - Extracts system ID, component ID, vehicle type, autopilot type

### Mission Planning

#### Write Mission (Upload)

```python
# Creates mission from DEMO_WAYPOINTS:
# - First waypoint: TAKEOFF (15m altitude, 15° minimum pitch)
# - Middle waypoints: WAYPOINT (15m altitude)
# - Last waypoint: LAND (0m altitude)

1. Clears existing mission
2. Sends waypoint count
3. Waits for autopilot to request each waypoint
4. Sends waypoints sequentially
5. Waits for ACK confirmation
```

#### Read Mission (Download)

```python
1. Requests mission list from autopilot
2. Gets waypoint count
3. Requests each waypoint individually
4. Displays waypoint details (command, lat, lon, alt)
5. Sends ACK to autopilot
```

#### Delete Mission (Clear)

```python
1. Sends clear all waypoints command
2. Waits for ACK confirmation
```

## Demo Waypoints

The script includes demo waypoints from your example code:

```python
DEMO_WAYPOINTS = [
    (37.5090904347, 127.045094298),  # TAKEOFF
    (37.509070898, 127.048905867),   # WAYPOINT
    (37.5063678607, 127.048960654),  # WAYPOINT
    (37.5061713129, 127.044741936),  # WAYPOINT
    (37.5078823794, 127.046914506)   # LAND
]
```

These are automatically converted to:
- **WP 0**: TAKEOFF at 15m
- **WP 1-3**: Waypoints at 15m
- **WP 4**: LAND at 0m

## Customization

### Adding Custom Waypoints

Edit the `DEMO_WAYPOINTS` list in the script:

```python
DEMO_WAYPOINTS = [
    (your_lat, your_lon),
    (your_lat2, your_lon2),
    # ... add more coordinates
]
```

### Changing Altitude

Modify the `create_demo_mission()` function:

```python
# Change default altitude from 15m to 30m
wp = Waypoint(lat, lon, alt=30.0, cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)
```

### Manual Port/Baud Selection

In the script, modify the connection call:

```python
# Auto-detect (default)
mav = connect_to_autopilot(auto=True)

# Manual selection
mav = connect_to_autopilot(port="COM5", baud=115200, auto=False)
```

## Differences from Original Code

### ✅ Improvements

| Original Code | This Script |
|--------------|-------------|
| Fixed UDP connection | **Auto-detection** of serial ports |
| Manual port/baud | **Intelligent scanning** |
| No error handling | **Robust error handling** |
| Basic connection | **Connection from ui6.py** (proven robust) |
| No menu system | **Interactive menu** |
| PX4-specific modes | **ArduPilot + PX4 compatible** |

### ✅ Retained Functionality

- ✅ Set/Get Home Location
- ✅ Write Mission (waypoint upload)
- ✅ Read Mission (waypoint download)
- ✅ Clear Mission
- ✅ Arm/Disarm
- ✅ Set Flight Mode
- ✅ Mission structure (TAKEOFF → WAYPOINT → LAND)

## Connection from ui6.py - What Was Extracted?

### Functions Used:
1. **`win_path()`** - Windows COM port path conversion
2. **`list_candidate_ports()`** - Port discovery with priority
3. **`probe_port()`** - MAVLink traffic detection
4. **`auto_detect_connection()`** - Smart port/baud scanning
5. **`connect_to_autopilot()`** - Main connection function

### Key Features:
- Robust parsing (handles corrupted packets)
- Heartbeat-based detection
- Flight controller identification
- Proper serial settings (no flow control)
- Target system/component extraction

## Troubleshooting

### "No serial ports found"
- Check USB/Bluetooth connection
- Verify Device Manager shows COM port
- Try unplugging and reconnecting

### "No MAVLink connection found"
- Ensure flight controller is powered
- Check correct baud rate (usually 115200 or 57600)
- Verify MAVLink telemetry is enabled on autopilot

### "Timeout waiting for heartbeat"
- Flight controller may be booting (wait 10-15 seconds)
- Wrong baud rate (try manual selection)
- MAVLink protocol version mismatch

### Connection works but mission upload fails
- Check autopilot is in correct mode (not ARMED)
- Verify SD card is inserted (some autopilots require it)
- Try clearing mission first (option 3)

## Example Session

```
[INFO] Scanning for MAVLink connections...
  Scanning COM5...
    115200 baud: msgs=45 heartbeats=8 fc_candidates=1

[CONNECT] Opening COM5 @ 115200 baud...
[CONNECT] Waiting for autopilot heartbeat...
[OK] Connected to autopilot:
     System ID: 1, Component ID: 1
     Type: MAV_TYPE_QUADROTOR
     Autopilot: MAV_AUTOPILOT_ARDUPILOTMEGA
     Mode: STABILIZE
     Armed: False

============================================================
  MISSION PLANNER - Main Menu
============================================================
  1. Write Mission (Upload demo waypoints)
  2. Read Mission (Download from autopilot)
  3. Delete Mission (Clear all waypoints)
  ...

Enter choice (1-9): 1

Demo mission has 5 waypoints:
  0: TAKEOFF | 37.5090904, 127.0450943, 15.0m
  1: WAYPOINT | 37.5090709, 127.0489059, 15.0m
  2: WAYPOINT | 37.5063679, 127.0489607, 15.0m
  3: WAYPOINT | 37.5061713, 127.0447419, 15.0m
  4: LAND | 37.5078824, 127.0469145, 0.0m

Upload this mission? (y/n): y

[WRITE MISSION] Uploading 5 waypoints...
  Sent waypoint 0: 37.5090904, 127.0450943, 15.0m
  Sent waypoint 1: 37.5090709, 127.0489059, 15.0m
  Sent waypoint 2: 37.5063679, 127.0489607, 15.0m
  Sent waypoint 3: 37.5061713, 127.0447419, 15.0m
  Sent waypoint 4: 37.5078824, 127.0469145, 0.0m
[OK] Mission uploaded successfully!
```

## Safety Notes

⚠️ **IMPORTANT**:
- Always test missions in **simulator** first (SITL)
- Never arm indoors without propellers removed
- Ensure GPS lock before arming
- Check battery voltage
- Verify failsafe settings (RTL, battery failsafe)
- Keep manual control transmitter ready

## Technical Details

### MAVLink Protocol
- Uses **pymavlink** library
- MAVLink v2.0 protocol (ardupilotmega dialect)
- Robust parsing enabled (tolerates packet corruption)

### Mission Protocol
- **MISSION_COUNT** → Request mission upload
- **MISSION_REQUEST** → Request specific waypoint
- **MISSION_ITEM** → Send waypoint data
- **MISSION_ACK** → Confirm success/failure

### Coordinate System
- **Frame**: MAV_FRAME_GLOBAL_RELATIVE_ALT (relative to home altitude)
- **Latitude/Longitude**: Decimal degrees
- **Altitude**: Meters above home position

## License

This script combines:
- Connection logic from **ui6.py** (your existing GCS)
- Mission planning from **community examples**

Use freely for your GCS project!

## Next Steps

### Integration with ui6.py

To add mission planning to your full GCS UI:

1. Import functions from this script:
```python
from mission_planner_standalone import write_mission, read_mission, delete_mission
```

2. Use the shared MAVLink worker:
```python
# In MapDashboard or new MissionPlanner tab
write_mission(self.worker.mav, waypoints)
```

3. Add mission planning tab to `QStackedWidget`

Would you like help integrating this into ui6.py as a new tab?
