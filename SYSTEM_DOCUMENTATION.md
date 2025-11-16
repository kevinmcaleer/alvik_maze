# Alvik ROS2 Mapping & Navigation System
## Complete System Documentation for YouTube Tutorial

---

## Table of Contents
1. [System Overview](#system-overview)
2. [Architecture Diagram](#architecture-diagram)
3. [Hardware Components](#hardware-components)
4. [Software Components](#software-components)
5. [ROS2 Topics](#ros2-topics)
6. [ROS2 Nodes](#ros2-nodes)
7. [Alvik Firmware](#alvik-firmware)
8. [MQTT Communication](#mqtt-communication)
9. [Coordinate Frames (TF)](#coordinate-frames-tf)
10. [Setup Instructions](#setup-instructions)
11. [Usage Guide](#usage-guide)

---

## System Overview

This system enables an Arduino Alvik robot to perform autonomous maze exploration and mapping using ROS2, SLAM (Simultaneous Localization and Mapping), and Time-of-Flight (ToF) sensors.

### Key Capabilities
- **Autonomous maze navigation** using right-hand wall following
- **Real-time mapping** using SLAM Toolbox
- **Live visualization** in Foxglove Studio
- **Hybrid scanning** - continuous front-facing + triggered 360° scans
- **Remote control** via keyboard teleoperation
- **Click-to-move** navigation in Foxglove

### Technology Stack
- **Robot**: Arduino Alvik (MicroPython firmware)
- **ROS Distribution**: ROS2 Jazzy
- **SLAM**: SLAM Toolbox (graph-based SLAM)
- **Visualization**: Foxglove Studio
- **Communication**: MQTT (robot ↔ ROS2)
- **Container**: Docker

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        ALVIK ROBOT                              │
│  ┌────────────────────────────────────────────────────────┐    │
│  │  MicroPython Firmware (main.py)                        │    │
│  │  - 5× ToF sensors (front-facing array)                 │    │
│  │  - Continuous scanning (25 points, 45° FOV)            │    │
│  │  - Triggered 360° rotation scans                       │    │
│  │  - Motor control & odometry                            │    │
│  │  - MQTT publisher/subscriber                           │    │
│  └────────────────────────────────────────────────────────┘    │
└──────────────────────────┬──────────────────────────────────────┘
                           │ WiFi
                           │ MQTT Messages
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│                    MQTT BROKER (Mosquitto)                      │
│                     Topics:                                     │
│                     - alvik/scan                                │
│                     - alvik/odom                                │
│                     - alvik/command                             │
│                     - alvik/explore                             │
└──────────────────────────┬──────────────────────────────────────┘
                           │
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│                  ROS2 DOCKER CONTAINER                          │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  ROS2 Nodes:                                             │  │
│  │                                                           │  │
│  │  1. tof_receiver                                         │  │
│  │     - Subscribes: alvik/scan (MQTT)                      │  │
│  │     - Publishes: /scan (LaserScan)                       │  │
│  │                                                           │  │
│  │  2. odom_publisher                                       │  │
│  │     - Subscribes: alvik/odom (MQTT)                      │  │
│  │     - Publishes: /odom (Odometry)                        │  │
│  │     - Broadcasts: TF transforms                          │  │
│  │                                                           │  │
│  │  3. robot_state_publisher                                │  │
│  │     - Publishes: /robot_description                      │  │
│  │     - Broadcasts: URDF-based transforms                  │  │
│  │                                                           │  │
│  │  4. slam_toolbox (async)                                 │  │
│  │     - Subscribes: /scan, /odom                           │  │
│  │     - Publishes: /map                                    │  │
│  │     - Broadcasts: map → odom transform                   │  │
│  │                                                           │  │
│  │  5. click_to_move                                        │  │
│  │     - Subscribes: /clicked_point                         │  │
│  │     - Publishes: alvik/command (MQTT)                    │  │
│  │                                                           │  │
│  │  6. maze_explorer                                        │  │
│  │     - Subscribes: /scan, /odom                           │  │
│  │     - Publishes: alvik/command (MQTT)                    │  │
│  │     - Logic: Right-hand wall following                   │  │
│  │                                                           │  │
│  │  7. teleop_keyboard                                      │  │
│  │     - User input: Keyboard                               │  │
│  │     - Publishes: alvik/command (MQTT)                    │  │
│  │                                                           │  │
│  │  8. foxglove_bridge                                      │  │
│  │     - WebSocket server (port 8765)                       │  │
│  │     - Streams all ROS2 topics to Foxglove                │  │
│  └──────────────────────────────────────────────────────────┘  │
└──────────────────────────┬──────────────────────────────────────┘
                           │ WebSocket
                           ↓
                  ┌─────────────────┐
                  │  FOXGLOVE       │
                  │  STUDIO         │
                  │  (localhost)    │
                  │                 │
                  │  - 3D view      │
                  │  - Map view     │
                  │  - Laser scans  │
                  │  - Robot model  │
                  └─────────────────┘
```

---

## Hardware Components

### Arduino Alvik Robot
- **Microcontroller**: ESP32-based
- **Programming**: MicroPython
- **Sensors**:
  - 5× VL53L5CX Time-of-Flight (ToF) sensors (front array)
  - 2× Additional ToF sensors (top, bottom - not used)
  - Wheel encoders for odometry
- **Actuators**: 2× DC motors with differential drive
- **Communication**: WiFi (2.4GHz)

### ToF Sensor Configuration
The Alvik has **7 separate ToF sensors**, but we primarily use the **5 front-facing sensors**:
- `_left_tof`
- `_center_left_tof`
- `_center_tof`
- `_center_right_tof`
- `_right_tof`

Each sensor provides a **single distance measurement**, giving us 5 readings across a ~45° field of view.

---

## Software Components

### 1. Docker Container
**Image**: `osrf/ros:jazzy-desktop-full`
**Purpose**: Provides ROS2 environment isolated from host system
**Key Packages Installed**:
- `ros-jazzy-slam-toolbox` - Graph-based SLAM
- `ros-jazzy-foxglove-bridge` - Foxglove visualization
- `mosquitto` - MQTT broker
- `mosquitto-clients` - MQTT command-line tools

### 2. ROS2 Workspace
**Location**: `/ros2_ws/src/alvik_navigation`
**Build System**: Colcon
**Package Type**: Python (setuptools)

---

## ROS2 Topics

### Published Topics

| Topic | Message Type | Publisher | Description |
|-------|-------------|-----------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | tof_receiver | ToF sensor data converted to laser scan format |
| `/odom` | `nav_msgs/Odometry` | odom_publisher | Robot position and velocity |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox | 2D occupancy grid map |
| `/robot_description` | `std_msgs/String` | robot_state_publisher | URDF robot model |
| `/tf` | `tf2_msgs/TFMessage` | odom_publisher, slam_toolbox | Transform tree |
| `/tf_static` | `tf2_msgs/TFMessage` | robot_state_publisher | Static transforms |

### Subscribed Topics

| Topic | Subscriber Nodes | Purpose |
|-------|-----------------|---------|
| `/scan` | slam_toolbox, maze_explorer | SLAM and navigation input |
| `/odom` | slam_toolbox, maze_explorer | Odometry for SLAM and navigation |
| `/clicked_point` | click_to_move | User clicks in Foxglove |

### LaserScan Message Details
```python
# /scan topic (sensor_msgs/LaserScan)
header:
  frame_id: "alvik_laser"
angle_min: -0.39 rad  # -22.5° (left edge of FOV)
angle_max: 0.39 rad   # +22.5° (right edge of FOV)
angle_increment: 0.031 rad  # ~1.8° between points
range_min: 0.02 m     # 2cm minimum range
range_max: 2.0 m      # 2m maximum range
ranges: [...]         # 25 distance measurements (5 sensors × 5 interpolation)
```

---

## ROS2 Nodes

### 1. **tof_receiver.py**
**Purpose**: Bridge between MQTT ToF data and ROS2 LaserScan messages

**Subscriptions (MQTT)**:
- `alvik/scan` - Individual scan points (angle, distance)
- `alvik/scan_complete` - Signal that scan is ready to publish

**Publications (ROS2)**:
- `/scan` (LaserScan) - Accumulated scan points

**Key Functionality**:
```python
# Accumulates individual MQTT messages:
# "37.5,48.4" → angle=37.5°, distance=48.4cm
# Until scan_complete signal received

# Then publishes as LaserScan with:
# - 25 range measurements
# - 45° field of view
# - Proper timestamps and frame_id
```

**Parameters**:
- `mqtt_host`: IP address of MQTT broker (default: 192.168.1.152)
- `mqtt_port`: MQTT port (default: 1883)

---

### 2. **odom_publisher.py**
**Purpose**: Publish robot odometry and TF transforms

**Subscriptions (MQTT)**:
- `alvik/odom` - Position updates `{x, y, theta}`

**Publications (ROS2)**:
- `/odom` (Odometry) - Full odometry message
- `/tf` (TFMessage) - Transform: odom → base_link

**Key Functionality**:
```python
# Receives: {"x": 0.5, "y": 0.3, "theta": 1.57}
# Publishes:
#   - Odometry message with pose and twist
#   - TF transform from odom to base_link
#   - Covariance for uncertainty
```

**TF Broadcast**:
- Parent frame: `odom`
- Child frame: `base_link`
- Updates at 10 Hz

---

### 3. **robot_state_publisher** (Standard ROS2 Node)
**Purpose**: Publish robot's kinematic tree from URDF

**Input**: `alvik.urdf` - Robot description file

**Publications**:
- `/robot_description` (String) - URDF XML
- `/tf_static` (TFMessage) - Fixed transforms
- `/joint_states` (JointState) - Joint positions

**Transforms Published**:
- `base_link` → `top_surface`
- `base_link` → `left_wheel`
- `base_link` → `right_wheel`
- `base_link` → `alvik_laser` (ToF sensor position)

---

### 4. **slam_toolbox** (Standard ROS2 Node)
**Purpose**: Perform graph-based SLAM to build 2D map

**Subscriptions**:
- `/scan` (LaserScan)
- `/odom` (Odometry)

**Publications**:
- `/map` (OccupancyGrid) - 2D map
- `/map_metadata` (MapMetaData)
- `/tf` (TFMessage) - Transform: map → odom

**Configuration** (`mapper_params_online_async.yaml`):
```yaml
slam_toolbox:
  ros__parameters:
    mode: mapping

    # Map parameters
    resolution: 0.01  # 1cm per pixel
    map_update_interval: 1.0  # Update every 1 second

    # Scan matching
    max_laser_range: 2.0  # 2 meters
    minimum_travel_distance: 0.02  # 2cm before updating
    minimum_travel_heading: 0.1  # ~6° rotation

    # Loop closure
    loop_search_maximum_distance: 3.0
    do_loop_closing: true

    # Performance
    use_scan_matching: true
    scan_buffer_size: 10
```

**SLAM Process**:
1. Receives laser scan + odometry
2. Performs scan-to-map matching
3. Updates pose graph
4. Detects loop closures
5. Publishes optimized map
6. Broadcasts map → odom transform

---

### 5. **click_to_move.py**
**Purpose**: Allow user to click points in Foxglove to navigate robot

**Subscriptions (ROS2)**:
- `/clicked_point` (PointStamped) - User clicks in Foxglove

**Publications (MQTT)**:
- `alvik/command` - Movement commands

**Key Functionality**:
```python
# When user clicks point (x, y) in map:
# 1. Get current robot position from /odom
# 2. Calculate distance and bearing to target
# 3. Send rotate + move commands via MQTT
# 4. Robot turns to face target, then drives straight
```

**Coordinate Transform**:
```python
# Clicked point is in 'map' frame
# Robot odometry is in 'odom' frame
# Uses TF to transform between frames
target_in_odom = tf_buffer.transform(clicked_point, 'odom')
```

---

### 6. **maze_explorer.py**
**Purpose**: Autonomous maze exploration using right-hand wall following

**Subscriptions (ROS2)**:
- `/scan` (LaserScan) - For obstacle detection
- `/odom` (Odometry) - For position tracking

**Subscriptions (MQTT)**:
- `alvik/explore` - Start/stop commands

**Publications (MQTT)**:
- `alvik/command` - Navigation commands

**Algorithm**: Right-Hand Wall Following
```python
State Machine:
1. FOLLOW_WALL:
   - Maintain 15cm distance from right wall
   - Check for obstacles ahead
   - Detect intersections (gaps > 40cm)
   - Priority: Turn right > Straight > Turn left

2. AT_INTERSECTION:
   - Trigger 360° scan
   - Choose direction (right-hand rule)
   - Execute turn/move
   - Return to FOLLOW_WALL

3. AT_DEAD_END:
   - Trigger 360° scan
   - Turn 180°
   - Return to FOLLOW_WALL
```

**Scan Region Analysis**:
```python
# Divides 45° FOV into 3 regions:
# Left: > 15°
# Front: ±15°
# Right: < -15°

# Calculates average distance in each region
# Uses for decision making
```

**Parameters**:
- `wall_distance`: 0.15m (target distance from wall)
- `forward_speed`: 5 cm/s
- `turn_speed`: 30 deg/s
- `min_front_distance`: 0.25m (stop threshold)
- `intersection_threshold`: 0.40m (gap detection)

---

### 7. **teleop_keyboard.py**
**Purpose**: Manual keyboard control of robot

**User Input**: Keyboard (via terminal)

**Publications (MQTT)**:
- `alvik/command` - Movement commands

**Key Bindings**:
```
w / ↑  : Move forward 10cm
s / ↓  : Move backward 10cm
a / ←  : Rotate left 15°
d / →  : Rotate right 15°
space  : Stop
q      : Quit
```

**Implementation**:
```python
# Uses raw terminal input (termios)
# Non-blocking keyboard read
# Publishes MQTT commands:
# {"type": "move", "distance": 10}
# {"type": "rotate", "angle": 15}
# {"type": "stop"}
```

---

### 8. **foxglove_bridge** (Standard ROS2 Node)
**Purpose**: Stream ROS2 data to Foxglove Studio via WebSocket

**Protocol**: Foxglove WebSocket Protocol
**Port**: 8765
**Address**: 0.0.0.0 (accessible from host)

**Streams All ROS2 Topics**:
- `/scan` → LaserScan visualization
- `/map` → 2D occupancy grid
- `/odom` → Robot pose
- `/tf` → Transform tree
- `/robot_description` → 3D robot model

**Foxglove Configuration**:
```
Connection: ws://localhost:8765
Topics to visualize:
  - /scan (3D markers - red dots)
  - /map (map layer)
  - /tf (3D robot model)
```

---

## Alvik Firmware

### **main.py** - Complete Firmware Overview

**Location**: `/Users/kev/Python/alvik_maze/alvik_firmware/main.py`
**Language**: MicroPython
**Size**: ~660 lines

**Core Responsibilities**:
1. WiFi connection management
2. MQTT communication
3. Continuous ToF sensor publishing
4. Triggered 360° scan execution
5. Motor control (move/rotate)
6. Dead reckoning odometry
7. Command handling

---

### Firmware Architecture

```python
# === IMPORTS ===
from arduino_alvik import ArduinoAlvik
import network  # WiFi
import uasyncio as asyncio  # Non-blocking operations
from umqtt.robust import MQTTClient  # MQTT library
import math  # Odometry calculations

# === INITIALIZATION ===
alvik = ArduinoAlvik()
alvik.begin()

# Robot state (dead reckoning)
robot_x = 0.0  # Position in cm
robot_y = 0.0
robot_theta = 0.0  # Heading in radians

# === MAIN FUNCTIONS ===
1. connect_wifi()
2. connect_mqtt()
3. on_mqtt_message(topic, msg)  # Command handler
4. continuous_tof_publisher()   # Background task
5. perform_scan()               # 360° rotation scan
6. move_robot(distance)
7. rotate_robot(angle)
8. publish_odometry()
9. main_loop()  # Asyncio event loop
```

---

### Key Functions Breakdown

#### 1. **WiFi Connection**
```python
def connect_wifi():
    """Connect to WiFi network"""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)

    # Wait up to 50 seconds for connection
    timeout = 50
    while not wlan.isconnected() and timeout > 0:
        time.sleep(1)
        timeout -= 1

    if wlan.isconnected():
        print("WiFi connected!")
        print("IP: {}".format(wlan.ifconfig()[0]))
        return True
    return False
```

**Credentials**: Stored in `my_secrets.py`:
```python
WIFI_SSID = "your_network_name"
WIFI_PASSWORD = "your_password"
MQTT_BROKER = "192.168.1.152"  # ROS2 container IP
MQTT_PORT = 1883
```

---

#### 2. **MQTT Connection**
```python
def connect_mqtt():
    """Connect to MQTT broker"""
    client = MQTTClient(
        client_id="alvik_robot",
        server=MQTT_BROKER,
        port=MQTT_PORT,
        keepalive=120
    )

    client.connect()
    client.set_callback(on_mqtt_message)
    client.subscribe(b"alvik/command")  # Listen for commands

    return client
```

**MQTT Topics Used**:
- **Subscribe**: `alvik/command` - Receive navigation commands
- **Publish**:
  - `alvik/scan` - Individual scan points
  - `alvik/scan_complete` - Scan ready signal
  - `alvik/odom` - Odometry updates
  - `alvik/status` - Status messages
  - `alvik/debug` - Debug information

---

#### 3. **Continuous ToF Publisher** (Background Task)
```python
async def continuous_tof_publisher():
    """Publishes ToF sensor data at ~5 Hz"""
    FOV_HORIZONTAL = 45.0  # Field of view in degrees

    while True:
        # Get current robot heading
        current_angle = math.degrees(robot_theta) % 360

        # Read 5 ToF sensors (returns tuple of 5 distances)
        dist = alvik.get_distance()  # Returns: (d1, d2, d3, d4, d5)

        if isinstance(dist, tuple):
            num_zones = len(dist)  # 5 sensors
            fov_per_zone = FOV_HORIZONTAL / num_zones  # 9° per sensor
            interpolation_factor = 5  # Create 5 points per sensor

            # For each sensor reading
            for zone_idx, zone_dist in enumerate(dist):
                # Calculate base angle for this sensor
                # Center sensor = 0°, left sensors = positive, right = negative
                zone_angle_offset = (zone_idx - 2) * fov_per_zone
                base_angle = (current_angle + zone_angle_offset) % 360

                # Create 5 interpolated points per sensor
                for interp_idx in range(interpolation_factor):
                    # Spread points across sensor's FOV
                    angle_delta = (interp_idx - 2) * (fov_per_zone / interpolation_factor)
                    interp_angle = (base_angle + angle_delta) % 360

                    # Publish: "angle,distance"
                    message = "{:.2f},{}".format(interp_angle, zone_dist)
                    mqtt_client.publish(b"alvik/scan", message.encode())

            # Signal scan complete
            mqtt_client.publish(b"alvik/scan_complete", b"1")

        await asyncio.sleep(0.2)  # 5 Hz update rate
```

**Result**: 25 scan points per scan (5 sensors × 5 interpolation)

---

#### 4. **360° Rotation Scan**
```python
async def perform_scan():
    """Perform encoder-controlled 360° rotation while scanning"""
    scanning = True
    num_readings = 360  # One reading per degree
    rotation_speed = 30  # Motor speed

    # Calculate encoder target for 360° rotation
    # Uses wheel diameter, wheelbase, and calibration factor
    WHEEL_DIAMETER_CM = 3.9
    WHEEL_BASE_CM = 14.3
    ROTATION_CALIBRATION = 0.73

    arc_length_cm = 3.14159 * WHEEL_BASE_CM
    wheel_rotations = arc_length_cm / (3.14159 * WHEEL_DIAMETER_CM)
    target_encoder_degrees = wheel_rotations * 360 * ROTATION_CALIBRATION

    # Reset motors
    alvik.set_wheels_speed(0, 0)
    await asyncio.sleep(0.5)

    # Get starting encoder positions
    left_start, right_start = alvik.get_wheels_position()

    # Start continuous rotation (clockwise)
    alvik.set_wheels_speed(-rotation_speed, rotation_speed)

    # Scan loop
    for i in range(num_readings):
        # Read current encoder positions
        left_pos, right_pos = alvik.get_wheels_position()
        avg_encoder_delta = (abs(left_pos - left_start) +
                            abs(right_pos - right_start)) / 2.0

        # Calculate actual rotation angle from encoders
        actual_rotation = (avg_encoder_delta / target_encoder_degrees) * 360.0
        angle = actual_rotation % 360

        # Check if completed 360°
        if actual_rotation >= 360.0:
            alvik.set_wheels_speed(0, 0)
            break

        # Get ToF reading
        await asyncio.sleep(0.03)  # Time for sensor
        dist = alvik.get_distance()

        # Publish scan points (with FOV distribution)
        # ... (similar to continuous publisher)

        # Update odometry
        robot_theta = math.radians(angle)
        if i % 10 == 0:
            publish_odometry()

    # Stop rotation
    alvik.set_wheels_speed(0, 0)
    mqtt_client.publish(b"alvik/scan_complete", b"1")
```

**Key Features**:
- **Encoder-based rotation** (not time-based)
- **Real-time angle calculation** from wheel positions
- **Overshoot correction** if rotation > 360°
- **Publishes ~360 scan points** (one per degree)

---

#### 5. **Motor Control**
```python
def move_robot(distance_cm):
    """Move forward/backward"""
    alvik.move(distance_cm)  # Blocking call

    # Update odometry (dead reckoning)
    distance_m = distance_cm / 100.0
    robot_x += distance_m * math.cos(robot_theta)
    robot_y += distance_m * math.sin(robot_theta)

    publish_odometry()

def rotate_robot(angle_deg):
    """Rotate in place"""
    alvik.rotate(angle_deg)  # Blocking call

    # Update heading
    robot_theta += math.radians(angle_deg)

    # Normalize to [-π, π]
    while robot_theta > math.pi:
        robot_theta -= 2 * math.pi
    while robot_theta < -math.pi:
        robot_theta += 2 * math.pi

    publish_odometry()
```

---

#### 6. **Command Handler**
```python
def on_mqtt_message(topic, msg):
    """Handle incoming commands from ROS2"""
    try:
        command = json.loads(msg.decode())
        cmd_type = command.get('type')

        if cmd_type == 'scan':
            # Trigger 360° scan
            asyncio.create_task(perform_scan())

        elif cmd_type == 'move':
            distance = command.get('distance', 0)
            move_robot(distance)

        elif cmd_type == 'rotate':
            angle = command.get('angle', 0)
            rotate_robot(angle)

        elif cmd_type == 'stop':
            alvik.stop()

    except Exception as e:
        print("Error handling command: {}".format(e))
```

**Command Examples**:
```json
// Move forward 20cm
{"type": "move", "distance": 20}

// Rotate left 90°
{"type": "rotate", "angle": 90}

// Trigger 360° scan
{"type": "scan"}

// Emergency stop
{"type": "stop"}
```

---

#### 7. **Odometry Publishing**
```python
def publish_odometry():
    """Publish current position to ROS2"""
    odom_data = {
        "x": round(robot_x, 3),      # meters
        "y": round(robot_y, 3),
        "theta": round(robot_theta, 3)  # radians
    }

    mqtt_client.publish(b"alvik/odom", json.dumps(odom_data).encode())
```

**Odometry Method**: Dead Reckoning
- Accumulates movement from motor commands
- Assumes perfect wheel motion (no slip)
- Errors accumulate over time
- SLAM corrects odometry drift

---

#### 8. **Main Event Loop**
```python
async def main_loop():
    """Asyncio main loop"""
    # Connect WiFi and MQTT
    connect_wifi()
    mqtt_client = connect_mqtt()

    # Start background tasks
    asyncio.create_task(mqtt_keepalive())       # MQTT ping
    asyncio.create_task(continuous_tof_publisher())  # Scanning
    asyncio.create_task(continuous_odom_publisher())  # Position

    # Keep event loop alive
    while True:
        await asyncio.sleep(1)

# Entry point
if __name__ == "__main__":
    asyncio.run(main_loop())
```

**Asyncio Tasks**:
1. **mqtt_keepalive**: Checks for incoming messages (100ms)
2. **continuous_tof_publisher**: Publishes scans (5 Hz)
3. **continuous_odom_publisher**: Publishes odometry (10 Hz)
4. **perform_scan**: Triggered on-demand (360° scan)

---

## MQTT Communication

### Message Flow

```
ALVIK ROBOT (Publisher)          MQTT BROKER          ROS2 (Subscriber)
     │                               │                        │
     ├─ alvik/scan ──────────────────┤                        │
     │  "37.5,48.4"                  ├───────────────────────►│ tof_receiver
     │                               │                        │
     ├─ alvik/odom ──────────────────┤                        │
     │  {"x":0.5,"y":0.3,"theta":0}  ├───────────────────────►│ odom_publisher
     │                               │                        │
     ◄─ alvik/command ───────────────┤                        │
       {"type":"move","distance":20} │◄───────────────────────┤ click_to_move
                                     │                        │ maze_explorer
                                     │                        │ teleop_keyboard
```

### MQTT Topics Reference

| Topic | Direction | Message Format | Purpose |
|-------|-----------|----------------|---------|
| `alvik/scan` | Robot → ROS2 | `"angle,distance"` | Individual scan points |
| `alvik/scan_complete` | Robot → ROS2 | `"1"` | Signal scan ready |
| `alvik/odom` | Robot → ROS2 | `{"x": 0.5, "y": 0.3, "theta": 1.57}` | Position update |
| `alvik/command` | ROS2 → Robot | `{"type": "move", "distance": 20}` | Navigation command |
| `alvik/explore` | User → Robot | `"start"` / `"stop"` | Exploration control |
| `alvik/status` | Robot → ROS2 | String | Status messages |
| `alvik/debug` | Robot → ROS2 | String | Debug information |

---

## Coordinate Frames (TF)

### Transform Tree

```
map
 └─ odom  (published by slam_toolbox)
     └─ base_link  (published by odom_publisher)
         ├─ top_surface  (published by robot_state_publisher)
         ├─ left_wheel
         ├─ right_wheel
         └─ alvik_laser  (ToF sensor frame)
```

### Frame Descriptions

| Frame | Parent | Description | Origin | Published By |
|-------|--------|-------------|--------|--------------|
| `map` | - | Global map frame | SLAM origin | slam_toolbox |
| `odom` | map | Odometry frame (drifts) | Robot start position | slam_toolbox |
| `base_link` | odom | Robot center | Robot geometric center | odom_publisher |
| `alvik_laser` | base_link | ToF sensor | 4.5cm forward of base | robot_state_publisher |

### Transform Significance

1. **map → odom**: SLAM correction
   - Compensates for odometry drift
   - Updated when loop closures detected
   - Jumps when map is corrected

2. **odom → base_link**: Dead reckoning
   - Smooth, continuous motion
   - Accumulates error over time
   - Updated at 10 Hz from robot

3. **base_link → alvik_laser**: Static transform
   - Defined in URDF
   - Accounts for sensor position on robot
   - Critical for scan-to-map alignment

### URDF Robot Model
**File**: `alvik.urdf`

```xml
<!-- Base Link - Robot body -->
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.096 0.095 0.025"/>  <!-- 9.6cm × 9.5cm × 2.5cm -->
    </geometry>
  </visual>
</link>

<!-- ToF Laser Sensor -->
<link name="alvik_laser">
  <visual>
    <geometry>
      <box size="0.015 0.020 0.010"/>  <!-- Small sensor module -->
    </geometry>
  </visual>
</link>

<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="alvik_laser"/>
  <!-- Sensor at front-center, 4.5cm forward, 3.7cm up -->
  <origin xyz="0.045 0.0 0.037" rpy="0 0 0"/>
</joint>
```

**Dimensions** (from Alvik specs):
- Robot body: 9.6cm × 9.5cm × 3.7cm
- Wheel diameter: 3.2cm
- Wheelbase: 8.2cm (between wheel centers)
- Sensor position: 4.5cm forward from center

---

## Setup Instructions

### Prerequisites
- Docker Desktop installed
- Arduino Alvik robot
- WiFi network (2.4GHz)
- Python 3.8+

### 1. Clone Repository
```bash
git clone <repository-url>
cd alvik_maze
```

### 2. Start Docker Container
```bash
docker-compose up -d
```

This starts:
- ROS2 Jazzy environment
- MQTT broker (Mosquitto)
- Foxglove Bridge

### 3. Build ROS2 Workspace
```bash
docker exec alvik_ros2 bash -c "
  cd /ros2_ws &&
  source /opt/ros/jazzy/setup.bash &&
  colcon build --symlink-install
"
```

### 4. Upload Firmware to Alvik
1. Edit `alvik_firmware/my_secrets.py`:
   ```python
   WIFI_SSID = "YourNetworkName"
   WIFI_PASSWORD = "YourPassword"
   MQTT_BROKER = "192.168.1.152"  # Docker container IP
   ```

2. Upload `main.py` to Alvik using Arduino IDE or Thonny

3. Robot will connect to WiFi and MQTT automatically

### 5. Verify MQTT Connection
```bash
# Subscribe to all Alvik topics
docker exec alvik_ros2 mosquitto_sub -h localhost -t 'alvik/#' -v
```

You should see:
- `alvik/scan` messages (5 Hz)
- `alvik/odom` messages (10 Hz)
- `alvik/status` on startup

---

## Usage Guide

### Starting the System

#### Terminal 1: Robot Description
```bash
docker exec -it alvik_ros2 bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch alvik_navigation robot_description.launch.py
```

#### Terminal 2: ToF Receiver
```bash
docker exec -it alvik_ros2 bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run alvik_navigation tof_receiver --ros-args -p mqtt_host:=localhost
```

#### Terminal 3: Odometry Publisher
```bash
docker exec -it alvik_ros2 bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run alvik_navigation odom_publisher --ros-args -p mqtt_host:=localhost
```

#### Terminal 4: SLAM Toolbox
```bash
docker exec -it alvik_ros2 bash
source /opt/ros/jazzy/setup.bash
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args --params-file /ros2_ws/src/alvik_navigation/config/mapper_params_online_async.yaml
```

Activate SLAM:
```bash
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

#### Terminal 5: Foxglove Bridge
```bash
docker exec -it alvik_ros2 bash
source /opt/ros/jazzy/setup.bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
```

### Foxglove Studio Setup
1. Open Foxglove Studio
2. Connect to: `ws://localhost:8765`
3. Add 3D panel
4. Subscribe to topics:
   - `/scan` (LaserScan) - red markers
   - `/map` (OccupancyGrid)
   - `/tf` (robot model)

---

### Manual Control

#### Keyboard Teleoperation
```bash
ros2 run alvik_navigation teleop_keyboard --ros-args -p mqtt_host:=localhost
```

Keys:
- `w` / `↑` : Forward 10cm
- `s` / `↓` : Backward 10cm
- `a` / `←` : Left 15°
- `d` / `→` : Right 15°
- `space` : Stop
- `q` : Quit

---

### Autonomous Exploration

#### Start Maze Explorer
```bash
ros2 run alvik_navigation maze_explorer --ros-args -p mqtt_host:=localhost
```

#### Start Exploration
```bash
# From another terminal
docker exec alvik_ros2 mosquitto_pub -h localhost -t 'alvik/explore' -m 'start'
```

#### Stop Exploration
```bash
docker exec alvik_ros2 mosquitto_pub -h localhost -t 'alvik/explore' -m 'stop'
```

**Robot behavior**:
- Follows right wall
- Turns right at intersections
- Triggers 360° scans at decision points
- Turns around at dead ends

---

### Triggering Manual Scans

```bash
# Trigger 360° scan
docker exec alvik_ros2 mosquitto_pub -h localhost -t 'alvik/command' \
  -m '{"type":"scan"}'
```

---

### Click-to-Move Navigation

1. Start click-to-move node:
   ```bash
   ros2 run alvik_navigation click_to_move --ros-args -p mqtt_host:=localhost
   ```

2. In Foxglove:
   - Switch to "Publish" mode (tool icon)
   - Click on map where you want robot to go
   - Robot will rotate and drive to clicked point

---

## Troubleshooting

### Common Issues

**1. Robot not connecting to MQTT**
```bash
# Check MQTT broker is running
docker exec alvik_ros2 mosquitto -c /etc/mosquitto/mosquitto.conf

# Check robot can reach broker IP
ping 192.168.1.152
```

**2. No scan data in ROS2**
```bash
# Check MQTT messages arriving
docker exec alvik_ros2 mosquitto_sub -h localhost -t 'alvik/scan' -C 10

# Check ROS2 topic
ros2 topic echo /scan --once
```

**3. SLAM not building map**
```bash
# Check SLAM is active
ros2 lifecycle get /slam_toolbox

# If "unconfigured", activate it:
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate

# Check scan rate
ros2 topic hz /scan  # Should be ~1-5 Hz
```

**4. TF transform errors**
```bash
# Check transform tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map base_link
```

---

## Performance Metrics

### Scan Characteristics
- **Continuous scans**: 25 points, 45° FOV, 5 Hz
- **360° scans**: ~360 points, full coverage, ~10 seconds
- **Range**: 2cm to 2m
- **Resolution**: 1.8° angular spacing

### SLAM Performance
- **Map resolution**: 1cm per pixel
- **Map update rate**: 1 Hz
- **Minimum travel for update**: 2cm or 6° rotation
- **Typical map size**: 4m × 4m (400×400 pixels)

### Network Latency
- **MQTT publish**: ~10-50ms
- **Scan end-to-end**: ~100-200ms (robot → MQTT → ROS2 → SLAM)
- **Command latency**: ~50-100ms (click → robot motion)

---

## Future Enhancements

### Potential Improvements
1. **Multi-zone sensor access** - Read all 7 ToF sensors for denser scans
2. **Adaptive wall following** - PID control for smoother navigation
3. **Path planning** - Use ROS2 Nav2 for optimal paths
4. **Map persistence** - Save/load maps for repeated use
5. **Loop closure** - Improve SLAM with visual landmarks
6. **Battery monitoring** - Track battery level via MQTT
7. **Web interface** - Control robot via web browser

---

## References

### Documentation
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Foxglove Studio](https://foxglove.dev/docs)
- [Arduino Alvik](https://docs.arduino.cc/hardware/alvik/)

### Key Concepts
- **SLAM**: Simultaneous Localization and Mapping
- **Dead Reckoning**: Position estimation from motion
- **TF Tree**: Coordinate frame relationships
- **MQTT**: Lightweight IoT messaging protocol
- **LaserScan**: Standard ROS2 range sensor format

---

## License
MIT License

## Author
Tutorial by [Your Name]
GitHub: [Your GitHub]
YouTube: [Your Channel]

---

**Last Updated**: 2025-11-15
**ROS2 Version**: Jazzy Jalisco
**Tested On**: Arduino Alvik, Docker Desktop (macOS/Linux)
