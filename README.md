# Alvik ROS2 Mapping & Navigation System

Complete Docker-based ROS2 system for Arduino Alvik robot mapping and click-to-navigate control via rviz2.

## Features

- 🗺️ **Real-time room mapping** using Alvik's ToF sensor
- 🖱️ **Click-to-navigate** - Click anywhere on the map in rviz2 to send the robot there
- 📡 **Wireless MQTT communication** between Mac and Alvik
- 🐳 **Fully Dockerized** - No local ROS2 installation needed
- 📊 **Live visualization** in rviz2

## Prerequisites

### On Your Mac:

1. **Docker Desktop**
   ```bash
   # Install via Homebrew
   brew install --cask docker
   ```

2. **XQuartz** (X11 server for GUI support)
   ```bash
   brew install --cask xquartz
   ```
   
   After installing:
   - Start XQuartz
   - Go to Preferences → Security
   - Enable "Allow connections from network clients"
   - Restart XQuartz
   - Run: `xhost + localhost`

### On Your Alvik:

1. MicroPython MQTT library (`umqtt`)
2. WiFi credentials configured

## Quick Start

### 1. Build the Docker Environment

```bash
cd alvik-ros2-docker
chmod +x scripts/*.sh
./scripts/build.sh
```

This will:
- Create necessary directories
- Configure MQTT broker
- Build the Docker image (~5-10 minutes first time)

### 2. Configure Alvik

Edit `alvik_firmware/main.py` and update:

```python
WIFI_SSID = "YourWiFiNetwork"
WIFI_PASSWORD = "YourPassword"
MQTT_BROKER = "192.168.1.XXX"  # Your Mac's IP (shown when you run the container)
```

Upload `main.py` to your Alvik using Arduino Lab or Thonny.

### 3. Start the System

**Terminal 1 - Start Docker container:**
```bash
./scripts/run.sh
```

Note the IP address shown - you'll need this for the Alvik.

**Terminal 2 - Inside the container:**
```bash
# Build ROS2 workspace (first time only)
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash

# Launch the full system
ros2 launch alvik_mapping alvik_system.launch.py
```

This starts:
- ToF receiver node
- Room mapper node
- Alvik bridge node (odometry)
- Goal commander node
- rviz2 visualization

### 4. Power On Alvik

Run the `main.py` script on your Alvik. It should:
1. Connect to WiFi
2. Connect to MQTT broker
3. Print "Alvik ready! Waiting for commands..."

## Usage

### Scanning a Room

In rviz2, you should see:
- Red laser scan points
- Occupancy grid map building up
- Robot's position (TF frames)

To trigger a 360° scan manually:
```bash
# In another terminal, inside the Docker container
mosquitto_pub -h localhost -t "alvik/command" -m '{"type":"scan"}'
```

The robot will:
1. Rotate 360° in place
2. Take ToF readings every 5°
3. Send data to ROS2
4. Display map in rviz2

### Click-to-Navigate

1. In rviz2, click the **"2D Goal Pose"** button in the toolbar
2. Click anywhere on the map
3. Drag to set the desired robot orientation
4. Release - the robot will navigate to that position!

The robot will:
- Rotate to face the goal
- Drive forward
- Align to final orientation
- Stop when arrived

### Manual Control

Send commands via MQTT:

```bash
# Move forward 50cm
mosquitto_pub -h localhost -t "alvik/command" -m '{"type":"move", "distance":50}'

# Rotate 90° left
mosquitto_pub -h localhost -t "alvik/command" -m '{"type":"rotate", "angle":-90}'

# Emergency stop
mosquitto_pub -h localhost -t "alvik/command" -m '{"type":"stop"}'
```

## Project Structure

```
alvik-ros2-docker/
├── Dockerfile                  # ROS2 container definition
├── docker-compose.yml          # Multi-container orchestration
├── scripts/
│   ├── build.sh               # Build script
│   └── run.sh                 # Run script
├── ros2_ws/src/alvik_mapping/
│   ├── alvik_mapping/
│   │   ├── tof_receiver.py    # Receives ToF scans via MQTT
│   │   ├── room_mapper.py     # Builds occupancy grid
│   │   ├── alvik_bridge.py    # Publishes odometry
│   │   └── goal_commander.py  # Click-to-navigate
│   ├── launch/
│   │   └── alvik_system.launch.py
│   └── config/
│       └── rviz_config.rviz
└── alvik_firmware/
    └── main.py                 # Upload this to Alvik
```

## Troubleshooting

### rviz2 Won't Start
- Ensure XQuartz is running
- Run `xhost + localhost`
- Check `echo $DISPLAY` shows `host.docker.internal:0`

### Alvik Won't Connect to MQTT
- Verify Mac and Alvik are on same WiFi network
- Check firewall isn't blocking port 1883
- Confirm MQTT_BROKER IP in `main.py` matches Mac's IP
- Test with: `mosquitto_sub -h localhost -t "alvik/#" -v`

### No Laser Scan Visible
- Check ToF receiver is running: `ros2 node list`
- Verify MQTT messages: `mosquitto_sub -t "alvik/scan" -v`
- Check topic: `ros2 topic echo /scan`

### Robot Doesn't Move When Clicking
- Ensure goal_commander node is running
- Check odometry is being published: `ros2 topic echo /odom`
- Verify MQTT broker is receiving commands

### Map Not Building
- Ensure room_mapper node is running
- Check it's receiving scans: `ros2 topic hz /scan`
- Verify map is being published: `ros2 topic echo /map`

## Configuration

### Changing Map Size/Resolution

Edit `ros2_ws/src/alvik_mapping/launch/alvik_system.launch.py`:

```python
parameters=[{
    'map_size_meters': 5.0,      # 5m x 5m map
    'map_resolution': 0.02        # 2cm per cell
}]
```

### Adjusting Navigation Speed

Edit `goal_commander.py`:

```python
def send_move_command(self, distance):
    twist.linear.x = 0.1  # Increase for faster movement
```

## Advanced Usage

### Recording a Map

```bash
# Save map to file
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: /map, map_url: /ros2_ws/my_maze_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```

### Multiple Rooms

To map multiple rooms, trigger scans at different positions and the mapper will combine them.

### Adding Camera Support

Future enhancement: Add camera feed visualization alongside ToF mapping.

## Tips for Best Results

1. **Lighting**: Ensure consistent lighting for color sensor accuracy
2. **Surface**: Place Alvik on flat, level surface
3. **Obstacles**: ToF works best on solid walls, may struggle with:
   - Transparent surfaces (glass)
   - Very dark surfaces (absorb IR)
   - Thin poles
4. **Calibration**: Robot position drifts over time (odometry error) - perform new scans periodically
5. **WiFi**: Strong signal needed for reliable MQTT communication

## YouTube Video Ideas

1. **Setup Tutorial**: Step-by-step guide to get this running
2. **Live Mapping Demo**: Show robot scanning maze rooms
3. **Click-to-Navigate**: Interactive control demonstration
4. **Troubleshooting Tips**: Common issues and solutions
5. **Custom Maze Challenge**: Design maze and navigate autonomously

## License

MIT License - Feel free to adapt for kevsrobots.com tutorials!

## Credits

Created for Arduino Alvik maze-solving project
By Kev @ kevsrobots.com
