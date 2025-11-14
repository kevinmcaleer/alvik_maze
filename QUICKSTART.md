# Alvik ROS2 Quick Start Guide

## Setup (One Time)

### 1. Install Prerequisites on Mac
```bash
brew install --cask docker xquartz
```

Start XQuartz:
- XQuartz → Preferences → Security → Enable "Allow connections from network clients"
- Restart XQuartz
- Run: `xhost + localhost`

### 2. Extract and Build
```bash
tar -xzf alvik-ros2-docker.tar.gz
cd alvik-ros2-docker
./scripts/build.sh
```

### 3. Configure Alvik Firmware
Edit `alvik_firmware/main.py`:
- Set your WiFi SSID and password
- Set MQTT_BROKER to your Mac's IP (shown when container starts)

Upload `main.py` to Alvik.

## Running the System

### Every Time You Want to Use It:

**Step 1:** Start XQuartz
```bash
open -a XQuartz
xhost + localhost
```

**Step 2:** Start Docker Container
```bash
cd alvik-ros2-docker
./scripts/run.sh
```
Note the Mac IP address shown!

**Step 3:** Inside Container, Launch ROS2
```bash
cd /ros2_ws
colcon build --symlink-install  # First time only
source install/setup.bash
ros2 launch alvik_mapping alvik_system.launch.py
```

**Step 4:** Power On Alvik
Run the main.py script on your Alvik.

**Step 5:** Use rviz2
- rviz2 window should open
- Click "2D Goal Pose" button
- Click on map where you want robot to go
- Robot moves there automatically!

## Common Commands

### Trigger Manual Scan
```bash
mosquitto_pub -h localhost -t "alvik/command" -m '{"type":"scan"}'
```

### Move Robot
```bash
# Forward 30cm
mosquitto_pub -h localhost -t "alvik/command" -m '{"type":"move", "distance":30}'

# Backward 20cm
mosquitto_pub -h localhost -t "alvik/command" -m '{"type":"move", "distance":-20}'
```

### Rotate Robot
```bash
# 90° left
mosquitto_pub -h localhost -t "alvik/command" -m '{"type":"rotate", "angle":-90}'

# 90° right
mosquitto_pub -h localhost -t "alvik/command" -m '{"type":"rotate", "angle":90}'
```

### Stop Robot
```bash
mosquitto_pub -h localhost -t "alvik/command" -m '{"type":"stop"}'
```

## Monitoring

### View All Topics
```bash
ros2 topic list
```

### Watch Laser Scan
```bash
ros2 topic echo /scan
```

### Watch Robot Position
```bash
ros2 topic echo /odom
```

### Monitor MQTT
```bash
mosquitto_sub -h localhost -t "alvik/#" -v
```

## Troubleshooting

**rviz2 doesn't open:**
- Is XQuartz running? `ps aux | grep XQuartz`
- Did you run `xhost + localhost`?

**Alvik won't connect:**
- Are Mac and Alvik on same WiFi?
- Is the IP address in main.py correct?
- Test: `ping <alvik-ip>`

**No map appearing:**
- Is ToF receiver getting data? `ros2 topic hz /scan`
- Check MQTT: `mosquitto_sub -t "alvik/scan"`

**Robot doesn't move when clicking:**
- Is Alvik connected? Check `alvik/status` topic
- Is goal_commander running? `ros2 node list`

## File Locations

- **ROS2 nodes:** `ros2_ws/src/alvik_mapping/alvik_mapping/`
- **Launch file:** `ros2_ws/src/alvik_mapping/launch/`
- **rviz config:** `ros2_ws/src/alvik_mapping/config/`
- **Alvik firmware:** `alvik_firmware/main.py`

## Next Steps

1. Build your foamcore maze (25cm x 25cm cells)
2. Place Alvik in center
3. Trigger 360° scan
4. Watch map build in rviz2
5. Click to navigate through maze!

---

For full documentation, see README.md
For video tutorials, visit kevsrobots.com
