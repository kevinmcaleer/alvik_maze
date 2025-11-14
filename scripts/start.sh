#!/bin/bash
set -e

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build workspace if not already built
if [ ! -f "/ros2_ws/install/setup.bash" ]; then
    echo "Building ROS2 workspace..."
    cd /ros2_ws
    colcon build --symlink-install
fi

# Source workspace
source /ros2_ws/install/setup.bash

# Start mosquitto in background (if config exists)
if [ -f "/mosquitto/config/mosquitto.conf" ]; then
    mosquitto -c /mosquitto/config/mosquitto.conf &
    echo "Started MQTT broker"
fi

# Start Foxglove Bridge in background
echo "Starting Foxglove Bridge..."
nohup ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 > /tmp/foxglove.log 2>&1 &
FOXGLOVE_PID=$!
sleep 2

# Check if Foxglove Bridge started successfully
if ps -p $FOXGLOVE_PID > /dev/null; then
    echo "✓ Foxglove Bridge started (PID: $FOXGLOVE_PID)"
else
    echo "⚠ Foxglove Bridge may have failed to start. Check /tmp/foxglove.log"
fi

echo ""
echo "========================================="
echo "Alvik ROS2 Environment Ready!"
echo "========================================="
echo ""
echo "Services running:"
echo "  ✓ Foxglove Bridge: ws://localhost:8765"
echo "  ✓ MQTT Broker: localhost:1883"
echo ""
echo "Available commands:"
echo "  ros2 launch alvik_mapping alvik_system.launch.py  - Start full system"
echo "  ros2 topic list  - List all topics"
echo "  ros2 topic echo /topic_name  - View topic data"
echo ""
echo "Foxglove Studio:"
echo "  1. Open Foxglove Studio: open -a Foxglove"
echo "  2. Connect to: ws://localhost:8765"
echo ""
echo "Logs:"
echo "  Foxglove: tail -f /tmp/foxglove.log"
echo ""
echo "Alvik should connect to MQTT broker at: $(hostname -I | awk '{print $1}')"
echo "========================================="

# Execute whatever command was passed
exec "$@"
