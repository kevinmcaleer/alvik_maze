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

# Start mosquitto in background
mosquitto -c /mosquitto/config/mosquitto.conf &

echo "========================================="
echo "Alvik ROS2 Environment Ready!"
echo "========================================="
echo ""
echo "Available commands:"
echo "  ros2 launch alvik_mapping alvik_system.launch.py  - Start full system"
echo "  rviz2 -d /ros2_ws/src/alvik_mapping/config/rviz_config.rviz  - Start rviz2"
echo "  ros2 topic list  - List all topics"
echo ""
echo "Alvik should connect to MQTT broker at: $(hostname -I | awk '{print $1}')"
echo "========================================="

# Execute whatever command was passed
exec "$@"
