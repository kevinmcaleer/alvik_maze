#!/bin/bash
# Start Foxglove Bridge for visualizing ROS2 topics

echo "Starting Foxglove Bridge..."
docker exec -d alvik_ros2 bash -c "source /opt/ros/jazzy/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765"

sleep 2

echo "Foxglove Bridge started!"
echo ""
echo "Connect Foxglove Studio to: ws://localhost:8765"
echo ""
echo "To open Foxglove Studio:"
echo "  open -a Foxglove"
