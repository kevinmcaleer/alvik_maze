#!/bin/bash
# Trigger 360° scan on Alvik

MQTT_BROKER="192.168.1.152"

echo "Triggering 360° scan on Alvik..."
docker exec alvik_ros2 mosquitto_pub -h $MQTT_BROKER -t "alvik/command" -m '{"type":"scan"}'
echo "Scan command sent!"
echo ""
echo "The Alvik will now:"
echo "  - Rotate 360° (taking 72 readings at 5° intervals)"
echo "  - This takes about 15-20 seconds"
echo "  - Watch in Foxglove Studio to see the scan data"
echo ""
echo "Monitor scan data:"
echo "  docker exec alvik_ros2 bash -c \"source /opt/ros/jazzy/setup.bash && ros2 topic echo /scan\""
