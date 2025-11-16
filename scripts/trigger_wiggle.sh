#!/bin/bash
# Trigger wiggle movement on Alvik to help SLAM build map confidence

MQTT_BROKER="192.168.1.152"

echo "Triggering wiggle movement on Alvik..."
docker exec alvik_ros2 mosquitto_pub -h $MQTT_BROKER -t "alvik/command" -m '{"type":"wiggle"}'
echo "Wiggle command sent!"
echo ""
echo "The Alvik will now:"
echo "  - Fast oscillation ±15° using direct motor control"
echo "  - 8 cycles with encoder feedback for accuracy"
echo "  - Motor speed: 40, should take about 8-10 seconds"
echo "  - Helps SLAM build confidence in wall positions through movement"
echo "  - Watch in Foxglove Studio to see the scan data from different perspectives"
echo ""
echo "Monitor status:"
echo "  docker exec alvik_ros2 mosquitto_sub -h $MQTT_BROKER -t 'alvik/status' -v"
