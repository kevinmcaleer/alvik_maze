#!/bin/bash
# Monitor Alvik MQTT activity

MQTT_BROKER="192.168.1.152"

echo "========================================="
echo "Monitoring Alvik MQTT Activity"
echo "========================================="
echo ""
echo "Listening to all alvik/* topics on $MQTT_BROKER"
echo "Press Ctrl+C to stop"
echo ""

# Subscribe to all Alvik topics and show verbose output
docker exec alvik_ros2 mosquitto_sub -h $MQTT_BROKER -t "alvik/#" -v
