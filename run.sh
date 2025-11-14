#!/bin/bash
set -e

# Enable X11 forwarding
xhost + localhost > /dev/null 2>&1 || true

echo "Starting Alvik ROS2 Docker container..."
echo ""

# Get Mac IP for Alvik to connect to
MAC_IP=$(ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null || echo "localhost")
echo "Your Mac IP: $MAC_IP"
echo "Configure Alvik to connect to this IP"
echo ""

# Start containers
docker-compose up -d

# Attach to main container
docker exec -it alvik_ros2 bash
