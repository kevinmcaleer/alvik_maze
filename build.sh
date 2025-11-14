#!/bin/bash
set -e

echo "========================================="
echo "Building Alvik ROS2 Docker Environment"
echo "========================================="

# Check if XQuartz is running (required for GUI on macOS)
if ! pgrep -x "XQuartz" > /dev/null; then
    echo ""
    echo "⚠️  XQuartz is not running!"
    echo "Please install and start XQuartz:"
    echo "  1. Install: brew install --cask xquartz"
    echo "  2. Start XQuartz and enable 'Allow connections from network clients'"
    echo "  3. Run: xhost + localhost"
    echo ""
    read -p "Press enter when XQuartz is ready..."
fi

# Create necessary directories
echo "Creating directory structure..."
mkdir -p mosquitto/config
mkdir -p mosquitto/data
mkdir -p mosquitto/log
mkdir -p ros2_ws/src

# Create mosquitto config
cat > mosquitto/config/mosquitto.conf << EOF
listener 1883
allow_anonymous true
persistence true
persistence_location /mosquitto/data/
log_dest file /mosquitto/log/mosquitto.log
EOF

# Build Docker image
echo "Building Docker image..."
docker-compose build

echo ""
echo "========================================="
echo "Build complete!"
echo "========================================="
echo ""
echo "To start the container:"
echo "  ./scripts/run.sh"
echo ""
