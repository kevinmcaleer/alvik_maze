#!/bin/bash
set -e

echo "========================================="
echo "Building Alvik ROS2 for Apple Silicon"
echo "========================================="
echo "Using optimized ARM64 Dockerfile..."
echo ""

# Check architecture
ARCH=$(uname -m)
if [[ "$ARCH" != "arm64" && "$ARCH" != "aarch64" ]]; then
    echo "⚠️  This build script is optimized for Apple Silicon (ARM64)"
    echo "   Detected: $ARCH"
    echo "   Continuing anyway, but you may encounter issues..."
    echo ""
fi

# Check if XQuartz is running
if ! pgrep -x "XQuartz" > /dev/null; then
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

# Build using the ARM64-optimized Dockerfile
echo ""
echo "Building Docker image with ARM64 Dockerfile..."
echo "This will take about 10-15 minutes..."
echo ""

export DOCKER_DEFAULT_PLATFORM=linux/arm64

docker build \
    --platform linux/arm64 \
    -f Dockerfile.arm64 \
    -t alvik-ros2-docker_ros2_alvik:latest \
    --progress=plain \
    .

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo "Build complete!"
    echo "========================================="
    echo ""
    echo "Next steps:"
    echo "1. Configure Alvik firmware (alvik_firmware/main.py)"
    echo "2. Start container: ./scripts/run.sh"
    echo ""
else
    echo ""
    echo "========================================="
    echo "Build failed!"
    echo "========================================="
    echo ""
    echo "Troubleshooting:"
    echo "1. Ensure Docker Desktop has at least 4GB RAM allocated"
    echo "2. Try: docker system prune -a"
    echo "3. Check Docker Desktop logs"
    echo ""
    exit 1
fi
