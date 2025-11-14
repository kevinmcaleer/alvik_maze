#!/bin/bash
set -e

echo "========================================="
echo "Building Alvik ROS2 Docker Environment"
echo "========================================="

# Check architecture
ARCH=$(uname -m)
echo "Detected architecture: $ARCH"

if [[ "$ARCH" == "arm64" || "$ARCH" == "aarch64" ]]; then
    echo "✓ Apple Silicon (ARM64) detected - using compatible image"
elif [[ "$ARCH" == "x86_64" ]]; then
    echo "⚠️  Intel Mac detected - you may need to enable Rosetta 2 in Docker settings"
    echo "   Go to Docker Desktop → Settings → General → Use Rosetta for x86/amd64 emulation"
else
    echo "⚠️  Unknown architecture: $ARCH"
fi

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
echo ""
echo "Building Docker image (this may take 10-15 minutes)..."
echo "Platform: linux/arm64"
echo ""

export DOCKER_DEFAULT_PLATFORM=linux/arm64
docker-compose build --progress=plain

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo "Build complete!"
    echo "========================================="
    echo ""
    echo "To start the container:"
    echo "  ./scripts/run.sh"
    echo ""
else
    echo ""
    echo "========================================="
    echo "Build failed!"
    echo "========================================="
    echo ""
    echo "Common fixes:"
    echo "1. Ensure Docker Desktop is running"
    echo "2. Try: docker system prune -a (clears cache)"
    echo "3. Check Docker has enough memory (Preferences → Resources)"
    echo "4. For Intel Macs: Enable Rosetta in Docker settings"
    echo ""
    exit 1
fi
