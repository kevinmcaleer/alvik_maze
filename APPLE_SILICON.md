# Apple Silicon (M1/M2/M3) Troubleshooting Guide

## Build Issues on Apple Silicon

You're getting platform mismatch errors because Docker is trying to use AMD64 images on your ARM64 Mac.

### Solution 1: Use the ARM64-Optimized Build (RECOMMENDED)

```bash
cd alvik-ros2-docker
./scripts/build-arm64.sh
```

This uses `Dockerfile.arm64` which:
- Explicitly targets ARM64 platform
- Uses ROS2 Humble (better ARM64 support)
- Builds from Ubuntu base for reliability
- Installs only necessary components

**Advantages:**
- Faster build times
- More reliable on Apple Silicon
- Smaller image size
- Better compatibility

### Solution 2: Fix the Original Dockerfile

The original Dockerfile has been updated to support ARM64. Try:

```bash
cd alvik-ros2-docker

# Clean everything
docker system prune -a

# Rebuild
./scripts/build.sh
```

## Common Apple Silicon Issues

### Issue: "platform linux/amd64 does not match"

**Fix:**
```bash
# Set default platform
export DOCKER_DEFAULT_PLATFORM=linux/arm64

# Or use the ARM64 build script
./scripts/build-arm64.sh
```

### Issue: Build fails with "exit code: 1" during pip install

**Fix:** The ARM64 Dockerfile installs packages individually to avoid this.

```bash
./scripts/build-arm64.sh
```

### Issue: "No matching manifest for linux/arm64"

This means the Docker image doesn't have an ARM64 version available.

**Fix:** Use the ARM64 Dockerfile which uses base images with ARM64 support:
```bash
./scripts/build-arm64.sh
```

### Issue: Very slow build times

**Fix:** Allocate more resources to Docker Desktop:
1. Open Docker Desktop
2. Settings → Resources
3. Set CPUs: 4-6 cores
4. Set Memory: 6-8 GB
5. Click "Apply & Restart"

### Issue: rviz2 fails with OpenGL/GLX errors

If you see errors like:
```
Failed to create an OpenGL context - BadValue
Unable to create a suitable GLXContext
Unable to create the rendering window after 100 tries
```

**Fix:** The docker-compose.yml has been configured to use software rendering (llvmpipe) instead of hardware OpenGL, which isn't available in Docker on macOS.

After updating, rebuild:
```bash
docker compose down
docker compose build
docker compose up -d
docker compose exec ros2_alvik bash
```

### Issue: rviz2 is slow or laggy

Apple Silicon Macs run Docker through virtualization which can impact GUI performance.

**Fixes:**
1. Use software rendering (already configured in docker-compose.yml)
   - This trades performance for compatibility
   - RViz2 will work but may be slower than native Linux

2. Enable VirtioFS (faster file sharing):
   - Docker Desktop → Settings → General
   - Enable "VirtioFS accelerated directory sharing"

3. Enable Rosetta 2 (if available):
   - Docker Desktop → Settings → Features in development
   - Enable "Use Rosetta for x86/amd64 emulation on Apple Silicon"

4. Reduce rviz2 settings:
   - Lower point cloud density
   - Reduce update rates
   - Disable unnecessary visualizations

## Docker Desktop Configuration for Apple Silicon

### Recommended Settings:

**Resources:**
- CPUs: 4-6 cores
- Memory: 6-8 GB
- Swap: 2 GB
- Disk: 60 GB

**General:**
- ✓ Use VirtioFS accelerated directory sharing
- ✓ Use Rosetta for x86/amd64 emulation

**Advanced:**
- ✓ Allow the default Docker socket to be used

## Verification Steps

After building, verify everything works:

```bash
# Check platform
docker inspect alvik-ros2-docker_ros2_alvik:latest | grep Architecture

# Should show: "Architecture": "arm64"

# Test container
docker run --rm alvik-ros2-docker_ros2_alvik:latest uname -m

# Should show: aarch64
```

## Still Having Issues?

### Try a complete reset:

```bash
# Stop all containers
docker stop $(docker ps -aq)

# Remove all containers
docker rm $(docker ps -aq)

# Remove all images
docker rmi $(docker images -q)

# Prune system
docker system prune -a --volumes

# Restart Docker Desktop

# Rebuild with ARM64
cd alvik-ros2-docker
./scripts/build-arm64.sh
```

## Performance Tips for Apple Silicon

1. **Use ARM64 native images** - Much faster than emulated AMD64
2. **Allocate sufficient resources** - Don't starve Docker of CPU/RAM
3. **Use VirtioFS** - Dramatically improves file I/O
4. **Close unnecessary apps** - Free up system resources
5. **Monitor Activity Monitor** - Ensure Docker isn't thermal throttling

## Known Limitations

- Some ROS2 packages may not have ARM64 versions yet
- GPU acceleration for rviz2 is limited in Docker on macOS
- Network performance may be slightly slower than native Linux

## Alternative: Native ROS2 Installation

If Docker continues to have issues, you can install ROS2 natively on macOS:

```bash
brew install ros

# But this is more complex and less isolated
```

The Docker approach is still recommended for easier setup and portability.

## Getting Help

If you're still stuck:

1. Check Docker Desktop logs: 
   - Docker Desktop → Troubleshoot → View logs

2. Check container logs:
   ```bash
   docker logs alvik_ros2
   ```

3. Test MQTT separately:
   ```bash
   brew install mosquitto
   mosquitto -v
   ```

4. Post on kevsrobots.com forums with:
   - Mac model (M1/M2/M3)
   - macOS version
   - Docker Desktop version
   - Full error message
