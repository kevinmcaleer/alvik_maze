# ROS2 Jazzy with GUI support for macOS
FROM osrf/ros:jazzy-desktop-full

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    mosquitto \
    mosquitto-clients \
    vim \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir \
    paho-mqtt \
    numpy \
    scipy \
    transforms3d \
    matplotlib

# Set up ROS2 workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Configure Cyclone DDS for better performance
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2 in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Expose ports for rviz2 and MQTT
EXPOSE 1883 11311

# Set up entrypoint
COPY scripts/start.sh /start.sh
RUN chmod +x /start.sh

ENTRYPOINT ["/start.sh"]
CMD ["bash"]
