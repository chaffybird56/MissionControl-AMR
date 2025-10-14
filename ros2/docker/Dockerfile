FROM ros:humble-ros-core

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-opencv \
    python3-numpy \
    libopencv-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-nav2-bringup \
    ros-humble-nav2-common \
    ros-humble-nav2-core \
    ros-humble-nav2-msgs \
    ros-humble-nav2-util \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-recoveries \
    ros-humble-nav2-behavior-tree \
    ros-humble-nav2-smoother \
    ros-humble-nav2-velocity-smoother \
    ros-humble-nav2-waypoint-follower \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-action-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rosbag2-transport \
    ros-humble-rosbag2-compression \
    ros-humble-rosbag2-cpp \
    ros-humble-rosbag2-py \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    opencv-python \
    numpy \
    websockets \
    prometheus-client

# Create workspace directory
WORKDIR /ros2_ws

# Copy package sources
COPY packages/ src/

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Install dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN colcon build --symlink-install

# Source the workspace
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Copy entrypoint script
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Create directories for logs and bags
RUN mkdir -p /bags /logs

# Expose ports
EXPOSE 8765 8080 9090

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "amr_bringup", "amr_bringup.launch.py"]
