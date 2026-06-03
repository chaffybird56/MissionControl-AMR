#!/bin/bash
set -e

# Set environment variables
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Source ROS2
source /opt/ros/$ROS_DISTRO/setup.bash

# Source workspace if it exists
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# Function to handle shutdown gracefully
cleanup() {
    echo "Shutting down ROS2 nodes..."
    # Kill all background processes
    jobs -p | xargs -r kill
    exit 0
}

# Set up signal handlers
trap cleanup SIGTERM SIGINT

# Wait for Webots to be ready (if using simulation)
if [ "${USE_WEBOTS:-true}" = "true" ]; then
    echo "Waiting for Webots to be ready..."
    sleep 5
fi

# Start the main command
echo "Starting ROS2 AMR system..."
exec "$@"

