#!/bin/bash

# Installation script for Iravath Perception package
# Run this script after setting up ROS2 Jazzy

set -e

echo "🚀 Installing Iravath Perception Package Dependencies"
echo "======================================================"

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not sourced. Please source ROS2 Jazzy first:"
    echo "   source /opt/ros/jazzy/setup.bash"
    exit 1
fi

echo "✅ ROS2 $ROS_DISTRO detected"

# Install Python dependencies
echo "📦 Installing Python dependencies..."
pip3 install -r requirements.txt

# Install ROS2 dependencies
echo "📦 Installing ROS2 dependencies..."
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-vision-msgs \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    python3-opencv \
    python3-numpy

# Install Intel RealSense SDK (if not already installed)
echo "📦 Installing Intel RealSense SDK..."
if ! dpkg -l | grep -q librealsense2; then
    echo "Installing RealSense SDK..."
    sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    sudo apt-key adv --keyserver hkp://ipv4.pool.sks-keyservers.net:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

    sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
    sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
fi

# Build the package
echo "🔨 Building ROS2 package..."
cd $(dirname "$0")
colcon build --packages-select iravath_perception

echo "✅ Installation complete!"
echo ""
echo "📋 Next steps:"
echo "1. Source the workspace: source install/setup.bash"
echo "2. Test camera: ros2 launch iravath_perception camera.launch.py"
echo "3. Run full perception: ros2 launch iravath_perception perception.launch.py"
echo ""
echo "📖 For more information, see README.md"