#!/bin/bash

# Update package lists and upgrade installed packages
sudo apt update
sudo apt upgrade -y

# Install necessary packages
sudo apt install -y software-properties-common curl

# Add Ubuntu Universe repository if not already added
if ! apt-cache policy | grep -q universe; then
    sudo add-apt-repository universe
fi

# Add ROS 2 GPG key and repository
sudo apt update
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Download ROS 2
echo "Downloading ROS2"
wget -P ~/Downloads/ https://github.com/ros2/ros2/releases/download/release-humble-20240222/ros2-humble-20240222-linux-jammy-amd64.tar.bz2
mkdir -p ~/ros2_humble
cd ~/ros2_humble
sleep 5
echo "Extracting the file, actual dir $(pwd)"
tar -p xf ~/Downloads/ros2-humble-20240222-linux-jammy-amd64.tar.bz2


# Install Rosdep
sudo apt update
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update -y

# Installing the missing dependencies
rosdep install --from-paths ~/ros2_humble/ros2-linux/share --ignore-src -y --rosdistro humble --skip-keys "cyclonedds fastcdr fastrtps rti-connext-dds-6.0.1 urdfdom_headers"

# Install development tools
sudo apt install -y ros-dev-tools

# Environment setup
echo "source ~/ros2_humble/ros2-linux/setup.bash" >> ~/.bashrc

# Setup ROS environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Log completion message
echo "Installation completed successfully"
