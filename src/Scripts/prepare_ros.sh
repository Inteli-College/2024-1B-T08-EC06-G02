#!/bin/bash

# Update package lists and upgrade installed packages
echo
echo "Update packages"
sudo apt update
sudo apt upgrade -y

# Installing dependencies
apt-get install -y \
    sudo \
    locales \
    curl \
    wget \
    nano \
    vim \
    git \
    bzip2 \
    libspdlog1 \
    tzdata \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Updating packages
echo "Updating Packages"
sudo apt update && sudo apt upgrade -y

# Install necessary packages
sudo apt install -y software-properties-common curl

# Add Ubuntu Universe repository if not already added
echo
if ! apt-cache policy | grep -q universe; then
    sudo add-apt-repository universe
fi

# Add ROS 2 GPG key and repository
echo
echo "Add ROS GPG"
sudo apt update
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

Download ROS 2
echo
echo "Downloading ROS2"
wget -P ~/Downloads/ https://github.com/ros2/ros2/releases/download/release-humble-20240222/ros2-humble-20240222-linux-jammy-amd64.tar.bz2


mkdir -p ~/ros2_humble
cd ~/ros2_humble

echo
echo "Extracting the file, actual dir $(pwd)"
echo "$(ls) -- $(pwd)"
sudo apt update && sudo apt upgrade
sudo apt update
tar xvf ~/Downloads/ros2-humble-20240222-linux-jammy-amd64.tar.bz2 


# Install Rosdep
echo
echo "Installing rosdep"
sudo apt update
sudo apt install -y python3-rosdep
sudo rosdep init -y
rosdep update -y && sudo apt upgrade -y

# Installing the missing dependencies
echo 
echo "Installing missing depencies"
rosdep install --from-paths ~/ros2_humble/ros2-linux/share --ignore-src -y --skip-keys "cyclonedds fastcdr fastrtps rti-connext-dds-6.0.1 urdfdom_headers"

# Install development tools
sudo apt install -y ros-dev-tools

# Updating
sudo apt update && sudo apt ugrade

# Seting the ros on the terminal
source ~/ros2_humble/ros2-linux/setup.bash

# Setting ROS_PYTHON_PATH
sudo apt update && sudo apt upgrade
sudo ldconfig
export LD_LIBRARY_PATH=/path/to/directory:$LD_LIBRARY_PATH

# Environment setup
echo "source ~/ros2_humble/ros2-linux/setup.bash" >> ~/.bashrc

# Setup ROS environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Checking for turtlebot3
if dpkg -l | grep ros-humble-turtlebot3; then
    echo "Installing turtlebot3 ..."
else
    echo "Installing turtlebot3"
    sudo apt install ros-humble-turtlebot3
fi
#====

# Log completion message
echo "Installation completed successfully"
source run_workspace.sh
