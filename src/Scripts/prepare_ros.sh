#bin/bash

# Update package lists and upgrade installed packages
execute_and_log sudo apt update
execute_and_log sudo apt upgrade -y

# Install necessary packages
execute_and_log sudo apt install -y software-properties-common curl

# Add Ubuntu Universe repository if not already added
if ! apt-cache policy | grep -q universe; then
    execute_and_log sudo add-apt-repository universe
fi

# Add ROS 2 GPG key and repository
execute_and_log sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists after adding new repositories
execute_and_log sudo apt update

# Install ROS Humble Desktop
execute_and_log sudo apt install -y ros-humble-desktop

# Setup ROS environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Log completion message
log "Installation completed successfully"
