#!/bin/bash

# Gets current dir
current_dir=$(pwd)

# Creates the virtual enviroment
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Check for UTF-8 
echo "Setting locales"
if ! locale -a | grep -q "UTF-8"; then
    echo "UTF-8 locale is not available. Installing..."
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    echo "UTF-8 locale installed and set."
else
    echo "UTF-8 locale is already available."
fi

# Seting time zone
apt-get update && apt-get install -y locales \
    && sed -i -e 's/# pt_BR.UTF-8 UTF-8/pt_BR.UTF-8 UTF-8/' /etc/locale.gen \
    && locale-gen

# Set the time zone to São Paulo
echo "Setting time zone"
sudo timedatectl set-timezone America/Sao_Paulo

# Verify locale settings
echo "Verifies locales"
locale

# Check if ROS 2 installation directory exists

echo "Checks for ROS 2 existence"
if [ -d "/opt/ros" ]; then
    # ROS 2 installation directory exists
    echo "ROS 2 is installed"  # Echo to terminal
    source "$current_dir/run_workspace.sh"
else
    # ROS 2 installation directory does not exist
    echo "ROS 2 is not installed"  # Echo to terminal
    source "$current_dir/prepare_ros.sh"
fi
