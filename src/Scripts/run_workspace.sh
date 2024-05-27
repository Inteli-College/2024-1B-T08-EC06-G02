#bin/bash

#================= Setting the project =====================

# Seting the Domain ID 
read -p "Chose the ROS_DOMAIN_ID <0-232>: " ROS_DOMAIN_ID

# Seting the topic name
read  -p "Chose the Topic name : " TOPIC_NAME  

# Setting the turtlebot model
read -p "Chose the turtlebot model <burger-wafle> : " TORTLEBOT_MODEL

# TURTLEBOT3_MODEL value to set
new_turtlebot3_model="burger"

# Check and update ROS_DOMAIN_ID
if grep -q "ROS_DOMAIN_ID" ~/.bashrc; then
    current_ros_domain_id=$(grep "ROS_DOMAIN_ID" ~/.bashrc | cut -d'=' -f2)
    if [ "$current_ros_domain_id" != "$ROS_DOMAIN_ID" ]; then
        sed -i "s/export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID=$ROS_DOMAIN_ID/" ~/.bashrc
        echo "Updated ROS_DOMAIN_ID in .bashrc."
    else
        echo "ROS_DOMAIN_ID is already set to $ROS_DOMAIN_ID."
    fi
else
    echo "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> ~/.bashrc
    echo "ROS_DOMAIN_ID added to .bashrc."
fi

# Check and update TURTLEBOT3_MODEL
if grep -q "TURTLEBOT3_MODEL" ~/.bashrc; then
    current_turtlebot3_model=$(grep "TURTLEBOT3_MODEL" ~/.bashrc | cut -d'=' -f2)
    if [ "$current_turtlebot3_model" != "$new_turtlebot3_model" ]; then
        sed -i "s/export TURTLEBOT3_MODEL=.*/export TURTLEBOT3_MODEL=$new_turtlebot3_model/" ~/.bashrc
        echo "Updated TURTLEBOT3_MODEL in .bashrc."
    else
        echo "TURTLEBOT3_MODEL is already set to $new_turtlebot3_model."
    fi
else
    echo "export TURTLEBOT3_MODEL=$new_turtlebot3_model" >> ~/.bashrc
    echo "TURTLEBOT3_MODEL added to .bashrc."
fi

source ~/.bashrc

cd ../backend

python3 cli.py