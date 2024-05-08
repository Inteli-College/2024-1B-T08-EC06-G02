#!/bin/bash

# Log file
LOGS_FOLDER="./logs"
LOG_FILE="$LOGS_FOLDER/install_log_$(date +%Y%m%d%H%M%S).log"

if [ ! -d $LOGS_FOLDER ]; then
    mkdir $LOGS_FOLDER
fi

# Function to log messages with timestamp and tag
log() {
    local timestamp=$(date +"%Y-%m-%d %H:%M:%S")
    local message="$@"
    echo "$timestamp $message" >> "$LOG_FILE"
}

# Function to execute commands and log output
execute_and_log() {
    local command="$@"
    log "[DEBUG] Executing: $command"
    {
        $command
    } | while IFS= read -r line; do
        log "[TRACE] $line"
    done
    local exit_code=${PIPESTATUS[0]}
    if [ $exit_code -eq 0 ]; then
        log "[INFO] Command '$command' executed successfully"
    else
        log "[ERROR] Command '$command' failed with exit code $exit_code"
    fi
    return $exit_code
}

# Function to execute commands and log output with [OUTPUT] tag
execute_and_log_output() {
    local command="$@"
    log "[OUTPUT] $command"
    {
        $command
    } > >(tee -a "$LOG_FILE") 2> >(tee -a "$LOG_FILE" >&2)
    return $?
}

# Check if ROS 2 installation directory exists

execute_and_log echo "Checks for ROS 2 existence"
if [ -d "/opt/ros" ]; then
    # ROS 2 installation directory exists
    execute_and_log echo "ROS 2 is installed"
    execute_and_log source project_runner.sh
else
    # ROS 2 installation directory does not exist
    execute_and_log echo "ROS 2 is not installed"
    execute_and_log source prepare_ros.sh
fi
