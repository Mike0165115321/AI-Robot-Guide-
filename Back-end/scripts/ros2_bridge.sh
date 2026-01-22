#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/robot22/microros_ws/install/setup.bash
source /home/robot22/ros2robot/install/setup.bash

# Navigate to the correct directory relative to this script
# Scripts are in Back-end/scripts
# Python file is in Back-end/core/hardware
cd "$(dirname "$0")/../core/hardware"

echo "🌉 Starting ROS 2 Bridge..."
python3 ros2_bridge.py
