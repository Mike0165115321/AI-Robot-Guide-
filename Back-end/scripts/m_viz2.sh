#! /bin/bash
export DISPLAY=:0
# export ROS_MASTER_URI=http://localhost:11311 # Not needed for ROS2
export ROS_IP=$(hostname -I | awk '{print $1}')
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/ros2robot/install/setup.bash
rviz2 -d /home/robot22/robot_rviz2.rviz
