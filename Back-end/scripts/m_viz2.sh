#! /bin/bash
export ROS_MASTER_URI=http://192.168.1.57:11311
export ROS_IP=192.168.1.50
source /opt/ros/humble/setup.bash
source ~/ros2robot/install/setup.bash
rviz2 -d /home/robot22/robot_ros2.rviz
