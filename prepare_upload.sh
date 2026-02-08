#!/bin/bash
BLUE='\033[0;34m'
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}🧹 Stopping Robot Processes for Arduino Upload...${NC}"

# Kill micro-ros-agent (holds Serial /dev/ttyACM0)
echo -e "${YELLOW}Killing micro-ros-agent...${NC}"
pkill -f micro_ros_agent

# Kill robot bringup (ROS2 launch)
echo -e "${YELLOW}Killing robot_bringup...${NC}"
pkill -f robot_bringup

# Kill ros2_bridge if running
echo -e "${YELLOW}Killing ros2_bridge...${NC}"
pkill -f ros2_bridge.py

echo -e "${GREEN}✅ Cleaned up! You can now upload to Arduino/ESP32.${NC}"
echo -e "${BLUE}Run ./start_web.sh or your ros start script to resume control after upload.${NC}"
