#!/bin/bash
# ============================================
# 🌐 start_web.sh - Web Development Mode
# ============================================
# Starts: Docker DBs + Python Backend + Opens Frontend
# Use this for web-based development (no LINE)
# ============================================

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}══════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}    🌐 AI Robot Guide - Web Development Mode${NC}"
echo -e "${BLUE}══════════════════════════════════════════════════════════${NC}"

cleanup() {
    echo -e "\n${RED}🛑 Stopping Web Services...${NC}"
    kill $BACKEND_PID 2>/dev/null
    kill $BRIDGE_PID 2>/dev/null
    
    # Hint to user
    echo -e "${YELLOW}ℹ️  Robot processes (start_robot.sh) are NOT killed automatically.${NC}"
    echo -e "${YELLOW}ℹ️  Use the 'Stop System' button in Settings or kill start_robot manually.${NC}"
    
    exit
}
trap cleanup SIGINT SIGTERM

# 0. Pre-flight Cleanup (Prevent Address already in use)
echo -e "\n${YELLOW}🧹 Cleaning up old processes...${NC}"
pkill -f "ros2_bridge.py" 2>/dev/null
fuser -k 8014/tcp 2>/dev/null
fuser -k 8015/tcp 2>/dev/null
sleep 1

# 1. Start Docker Databases
echo -e "\n${GREEN}📦 Starting Docker Databases...${NC}"
docker compose up -d mongodb qdrant redis
if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Failed to start databases. Is Docker running?${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Databases ready!${NC}"

# 2. Activate Virtual Environment
if [ -d ".venv-robot" ]; then
    source .venv-robot/bin/activate
    echo -e "${GREEN}✅ Virtual environment activated (.venv-robot)${NC}"
elif [ -d "Back-end/venv" ]; then
    source Back-end/venv/bin/activate
    echo -e "${GREEN}✅ Virtual environment activated (Back-end/venv)${NC}"
elif [ -d "Back-end/.venv-robot" ]; then
    source Back-end/.venv-robot/bin/activate
    echo -e "${GREEN}✅ Virtual environment activated (Back-end/.venv-robot)${NC}"
fi

# 2.5 Set Protobuf Python Implementation (Required for Google Assistant library)
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python



# 2.1 Source ROS2 (Required for rclpy)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✅ ROS2 Humble sourced!${NC}"
    
    # ROS2 Bridge is now managed by start_robot.sh or 'Start System' button
    # /usr/bin/python3 Back-end/core/hardware/ros2_bridge.py &
    # BRIDGE_PID=$!
    :
fi

# 2.9 Start Nav Bridge (Sidecar) for ROS 2 (Python 3.10)
echo -e "\n${GREEN}🌉 Starting Nav Bridge (ROS 2 Sidecar - Port 8015)...${NC}"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
# Use the dedicated bridge virtual environment which has access to ROS 2 via PYTHONPATH (sourced above)
# Start Nav Bridge with Hot Reload (Uvicorn)
Back-end/.venv-bridge/bin/python3 -m uvicorn nav_bridge.bridge_server:app --app-dir Back-end --host 0.0.0.0 --port 8015 --reload &
BRIDGE_PID=$!
sleep 2

# 3. Start Python Backend
echo -e "\n${GREEN}🐍 Starting Python Backend (port 8014)...${NC}"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
cd Back-end
python3 -m uvicorn api.main:app --host 0.0.0.0 --port 8014 --reload --no-access-log &
BACKEND_PID=$!
cd ..

# Wait for backend to start
sleep 3

echo -e "\n${BLUE}══════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}    ✅ Web Development Mode Ready!${NC}"
echo -e "${BLUE}══════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}📍 Frontend:  ${NC}http://localhost:8014"
echo -e "${GREEN}📍 API Docs:  ${NC}http://localhost:8014/docs"
echo -e "${GREEN}📍 Admin:     ${NC}http://localhost:8014/admin"
echo -e "${BLUE}══════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"

wait $BACKEND_PID
