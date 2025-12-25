#!/bin/bash

# Definition of colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 Starting AI Robot Guide System (Local Hybrid Mode)...${NC}"

# Function to kill processes on exit
# Function to kill processes on exit
cleanup() {
    echo -e "\n${RED}🛑 Stopping all services...${NC}"
    if [ -n "$BACKEND_PID" ]; then
        kill $BACKEND_PID 2>/dev/null
    fi
    exit
}
trap cleanup SIGINT SIGTERM

# 1. Start Databases (Docker)
echo -e "\n${GREEN}📦 Starting Databases (MongoDB & Qdrant)...${NC}"
docker-compose up -d mongodb qdrant redis
if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Failed to start databases. Make sure Docker is running.${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Databases match configuration.${NC}"

# 2. Check and Free Port 9090
echo -e "\n${BLUE}🧹 Checking for existing processes on port 9090...${NC}"
PID=$(lsof -t -i:9090)
if [ -n "$PID" ]; then
    echo -e "${RED}⚠️  Port 9090 is in use by PID $PID. Killing it...${NC}"
    kill -9 $PID
    sleep 1
fi

# 2. Start Python Backend
echo -e "\n${GREEN}🐍 Starting Python Backend (Port 9090)...${NC}"
cd Back-end
# Activate venv if exists, otherwise assume system python
if [ -d "venv" ]; then
    source venv/bin/activate
    ./venv/bin/python3 -m uvicorn api.main:app --host 0.0.0.0 --port 9090 --reload &
elif [ -d "../.venv-robot" ]; then
    source ../.venv-robot/bin/activate
    ../.venv-robot/bin/python3 -m uvicorn api.main:app --host 0.0.0.0 --port 9090 --reload &
else
    python3 -m uvicorn api.main:app --host 0.0.0.0 --port 9090 --reload &
fi
BACKEND_PID=$!
cd ..
echo -e "${GREEN}✅ Python Backend started (PID: $BACKEND_PID)${NC}"

echo -e "\n${BLUE}✨ All Systems GO! Access the app at http://localhost:9090${BLUE}"
echo -e "${BLUE}📝 Press Ctrl+C to stop all services.${NC}"

# Keep script running
wait $BACKEND_PID
