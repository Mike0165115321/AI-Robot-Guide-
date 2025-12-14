#!/bin/bash

# Definition of colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 Starting AI Robot Guide System (Local Hybrid Mode)...${NC}"

# Function to kill processes on exit
cleanup() {
    echo -e "\n${RED}🛑 Stopping all services...${NC}"
    kill $(jobs -p) 2>/dev/null
    exit
}
trap cleanup SIGINT SIGTERM

# 1. Start Databases (Docker)
echo -e "\n${GREEN}📦 Starting Databases (MongoDB & Qdrant)...${NC}"
docker-compose up -d mongodb qdrant
if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Failed to start databases. Make sure Docker is running.${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Databases match configuration.${NC}"

# 2. Start C++ Audio Engine
echo -e "\n${GREEN}🔊 Starting C++ Audio Engine (Port 50051)...${NC}"
if [ -f "cpp-audio-engine/build/audio-engine" ]; then
    cd cpp-audio-engine
    ./build/audio-engine --port 50051 &
    AUDIO_PID=$!
    cd ..
    echo -e "${GREEN}✅ Audio Engine started (PID: $AUDIO_PID)${NC}"
else
    echo -e "${RED}❌ C++ Audio Engine binary not found. Please build it first.${NC}"
fi

# 3. Start Python Backend
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

# 4. Wait for Backend to be ready
sleep 2

# 5. Start Go Gateway
echo -e "\n${GREEN}🐹 Starting Go Gateway (Port 8080)...${NC}"
cd go-gateway
export PYTHON_BACKEND="localhost:9090"
export CPP_AUDIO_ENGINE="localhost:50051"
export GIN_MODE=release
# Prevent auto-download of new toolchains
export GOTOOLCHAIN=local
# Build if main.go changed 
go build -o go-gateway cmd/server/main.go
./go-gateway &
GATEWAY_PID=$!
cd ..
echo -e "${GREEN}✅ Go Gateway started (PID: $GATEWAY_PID)${NC}"

echo -e "\n${BLUE}✨ All Systems GO! Access the app at http://localhost:8080${BLUE}"
echo -e "${BLUE}📝 Press Ctrl+C to stop all services.${NC}"

# Keep script running
wait
