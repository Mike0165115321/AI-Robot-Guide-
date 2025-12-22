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
    echo -e "\n${RED}🛑 Stopping all services...${NC}"
    kill $(jobs -p) 2>/dev/null
    exit
}
trap cleanup SIGINT SIGTERM

# 1. Start Docker Databases
echo -e "\n${GREEN}📦 Starting Docker Databases...${NC}"
docker-compose up -d mongodb qdrant redis
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
fi

# 3. Start Python Backend
echo -e "\n${GREEN}🐍 Starting Python Backend (port 9090)...${NC}"
cd Back-end
python3 -m uvicorn api.main:app --host 0.0.0.0 --port 9090 --reload &
BACKEND_PID=$!
cd ..

# Wait for backend to start
sleep 3

echo -e "\n${BLUE}══════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}    ✅ Web Development Mode Ready!${NC}"
echo -e "${BLUE}══════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}📍 Frontend:  ${NC}http://localhost:9090"
echo -e "${GREEN}📍 API Docs:  ${NC}http://localhost:9090/docs"
echo -e "${GREEN}📍 Admin:     ${NC}http://localhost:9090/admin"
echo -e "${BLUE}══════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"

wait $BACKEND_PID
