#!/bin/bash
# ============================================
# 📱 start_line.sh - LINE Integration Mode
# ============================================
# Starts: Docker DBs + ngrok (auto-update) + Backend + LINE Worker
# Use this when you want to test LINE features
# ============================================

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# Configuration
BACKEND_PORT=9090
NGROK_API_URL="http://127.0.0.1:4040/api/tunnels"
ENV_FILE="Back-end/.env"

echo -e "${CYAN}══════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}    📱 AI Robot Guide - LINE Integration Mode${NC}"
echo -e "${CYAN}══════════════════════════════════════════════════════════${NC}"

cleanup() {
    echo -e "\n${RED}🛑 Stopping all services...${NC}"
    kill $(jobs -p) 2>/dev/null
    # Kill ngrok if running
    pkill -f "ngrok http $BACKEND_PORT" 2>/dev/null
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

# 3. Kill existing ngrok if running
pkill -f "ngrok http $BACKEND_PORT" 2>/dev/null

# 4. Start ngrok in background
echo -e "\n${GREEN}🌐 Starting ngrok tunnel...${NC}"
ngrok http $BACKEND_PORT > /dev/null 2>&1 &
NGROK_PID=$!
sleep 3  # Wait for ngrok to start

# 5. Get ngrok public URL and update .env
echo -e "${GREEN}🔄 Getting ngrok URL...${NC}"
NGROK_URL=""
for i in {1..10}; do
    NGROK_URL=$(curl -s $NGROK_API_URL 2>/dev/null | grep -oP '"public_url":"https://[^"]+' | head -1 | sed 's/"public_url":"//')
    if [ -n "$NGROK_URL" ]; then
        break
    fi
    sleep 1
done

if [ -z "$NGROK_URL" ]; then
    echo -e "${RED}❌ Failed to get ngrok URL. Please check ngrok.${NC}"
    exit 1
fi

echo -e "${GREEN}✅ ngrok URL: ${CYAN}$NGROK_URL${NC}"

# 6. Update .env file with new ngrok URL
if [ -f "$ENV_FILE" ]; then
    # Check if LINE_STATIC_BASE_URL exists in .env
    if grep -q "^LINE_STATIC_BASE_URL=" "$ENV_FILE"; then
        # Update existing value
        sed -i "s|^LINE_STATIC_BASE_URL=.*|LINE_STATIC_BASE_URL='$NGROK_URL'|" "$ENV_FILE"
    else
        # Add new line
        echo "LINE_STATIC_BASE_URL='$NGROK_URL'" >> "$ENV_FILE"
    fi
    echo -e "${GREEN}✅ Updated .env with new ngrok URL${NC}"
else
    echo -e "${YELLOW}⚠️ .env file not found at $ENV_FILE${NC}"
fi

# 7. Start Python Backend
echo -e "\n${GREEN}🐍 Starting Python Backend (port $BACKEND_PORT)...${NC}"
cd Back-end
python3 -m uvicorn api.main:app --host 0.0.0.0 --port $BACKEND_PORT --reload &
BACKEND_PID=$!
cd ..
sleep 3

# 8. Start LINE Worker
echo -e "\n${GREEN}🤖 Starting LINE Worker...${NC}"
cd Back-end
PYTHONPATH=$(pwd) python3 workers/worker_line.py &
WORKER_PID=$!
cd ..

# Print summary
echo -e "\n${CYAN}══════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}    ✅ LINE Integration Mode Ready!${NC}"
echo -e "${CYAN}══════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}📍 Local Backend:   ${NC}http://localhost:$BACKEND_PORT"
echo -e "${GREEN}📍 Public URL:      ${NC}${CYAN}$NGROK_URL${NC}"
echo -e "${GREEN}📍 LINE Webhook:    ${NC}${CYAN}$NGROK_URL/api/v1/line/webhook${NC}"
echo -e "${GREEN}📍 ngrok Inspector: ${NC}http://127.0.0.1:4040"
echo -e "${CYAN}══════════════════════════════════════════════════════════${NC}"
echo -e ""
echo -e "${YELLOW}⚠️  อย่าลืมอัปเดต Webhook URL ใน LINE Developers Console!${NC}"
echo -e "${YELLOW}    URL: $NGROK_URL/api/v1/line/webhook${NC}"
echo -e ""
echo -e "${YELLOW}Press Ctrl+C to stop all services${NC}"

# Wait for any process to exit
wait $BACKEND_PID $WORKER_PID
