#!/bin/bash

# Change directory to Back-end
cd "$(dirname "$0")/Back-end"

echo "🚀 กำลังเปิดใช้งานระบบ API Backend (Local)..."

# Activate the virtual environment if it exists
if [ -d "../.venv-robot" ]; then
    source ../.venv-robot/bin/activate
    echo "🐍 เปิดใช้งาน Virtual Environment (.venv-robot)..."
fi

# Set PYTHONPATH to include current directory
export PYTHONPATH=$PYTHONPATH:.

# Run FastAPI with Uvicorn on port 9090
echo "📡 เริ่มต้นเซิร์ฟเวอร์ที่ http://localhost:9090"
uvicorn api.main:app --host 0.0.0.0 --port 9090 --reload
