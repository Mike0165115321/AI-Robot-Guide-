#!/bin/bash
# Check for virtual environment
if [ -d ".venv-robot" ]; then
    source .venv-robot/bin/activate
fi

export PYTHONPATH=$PYTHONPATH:$(pwd)/Back-end
echo "🚀 กำลังเปิดใช้งานระบบ LINE Worker..."
python3 Back-end/workers/worker_line.py

