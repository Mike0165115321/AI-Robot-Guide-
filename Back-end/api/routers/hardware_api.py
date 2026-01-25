import json
import time
from pathlib import Path
from fastapi import APIRouter, HTTPException, BackgroundTasks
from pydantic import BaseModel
from core.services.hardware_service import hardware_service

router = APIRouter(prefix="/api/hardware", tags=["Hardware Control"])

HEARTBEAT_FILE = Path("/tmp/robot_heartbeat.json")

class MoveCommand(BaseModel):
    vx: float
    wz: float

@router.post("/launch/{script_name}")
async def launch_script(script_name: str, background_tasks: BackgroundTasks):
    """
    Launch a hardware script (ROS 2 node) via HardwareService.
    """
    try:
        return await hardware_service.launch_script(script_name)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/stop/{script_name}")
async def stop_script(script_name: str):
    """
    Stop a running hardware script.
    """
    try:
        return await hardware_service.stop_script(script_name)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/logs/{script_name}")
async def get_script_logs(script_name: str):
    """
    Get logs for a specific script.
    """
    return {"logs": await hardware_service.get_script_logs(script_name)}

@router.get("/status")
async def get_status():
    """
    Get status of all running scripts.
    """
    return {"running_scripts": await hardware_service.get_status()}

@router.get("/robot-status")
async def robot_status():
    """
    Get robot connection status from heartbeat file.
    Returns: offline (system not running), connecting (bridge running but no ESP32), ready (ESP32 connected)
    """
    try:
        # Check if bringup is running
        running_scripts = await hardware_service.get_status()
        bringup_running = "bringup" in running_scripts
        
        if not bringup_running:
            return {"status": "offline", "message": "ระบบปิดอยู่"}
        
        # Check heartbeat file
        if not HEARTBEAT_FILE.exists():
            return {"status": "connecting", "message": "กำลังเชื่อมต่อ..."}
        
        with open(HEARTBEAT_FILE, 'r') as f:
            heartbeat = json.load(f)
        
        # Check if heartbeat is recent (within 3 seconds)
        current_time = time.time()
        last_heartbeat = heartbeat.get("last_heartbeat", 0)
        is_connected = heartbeat.get("connected", False)
        
        if is_connected and (current_time - last_heartbeat) < 3.0:
            return {"status": "ready", "message": "พร้อมใช้งาน!"}
        else:
            return {"status": "connecting", "message": "กำลังเชื่อมต่อ..."}
            
    except Exception as e:
        return {"status": "offline", "message": f"Error: {str(e)}"}

@router.post("/move")
async def move_robot(cmd: MoveCommand):
    """
    Send velocity command to ROS 2 Bridge via UDP.
    """
    try:
        success = await hardware_service.send_move_command(cmd.vx, cmd.wz)
        return {"status": "ok", "sent_udp": success}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
