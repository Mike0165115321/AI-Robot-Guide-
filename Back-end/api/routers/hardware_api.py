from fastapi import APIRouter, HTTPException, BackgroundTasks
from pydantic import BaseModel
from core.services.hardware_service import hardware_service

router = APIRouter(prefix="/api/hardware", tags=["Hardware Control"])

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
