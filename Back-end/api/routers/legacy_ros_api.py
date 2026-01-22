from fastapi import APIRouter, HTTPException, BackgroundTasks
import subprocess
import os
import signal
import logging
from typing import Dict, Optional

router = APIRouter(prefix="/api/ros", tags=["Legacy ROS Control"])

# Store running processes: {script_name: subprocess.Popen}
running_processes: Dict[str, subprocess.Popen] = {}

# Path to the scripts directory
SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../scripts"))

def get_script_path(script_name: str) -> str:
    # Ensure usage of the scripts we copied
    return os.path.join(SCRIPTS_DIR, f"{script_name}.sh")

@router.post("/launch/{script_name}")
async def launch_script(script_name: str, background_tasks: BackgroundTasks):
    """
    Launch a legacy shell script in a new process group.
    """
    script_path = get_script_path(script_name)
    if not os.path.exists(script_path):
        raise HTTPException(status_code=404, detail="Script not found")
        
    if script_name in running_processes:
        # Check if actually running
        if running_processes[script_name].poll() is None:
            return {"status": "already_running", "pid": running_processes[script_name].pid}
            
    # Open log file
    log_file = open(f"/tmp/{script_name}.log", "w")
            
    try:
        # Run process with logs redirected to file
        proc = subprocess.Popen(
            ["/bin/bash", script_path],
            cwd="/home/robot22", 
            stdout=log_file,
            stderr=subprocess.STDOUT, # Merge stderr into stdout
            preexec_fn=os.setsid 
        )
        running_processes[script_name] = proc
        logging.info(f"🚀 Launched {script_name} with PID {proc.pid}")
        return {"status": "launched", "pid": proc.pid}
    except Exception as e:
        log_file.close()
        logging.error(f"Failed to launch {script_name}: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/logs/{script_name}")
async def get_script_logs(script_name: str):
    """
    Read the last 100 lines of the script's log file.
    """
    log_path = f"/tmp/{script_name}.log"
    if not os.path.exists(log_path):
        return {"logs": "No log file found."}
        
    try:
        with open(log_path, "r") as f:
            lines = f.readlines()
            # Return last 50 lines
            return {"logs": "".join(lines[-50:])}
    except Exception as e:
        return {"logs": f"Error reading logs: {str(e)}"}

@router.post("/stop/{script_name}")
async def stop_script(script_name: str):
    """
    Stop a running script.
    """
    if script_name not in running_processes:
        # Check if it's dead but still in dict
        return {"status": "not_running", "message": "Script is not tracked as running."}

    proc = running_processes[script_name]
    
    try:
        # Poll to see if it's already done
        if proc.poll() is not None:
             del running_processes[script_name]
             return {"status": "stopped", "message": "Process had already finished."}

        # Try gentle shutdown first (SIGINT = Ctrl+C) which ROS handles better
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        
        # Verify
        try:
             proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
             logging.warning(f"⚠️ {script_name} timed out, forcing SIGKILL.")
             os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        
        del running_processes[script_name]
        logging.info(f"🛑 Stopped {script_name} (PID {proc.pid})")
        return {"status": "stopped", "pid": proc.pid}

    except Exception as e:
         logging.error(f"Error stopping {script_name}: {e}")
         raise HTTPException(status_code=500, detail=str(e))

@router.get("/status")
async def get_status():
    """
    Get status of all tracked scripts.
    """
    # Clean up dead processes first
    dead_keys = []
    status_dict = {}
    
    for name, proc in running_processes.items():
        if proc.poll() is not None:
            dead_keys.append(name)
        else:
            status_dict[name] = "running"
    
    for k in dead_keys:
        del running_processes[k]
        
    return {"running_scripts": status_dict}

# ==========================================
# 🎮 ROS2 Teleop (UDP Bridge)
# ==========================================
import socket
import json

UDP_IP = "127.0.0.1"
UDP_PORT = 9999
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

from pydantic import BaseModel

class MoveCommand(BaseModel):
    vx: float  # Linear Velocity (m/s)
    wz: float  # Angular Velocity (rad/s)

@router.post("/move")
async def move_robot(cmd: MoveCommand):
    """
    Send velocity command to ROS2 Bridge via UDP.
    """
    try:
        # Scale inputs if necessary (Frontend sends -1.0 to 1.0)
        # Assuming robot max speed ~ 0.5 m/s, max turn ~ 1.0 rad/s
        MAX_LINEAR = 0.5
        MAX_ANGULAR = 1.0
        
        final_vx = cmd.vx * MAX_LINEAR
        final_wz = cmd.wz * MAX_ANGULAR * -1 # Invert angular for correct Joystick feel
        
        payload = json.dumps({"vx": final_vx, "wz": final_wz}).encode('utf-8')
        sock.sendto(payload, (UDP_IP, UDP_PORT))
        
        return {"status": "ok", "sent_udp": True}
    except Exception as e:
        logging.error(f"UDP Send failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))
