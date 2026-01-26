import subprocess
import os
import signal
import logging
import socket
import json
from typing import Dict, Optional
from pathlib import Path

# Path to the scripts directory
# Assuming this file is in Back-end/core/services/
# Scripts are in Back-end/scripts/
BACKEND_ROOT = Path(__file__).resolve().parent.parent.parent
SCRIPTS_DIR = BACKEND_ROOT / "scripts"

class HardwareService:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(HardwareService, cls).__new__(cls)
            cls._instance.initialized = False
        return cls._instance

    def __init__(self):
        if self.initialized:
            return
        
        self.running_processes: Dict[str, subprocess.Popen] = {}
        
        # UDP Configuration for ROS 2 Bridge
        self.udp_ip = "127.0.0.1"
        self.udp_port = 9999
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Speed Configuration (Adjustable at runtime)
        self.max_linear_speed = 0.5   # m/s (default)
        self.max_angular_speed = 1.0  # rad/s (default)
        
        self.initialized = True
        logging.info("🔧 HardwareService initialized")

    def _get_script_path(self, script_name: str) -> str:
        return str(SCRIPTS_DIR / f"{script_name}.sh")

    async def launch_script(self, script_name: str) -> Dict[str, any]:
        """
        Launch a shell script in a new process group.
        """
        script_path = self._get_script_path(script_name)
        
        if not os.path.exists(script_path):
            raise FileNotFoundError(f"Script {script_name} not found at {script_path}")

        if script_name in self.running_processes:
            # Check if actually running
            if self.running_processes[script_name].poll() is None:
                return {"status": "already_running", "pid": self.running_processes[script_name].pid}

        # Open log file
        log_file_path = f"/tmp/{script_name}.log"
        log_file = open(log_file_path, "w")

        try:
            # Run process with logs redirected to file
            # preexec_fn=os.setsid is crucial for killing the whole process tree later
            proc = subprocess.Popen(
                ["/bin/bash", script_path],
                cwd=str(BACKEND_ROOT.parent), # Adjust CWD if necessary, previously /home/robot22
                stdout=log_file,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )
            self.running_processes[script_name] = proc
            logging.info(f"🚀 Launched {script_name} with PID {proc.pid}")
            return {"status": "launched", "pid": proc.pid}
        except Exception as e:
            log_file.close()
            logging.error(f"Failed to launch {script_name}: {e}")
            raise e

    async def stop_script(self, script_name: str) -> Dict[str, any]:
        """
        Stop a running script gracefully, then forcefully if needed.
        """
        if script_name not in self.running_processes:
            return {"status": "not_running", "message": "Script is not tracked as running."}

        proc = self.running_processes[script_name]

        try:
            if proc.poll() is not None:
                del self.running_processes[script_name]
                return {"status": "stopped", "message": "Process had already finished."}

            # Try SIGINT (Ctrl+C) first
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)

            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logging.warning(f"⚠️ {script_name} timed out, forcing SIGKILL.")
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)

            del self.running_processes[script_name]
            
            # --- Guarantee Cleanup for start_robot ---
            if script_name == "start_robot":
                logging.info("🧹 Forcing specific cleanup for Robot System Restart...")
                # These are the commands verified by user to fix the motor/restart issue
                subprocess.run(["pkill", "-f", "micro_ros_agent"], stderr=subprocess.DEVNULL)
                subprocess.run(["pkill", "-f", "ros2_bridge.py"], stderr=subprocess.DEVNULL)
                subprocess.run(["pkill", "-f", "qos_relay_node.py"], stderr=subprocess.DEVNULL)
                subprocess.run(["pkill", "-f", "scan_relay.py"], stderr=subprocess.DEVNULL)  # Legacy
            # ----------------------------------------

            logging.info(f"🛑 Stopped {script_name} (PID {proc.pid})")
            return {"status": "stopped", "pid": proc.pid}

        except Exception as e:
            logging.error(f"Error stopping {script_name}: {e}")
            raise e

    async def get_script_logs(self, script_name: str, lines: int = 50) -> str:
        """
        Read the last N lines of the script's log file.
        """
        log_path = f"/tmp/{script_name}.log"
        if not os.path.exists(log_path):
            return "No log file found."

        try:
            with open(log_path, "r") as f:
                content = f.readlines()
                return "".join(content[-lines:])
        except Exception as e:
            return f"Error reading logs: {str(e)}"

    async def get_status(self) -> Dict[str, str]:
        """
        Get status of all tracked scripts.
        """
        dead_keys = []
        status_dict = {}

        for name, proc in self.running_processes.items():
            if proc.poll() is not None:
                dead_keys.append(name)
            else:
                status_dict[name] = "running"

        for k in dead_keys:
            del self.running_processes[k]

        return status_dict

    async def send_move_command(self, vx: float, vy: float = 0.0, wz: float = 0.0) -> bool:
        """
        Send velocity command to ROS 2 Bridge via UDP.
        Supports 3-axis control for Mecanum wheels (vx, vy, wz).
        Frontend sends normalized values (-1.0 to 1.0).
        Kinematics Bridge handles conversion to RPM.
        """
        try:
            # Scale inputs using instance-level max speeds (Slider Control)
            # Default Max Linear: 0.5, Max Angular: 1.0
            # User slider adjusts these values via /config/speed
            final_vx = max(min(vx, 1.0), -1.0) * self.max_linear_speed
            final_vy = max(min(vy, 1.0), -1.0) * self.max_linear_speed
            final_wz = max(min(wz, 1.0), -1.0) * self.max_angular_speed * -1  # Invert angular

            # Note: Kinematics Bridge BASE_RPM is now 120.
            # If max_linear_speed is 0.5 -> sends 0.5 -> kinematics uses 0.5 * 120 = 60 RPM (Safe default)
            # If max_linear_speed is 1.0 -> sends 1.0 -> kinematics uses 1.0 * 120 = 120 RPM (Fast)

            payload = json.dumps({"vx": final_vx, "vy": final_vy, "wz": final_wz}).encode('utf-8')
            self.sock.sendto(payload, (self.udp_ip, self.udp_port))
            return True
        except Exception as e:
            logging.error(f"UDP Send failed: {e}")
            raise e

    def set_max_speed(self, linear: float = None, angular: float = None):
        """
        Set maximum speed limits at runtime.
        - linear: Max linear speed in m/s (affects vx, vy)
        - angular: Max angular speed in rad/s (affects wz)
        """
        if linear is not None:
            self.max_linear_speed = max(0.1, min(linear, 2.0))  # Clamp 0.1 - 2.0 m/s
            logging.info(f"⚙️ Max Linear Speed set to {self.max_linear_speed} m/s")
        if angular is not None:
            self.max_angular_speed = max(0.1, min(angular, 3.0))  # Clamp 0.1 - 3.0 rad/s
            logging.info(f"⚙️ Max Angular Speed set to {self.max_angular_speed} rad/s")

    def get_speed_config(self) -> dict:
        """Return current speed configuration."""
        return {
            "max_linear_speed": self.max_linear_speed,
            "max_angular_speed": self.max_angular_speed
        }

# Global Instance
hardware_service = HardwareService()

