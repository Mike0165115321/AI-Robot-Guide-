import os
import io
import asyncio
import threading
import json
import socket
import subprocess # Added
import numpy as np
import math
from pathlib import Path
from typing import List, Optional, Dict
from PIL import Image
from fastapi import FastAPI, HTTPException, Response
from pydantic import BaseModel
import uvicorn

# ROS 2 Imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist, Point
    from sensor_msgs.msg import LaserScan
    from nav2_msgs.action import NavigateToPose
    from tf2_ros import Buffer, TransformListener
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
except ImportError:
    print("CRITICAL: ROS 2 libraries not found. Ensure you are running with Python 3.10 and ROS 2 sourced.")
    exit(1)

# --- Configuration ---
MAP_DIR = Path(__file__).parent.parent / "maps"
MAP_DIR.mkdir(parents=True, exist_ok=True)
HOST = "0.0.0.0"
PORT = 8015

app = FastAPI(title="Nav Bridge", description="Bridge for ROS 2 Humble (Py3.10) <-> AI Backend (Py3.12)")

# --- ROS Node & Logic ---
class NavBridgeNode(Node):
    def __init__(self):
        super().__init__('nav_bridge_node')
        
        # SLAM / Map Subscription
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self._map_callback, qos)
        self.latest_map_image = None
        self.map_metadata = None

        # TF / Pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Navigation Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # UDP / Twist Publisher (Replacing ros2_bridge.py logic partially or fully?)
        # For now, let's keep it simple: We just publish cmd_vel if requested via API
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initial Pose Publisher (RViz 2D Pose Estimate)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Laser Scan Subscription (RViz Visualization)
        # Use BEST_EFFORT to match typically configured laser drivers (e.g. rplidar, ld06)
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_callback, scan_qos)

        self.latest_scan = None
        
        # RViz Goal Subscription
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self._goal_callback, 10)
        self.latest_rviz_goal = None

        # Navigation Status tracking
        self.nav_status = "IDLE" # IDLE, ACTIVE, SUCCEEDED, FAILED, CANCELED
        
        self.get_logger().info("Bridge Server Status initialized to IDLE")
        self.get_logger().info("🌉 Nav Bridge Node Initialized")

    def _goal_callback(self, msg: PoseStamped):
        # Cache the goal set in RViz
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.latest_rviz_goal = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "theta": theta
        }
        self.get_logger().info(f"📍 Captured RViz Goal: {self.latest_rviz_goal}")

    def _scan_callback(self, msg: LaserScan):
        # Downsample scan for web visualization (save bandwidth)
        # Convert to simple array of points [x, y, x, y...] relative to robot frame
        # Or just sending ranges? Sending XY points is easier for frontend rendering.
        # But ranges + angle_min/increment is less data. Let's send raw params + ranges (downsampled)
        
        # Optimization: Send every 5th point
        step = 5
        self.latest_scan = {
            "angle_min": msg.angle_min,
            "angle_increment": msg.angle_increment * step,
            "ranges": [r if not math.isinf(r) else 0.0 for r in msg.ranges[::step]],
            "max_range": msg.range_max
        }

    def _map_callback(self, msg: OccupancyGrid):
        # Cache metadata
        self.map_metadata = {
            "resolution": msg.info.resolution,
            "origin": [
                msg.info.origin.position.x,
                msg.info.origin.position.y,
                0.0 # Simplify Z
            ],
            "width": msg.info.width,
            "height": msg.info.height
        }
    
        # Convert to Image
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        img_data = np.zeros((height, width), dtype=np.uint8)
        img_data.fill(127) # Unknown
        img_data[data == 0] = 255 # Free
        img_data[data == 100] = 0 # Occupied
        
        img_data = np.flipud(img_data)
        image = Image.fromarray(img_data, mode='L')
        
        buf = io.BytesIO()
        image.save(buf, format='PNG')
        self.latest_map_image = buf.getvalue()

    def get_robot_pose(self):
        try:
            if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.0)):
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                
                # Quaternion to Euler
                qx = t.transform.rotation.x
                qy = t.transform.rotation.y
                qz = t.transform.rotation.z
                qw = t.transform.rotation.w
                siny_cosp = 2 * (qw * qz + qx * qy)
                cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                theta = math.atan2(siny_cosp, cosy_cosp)
                
                return {
                    "x": t.transform.translation.x,
                    "y": t.transform.translation.y,
                    "theta": theta
                }
            return None
        except Exception as e:
            return None

    def publish_velocity(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_vel_pub.publish(msg)

    def set_initial_pose(self, x, y, theta):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Theta to Quaternion
        cy = math.cos(theta * 0.5); sy = math.sin(theta * 0.5)
        cp = math.cos(0); sp = math.sin(0)
        cr = math.cos(0); sr = math.sin(0)
        
        msg.pose.pose.orientation.w = cy * cp * cr + sy * sp * sr
        msg.pose.pose.orientation.z = sy * cp * cr - cy * sp * sr
        msg.pose.pose.orientation.x = cy * cp * sr - sy * sp * cr
        msg.pose.pose.orientation.y = sy * cp * sr + cy * sp * cr

        # Covariance (Standard AMCL default)
        msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.068] # yaw variance

        self.initial_pose_pub.publish(msg)

# Global Node Instance
nav_node: Optional[NavBridgeNode] = None

# --- Lifespan ---
@app.on_event("startup")
async def startup_event():
    global nav_node
    rclpy.init()
    nav_node = NavBridgeNode()
    
    def spin():
        rclpy.spin(nav_node)
    
    t = threading.Thread(target=spin, daemon=True)
    t.start()

@app.on_event("shutdown")
async def shutdown_event():
    global nav_node
    if nav_node:
        nav_node.destroy_node()
    rclpy.shutdown()

# --- Endpoints ---

@app.get("/health")
def health():
    return {"status": "ok", "ros": rclpy.ok()}

# 1. Mapping
@app.get("/map/preview")
def get_map_preview():
    if not nav_node or not nav_node.latest_map_image:
        raise HTTPException(status_code=404, detail="Map not ready")
    return Response(content=nav_node.latest_map_image, media_type="image/png")

@app.get("/map/live/metadata")
def get_live_map_metadata():
    if not nav_node or not nav_node.map_metadata:
            # Return default if not ready
            return {"resolution": 0.05, "origin": [0.0, 0.0, 0.0], "width": 0, "height": 0}
    return nav_node.map_metadata

@app.post("/map/save/{name}")
async def save_map(name: str):
    # Use map_saver_cli to save PGM and YAML
    safe_name = "".join([c for c in name if c.isalpha() or c.isdigit() or c in ('-', '_')]).rstrip()
    target_path = str(MAP_DIR / safe_name)
    debug_log = "/tmp/save_map_debug.log"
    
    # Wrapped command to ensure environment
    full_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 run nav2_map_server map_saver_cli -f {target_path} "
        f"--ros-args -p save_map_timeout:=10000.0"
    )
    
    cmd = ["/bin/bash", "-c", full_cmd]
    
    try:
        with open(debug_log, "w") as log:
            log.write(f"Executing: {full_cmd}\n")
            
            proc = await asyncio.create_subprocess_exec(
                *cmd, stdout=log, stderr=log
            )
            
            # Wait with timeout
            try:
                await asyncio.wait_for(proc.wait(), timeout=15.0)
            except asyncio.TimeoutError:
                proc.kill()
                raise HTTPException(status_code=500, detail="Map Save Timed Out (15s)")

            if proc.returncode == 0:
                # Verify file creation
                if (MAP_DIR / f"{safe_name}.yaml").exists():
                    return {"status": "success", "path": target_path}
                else:
                    return {"status": "error", "detail": "Command success but file missing. See /tmp/save_map_debug.log"}
            else:
                 return {"status": "error", "detail": f"Process failed code {proc.returncode}. See /tmp/save_map_debug.log"}
                 
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/map/list")
def list_maps():
    maps = []
    for f in MAP_DIR.glob("*.yaml"):
        maps.append({"name": f.stem, "created_at": f.stat().st_mtime})
    maps.sort(key=lambda x: x["created_at"], reverse=True)
    return {"maps": maps}

@app.delete("/map/{name}")
def delete_map(name: str):
    (MAP_DIR / f"{name}.yaml").unlink(missing_ok=True)
    (MAP_DIR / f"{name}.pgm").unlink(missing_ok=True)
    return {"status": "deleted"}

@app.get("/map/image/{name}")
def get_static_map_image(name: str):
    pgm_path = MAP_DIR / f"{name}.pgm"
    if not pgm_path.exists():
        raise HTTPException(status_code=404)
    
    try:
        with Image.open(pgm_path) as img:
            buf = io.BytesIO()
            img.save(buf, format='PNG')
            return Response(content=buf.getvalue(), media_type="image/png")
    except Exception:
        raise HTTPException(status_code=500)

@app.get("/map/metadata/{name}")
def get_map_metadata(name: str):
    # Retrieve static metadata from YAML
    yaml_path = MAP_DIR / f"{name}.yaml"
    if not yaml_path.exists():
        raise HTTPException(status_code=404)
    
    try:
        content = yaml_path.read_text()
        resolution = 0.05
        origin = [0.0, 0.0, 0.0]
        for line in content.split('\n'):
            if 'resolution:' in line: resolution = float(line.split(':')[1])
            if 'origin:' in line: 
                val = line.split(':')[1].strip().strip('[]')
                origin = [float(x) for x in val.split(',')]
        return {"resolution": resolution, "origin": origin}
    except:
        return {"resolution": 0.05, "origin": [0,0,0]}

# 2. Navigation
@app.get("/nav/pose")
def get_pose():
    if not nav_node: return {"error": "Node not ready"}
    pose = nav_node.get_robot_pose()
    if not pose: return {"error": "TF not available"}
    return pose

class NavGoal(BaseModel):
    x: float
    y: float
    theta: float

@app.post("/nav/goto")
def goto_pose(goal: NavGoal):
    if not nav_node: raise HTTPException(status_code=503)
    
    msg = NavigateToPose.Goal()
    msg.pose.header.frame_id = 'map'
    msg.pose.header.stamp = nav_node.get_clock().now().to_msg()
    msg.pose.pose.position.x = goal.x
    msg.pose.pose.position.y = goal.y
    
    # Theta to Quaternion
    cy = math.cos(goal.theta * 0.5); sy = math.sin(goal.theta * 0.5)
    cp = math.cos(0); sp = math.sin(0)
    cr = math.cos(0); sr = math.sin(0)
    msg.pose.pose.orientation.w = cy * cp * cr + sy * sp * sr
    msg.pose.pose.orientation.z = sy * cp * cr - cy * sp * sr
    
    nav_node.nav_client.wait_for_server()
    
    send_goal_future = nav_node.nav_client.send_goal_async(msg)
    nav_node.nav_status = "ACTIVE"
    
    def goal_response_callback(future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            nav_node.get_logger().info('Goal rejected')
            nav_node.nav_status = "FAILED"
            return

        nav_node.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(get_result_callback)

    def get_result_callback(future):
        from action_msgs.msg import GoalStatus
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            nav_node.get_logger().info('Goal succeeded!')
            nav_node.nav_status = "SUCCEEDED"
        else:
            nav_node.get_logger().info(f'Goal failed with status: {result.status}')
            nav_node.nav_status = "FAILED"

    send_goal_future.add_done_callback(goal_response_callback)
    return {"status": "sent"}

@app.get("/nav/status")
def get_nav_status():
    if not nav_node: return {"status": "UNKNOWN"}
    return {"status": nav_node.nav_status}

@app.get("/nav/last_rviz_goal")
def get_last_rviz_goal():
    if not nav_node or not nav_node.latest_rviz_goal:
        return {"error": "No goal set in RViz yet"}
    return nav_node.latest_rviz_goal

# --- Map Loading & Localization ---
nav_process: Optional[asyncio.subprocess.Process] = None
rviz_process: Optional[subprocess.Popen] = None

async def start_rviz(force=False):
    global rviz_process
    
    if force:
        try: 
            subprocess.run(["pkill", "-9", "-f", "rviz2"], stderr=subprocess.DEVNULL)
            await asyncio.sleep(0.5)
        except: pass
    else:
        # Check if rviz2 is already running (system-wide)
        try:
            res = subprocess.run(["pgrep", "-f", "rviz2"], capture_output=True)
            if res.returncode == 0:
                print("INFO: RViz2 is already running.")
                return
        except: pass

    rviz_config = Path(__file__).parent.parent / "config" / "rviz" / "robot_rviz2.rviz"
    if not rviz_config.exists():
        print(f"ERROR: RViz config not found at {rviz_config}")
        return

    cmd = ["rviz2", "-d", str(rviz_config)]
    try:
        # We use a non-async Popen because RViz is a GUI app that stays open
        # and we don't necessarily want to wait for it or manage its lifecycle strictly like nav_process
        rviz_process = subprocess.Popen(
            cmd, 
            stdout=subprocess.DEVNULL, 
            stderr=subprocess.DEVNULL,
            env={**os.environ, "DISPLAY": ":0"} # Force display if needed
        )
        print(f"INFO: RViz2 started (PID: {rviz_process.pid})")
    except Exception as e:
        print(f"ERROR: Failed to launch RViz2: {e}")

@app.post("/nav/load_map/{map_name}")
async def load_map(map_name: str):
    global nav_process, slam_process
    
    # 1. Stop SLAM if running
    if slam_process:
        await stop_slam()
        
    # 2. Stop existing Nav2/AMCL if running (via our process or system)
    if nav_process:
        nav_process.terminate()
        try:
            await asyncio.wait_for(nav_process.wait(), timeout=5.0)
        except:
            nav_process.kill()
        nav_process = None

    # Force kill system-wide to be sure
    try:
        # Kill all nav2 related components
        subprocess.run(["pkill", "-9", "-f", "nav2"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "amcl"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "map_server"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "lifecycle_manager"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "rviz2"], stderr=subprocess.DEVNULL)
        
        # KEY: Clear ROS 2 Shared Memory (fixes "Failed init_port" errors)
        subprocess.run(["rm", "-rf", "/dev/shm/fastrtps*"], stderr=subprocess.DEVNULL)
        
        await asyncio.sleep(2.0)
    except: pass
    
    # 3. Launch Nav2 Bringup (Localization + Navigation)
    # bringup_launch.py includes AMCL, Map Server, Planner, Controller, etc.
    map_yaml = MAP_DIR / f"{map_name}.yaml"
    if not map_yaml.exists():
        raise HTTPException(status_code=404, detail="Map YAML not found")
        
    params_file = Path(__file__).parent.parent / "core" / "hardware" / "params" / "nav2_params.yaml"

    cmd = [
        "ros2", "launch", "nav2_bringup", "bringup_launch.py",
        f"map:={str(map_yaml)}",
        f"params_file:={str(params_file)}",
        "use_sim_time:=false",
        "autostart:=true"
    ]
    
    try:
        nav_process = await asyncio.create_subprocess_exec(
            *cmd, stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL
        )
        # 4. Auto-Launch RViz
        await asyncio.sleep(2.0) # Wait for Nav2 to stabilize a bit
        await start_rviz()
        
        return {"status": "started", "map": map_name, "pid": nav_process.pid}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# --- SLAM Management ---
slam_process: Optional[asyncio.subprocess.Process] = None

@app.post("/slam/start")
async def start_slam():
    global slam_process
    if slam_process and slam_process.returncode is None:
        return {"status": "already_running"}

    # Kill conflicting nodes (AMCL, Map Server) to prevent TF/Map conflicts
    # Since start_robot.sh launches them, we must stop them to run SLAM.
    try:
        subprocess.run(["pkill", "-f", "amcl"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-f", "map_server"], stderr=subprocess.DEVNULL)
        # Also kill any existing slam_toolbox to be safe
        subprocess.run(["pkill", "-f", "slam_toolbox"], stderr=subprocess.DEVNULL)
        await asyncio.sleep(1.0) # Wait for cleanup
    except:
        pass
    
    # Use our optimized params file
    backend_dir = Path(__file__).parent.parent
    params_path = str(backend_dir / "core" / "hardware" / "params" / "slam_params.yaml")
    
    cmd = [
        "ros2", "launch", "slam_toolbox", "online_async_launch.py",
        f"slam_params_file:={params_path}"
    ]
    try:
        slam_process = await asyncio.create_subprocess_exec(
            *cmd, stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL
        )
        return {"status": "started", "pid": slam_process.pid}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/slam/stop")
async def stop_slam():
    global slam_process
    if slam_process and slam_process.returncode is None:
        slam_process.terminate()
        try:
            await asyncio.wait_for(slam_process.wait(), timeout=5.0)
        except asyncio.TimeoutError:
            slam_process.kill()
        slam_process = None
        return {"status": "stopped"}
    return {"status": "not_running"}

# 3. Hardware (Proxy moved here for Py3.10 compat if needed, but simple Twist is enough)
class MoveCmd(BaseModel):
    vx: float
    vy: float
    wz: float

@app.post("/hardware/move")
def move_robot(cmd: MoveCmd):
    if nav_node:
        nav_node.publish_velocity(cmd.vx, cmd.vy, cmd.wz)
@app.post("/hardware/move")
def move_robot(cmd: MoveCmd):
    if nav_node:
        nav_node.publish_velocity(cmd.vx, cmd.vy, cmd.wz)
    return {"status": "ok"}

# 4. RViz Features
class InitialPose(BaseModel):
    x: float
    y: float
    theta: float

@app.post("/nav/initial_pose")
def set_initial_pose(pose: InitialPose):
    if nav_node:
        nav_node.set_initial_pose(pose.x, pose.y, pose.theta)
        return {"status": "published"}
    return {"status": "error"}

@app.get("/nav/scan")
def get_scan():
    if not nav_node or not nav_node.latest_scan:
        return {"error": "No scan data"}
    return nav_node.latest_scan

@app.post("/nav/open_rviz")
async def open_rviz_endpoint():
    await start_rviz(force=True)
    return {"status": "rviz_launched"}

if __name__ == "__main__":
    uvicorn.run(app, host=HOST, port=PORT)
# Test Hot Reload
