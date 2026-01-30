import json
import httpx
from pathlib import Path
from typing import List, Dict, Optional, Any
import asyncio

BRIDGE_URL = "http://localhost:8015"

class NavigationService:
    def __init__(self, waypoint_file: str):
        self.waypoint_file = Path(waypoint_file)
        self.waypoints = self._load_waypoints()
        self.is_autowalking = False
        
    def set_ros_node(self, node):
        pass

    def _load_waypoints(self) -> Dict:
        if self.waypoint_file.exists():
            try:
                with open(self.waypoint_file, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except Exception:
                return {}
        return {}

    def _save_waypoints(self):
        with open(self.waypoint_file, 'w', encoding='utf-8') as f:
            json.dump(self.waypoints, f, ensure_ascii=False, indent=2)

    async def get_robot_pose(self) -> Dict:
        async with httpx.AsyncClient() as client:
            try:
                res = await client.get(f"{BRIDGE_URL}/nav/pose")
                if res.status_code == 200:
                    return res.json()
                return {"error": "Bridge error"}
            except Exception as e:
                return {"error": str(e)}

    async def save_current_pose(self, map_name: str, point_name: str) -> Dict:
        pose = await self.get_robot_pose()
        if "error" in pose:
            raise RuntimeError(f"Cannot get robot pose: {pose['error']}")

        if map_name not in self.waypoints:
            self.waypoints[map_name] = []

        self.waypoints[map_name] = [wp for wp in self.waypoints[map_name] if wp['name'] != point_name]
        
        new_point = {
            "name": point_name,
            "x": pose["x"],
            "y": pose["y"],
            "theta": pose["theta"]
        }
        self.waypoints[map_name].append(new_point)
        self._save_waypoints()
        return new_point

    async def add_waypoint(self, map_name: str, name: str, x: float, y: float, theta: float = 0.0) -> Dict:
        if map_name not in self.waypoints:
            self.waypoints[map_name] = []
            
        self.waypoints[map_name] = [wp for wp in self.waypoints[map_name] if wp['name'] != name]
        
        new_point = {
            "name": name,
            "x": x,
            "y": y,
            "theta": theta
        }
        self.waypoints[map_name].append(new_point)
        self._save_waypoints()
        return new_point

    def get_waypoints(self, map_name: str) -> List[Dict]:
        return self.waypoints.get(map_name, [])

    async def delete_waypoint(self, map_name: str, point_name: str) -> bool:
        if map_name in self.waypoints:
            initial_len = len(self.waypoints[map_name])
            self.waypoints[map_name] = [wp for wp in self.waypoints[map_name] if wp['name'] != point_name]
            if len(self.waypoints[map_name]) < initial_len:
                self._save_waypoints()
                return True
        return False

    async def navigate_to(self, x: float, y: float, theta: float):
        async with httpx.AsyncClient() as client:
            try:
                res = await client.post(f"{BRIDGE_URL}/nav/goto", json={"x": x, "y": y, "theta": theta})
                return res.json()
            except Exception as e:
                return {"error": str(e)}

    async def load_map(self, map_name: str):
        async with httpx.AsyncClient() as client:
            try:
                # Call bridge to restart nav with new map
                res = await client.post(f"{BRIDGE_URL}/nav/load_map/{map_name}", timeout=60.0)
                if res.status_code == 200:
                    return res.json()
                return {"error": f"Bridge error: {res.text}"}
            except Exception as e:
                return {"error": str(e)}

    async def set_initial_pose(self, x: float, y: float, theta: float):
        async with httpx.AsyncClient() as client:
            try:
                res = await client.post(f"{BRIDGE_URL}/nav/initial_pose", json={"x": x, "y": y, "theta": theta})
                return res.json()
            except Exception as e:
                return {"error": str(e)}

    async def get_scan(self):
        async with httpx.AsyncClient() as client:
            try:
                res = await client.get(f"{BRIDGE_URL}/nav/scan")
                if res.status_code == 200:
                    return res.json()
                return {"error": "Scan unavailable"}
            except Exception as e:
                return {"error": str(e)}

                return {"error": str(e)}

    async def open_rviz(self):
        async with httpx.AsyncClient() as client:
            try:
                res = await client.post(f"{BRIDGE_URL}/nav/open_rviz")
                return res.json()
            except Exception as e:
                return {"error": str(e)}

    async def get_nav_status(self) -> str:
        async with httpx.AsyncClient() as client:
            try:
                res = await client.get(f"{BRIDGE_URL}/nav/status")
                if res.status_code == 200:
                    return res.json().get("status", "UNKNOWN")
                return "UNKNOWN"
            except Exception:
                return "UNKNOWN"

    async def stop_navigation(self):
        # We don't have a direct 'stop' in Bridge yet, but we can send an empty velocity
        # Or just wait for its implementation. For now, let's keep it simple.
        async with httpx.AsyncClient() as client:
            try:
                await client.post(f"{BRIDGE_URL}/hardware/move", json={"vx": 0.0, "vy": 0.0, "wz": 0.0})
                return {"status": "stopped"}
            except Exception as e:
                return {"error": str(e)}

    async def auto_walk(self, map_name: str):
        if self.is_autowalking:
            return {"error": "Already autowalking"}
        
        waypoints = self.get_waypoints(map_name)
        if not waypoints:
            return {"error": f"No waypoints found for map: {map_name}"}
            
        self.is_autowalking = True
        
        async def run_sequence():
            try:
                for wp in waypoints:
                    if not self.is_autowalking:
                        break
                    
                    print(f"DEBUG: Navigating to waypoint: {wp['name']}")
                    await self.navigate_to(wp['x'], wp['y'], wp['theta'])
                    
                    # Wait for status to become ACTIVE
                    for _ in range(10): # 5 seconds timeout
                        status = await self.get_nav_status()
                        if status == "ACTIVE":
                            break
                        await asyncio.sleep(0.5)
                    
                    # Polling for SUCCESS or FAILURE
                    while self.is_autowalking:
                        status = await self.get_nav_status()
                        if status in ["SUCCEEDED", "FAILED", "CANCELED"]:
                            break
                        await asyncio.sleep(1.0)
                    
                    if status != "SUCCEEDED":
                        print(f"DEBUG: Waypoint {wp['name']} failed or was canceled")
                        # Should we continue or stop? Let's continue for now.
                    
                    await asyncio.sleep(2.0) # Pause between waypoints
            finally:
                self.is_autowalking = False
                print("DEBUG: Auto Walk sequence finished")

        # Start in background
        asyncio.create_task(run_sequence())
        return {"status": "autowalk_started", "waypoint_count": len(waypoints)}


    async def save_last_rviz_goal(self, map_name: str, point_name: str) -> Dict:
        async with httpx.AsyncClient() as client:
            try:
                # 1. Get Goal from Bridge
                res = await client.get(f"{BRIDGE_URL}/nav/last_rviz_goal")
                if res.status_code != 200:
                    raise RuntimeError(f"Bridge Error: {res.text}")
                
                goal = res.json()
                if "error" in goal:
                    raise RuntimeError(goal["error"])

                # 2. Save as Waypoint
                if map_name not in self.waypoints:
                    self.waypoints[map_name] = []
                
                # Update if exists
                self.waypoints[map_name] = [wp for wp in self.waypoints[map_name] if wp['name'] != point_name]
                
                new_point = {
                    "name": point_name,
                    "x": goal["x"],
                    "y": goal["y"],
                    "theta": goal["theta"]
                }
                self.waypoints[map_name].append(new_point)
                self._save_waypoints()
                return new_point

            except Exception as e:
                raise RuntimeError(str(e))

# Singleton
navigation_service = NavigationService(waypoint_file="/home/robot22/AI-Robot-Guide-/Back-end/config/waypoints.json")
