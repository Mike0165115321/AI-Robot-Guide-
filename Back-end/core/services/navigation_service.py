import json
import httpx
from pathlib import Path
from typing import List, Dict, Optional

BRIDGE_URL = "http://localhost:8015"

class NavigationService:
    def __init__(self, waypoint_file: str):
        self.waypoint_file = Path(waypoint_file)
        self.waypoints = self._load_waypoints()
        
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
