import httpx
import json
from typing import List, Dict, Optional
from pathlib import Path

# Config
BRIDGE_URL = "http://localhost:8015"

class MappingService:
    def __init__(self, map_dir: str):
        # We don't use map_dir directly in Main Backend anymore, 
        # but we keep the signature valid if needed or just ignore it.
        pass
        
    def set_ros_node(self, node):
        # No longer needed, but kept for compatibility during transition
        pass

    async def get_maps(self) -> List[Dict]:
        async with httpx.AsyncClient() as client:
            try:
                res = await client.get(f"{BRIDGE_URL}/map/list")
                if res.status_code == 200:
                    return res.json().get("maps", [])
                return []
            except Exception:
                return []

    async def delete_map(self, map_name: str) -> bool:
        async with httpx.AsyncClient() as client:
            try:
                res = await client.delete(f"{BRIDGE_URL}/map/{map_name}")
                return res.status_code == 200
            except:
                return False

    async def get_live_map_image(self) -> Optional[bytes]:
        async with httpx.AsyncClient() as client:
            try:
                res = await client.get(f"{BRIDGE_URL}/map/preview")
                if res.status_code == 200:
                    return res.content
                return None
            except:
                return None

    async def get_static_map_image(self, map_name: str) -> Optional[bytes]:
        async with httpx.AsyncClient() as client:
            try:
                res = await client.get(f"{BRIDGE_URL}/map/image/{map_name}")
                if res.status_code == 200:
                    return res.content
                return None
            except:
                return None
                
    async def get_map_metadata(self, map_name: str) -> Optional[Dict]:
        async with httpx.AsyncClient() as client:
            try:
                res = await client.get(f"{BRIDGE_URL}/map/metadata/{map_name}")
                if res.status_code == 200:
                    return res.json()
                return None
            except:
                return None

    async def get_live_map_metadata(self) -> Optional[Dict]:
        async with httpx.AsyncClient() as client:
            try:
                res = await client.get(f"{BRIDGE_URL}/map/live/metadata")
                if res.status_code == 200:
                    return res.json()
                return None
            except:
                return None

    async def start_mapping(self):
        async with httpx.AsyncClient() as client:
            try:
                res = await client.post(f"{BRIDGE_URL}/slam/start")
                return res.json()
            except Exception as e:
                return {"status": "error", "detail": str(e)}

    async def stop_mapping(self):
        async with httpx.AsyncClient() as client:
            try:
                res = await client.post(f"{BRIDGE_URL}/slam/stop")
                return res.json()
            except Exception as e:
                return {"status": "error", "detail": str(e)}

    async def save_map(self, map_name: str) -> Dict:
        async with httpx.AsyncClient() as client:
            res = await client.post(f"{BRIDGE_URL}/map/save/{map_name}")
            if res.status_code == 200:
                return res.json()
            else:
                raise RuntimeError(f"Bridge Error: {res.text}")

# Singleton
mapping_service = MappingService(map_dir="")
