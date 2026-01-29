from fastapi import APIRouter, HTTPException, UploadFile, File
from pydantic import BaseModel
from typing import List, Optional
from core.services.mapping_service import mapping_service
from core.services.navigation_service import navigation_service

router = APIRouter(prefix="/api/navigation", tags=["Navigation & Mapping"])

class WaypointCreate(BaseModel):
    map_name: str
    name: str
    x: float
    y: float
    y: float
    theta: float = 0.0

class InitialPose(BaseModel):
    x: float
    y: float
    theta: float = 0.0

class WaypointSavePose(BaseModel):
    map_name: str
    point_name: str

class NavGoal(BaseModel):
    x: float
    y: float
    theta: float = 0.0

# --- Mapping Endpoints ---

@router.get("/maps")
async def list_maps():
    return {"maps": await mapping_service.get_maps()}

@router.delete("/maps/{map_name}")
async def delete_map(map_name: str):
    success = await mapping_service.delete_map(map_name)
    if not success:
        raise HTTPException(status_code=404, detail="Map not found")
    return {"status": "deleted", "map_name": map_name}

@router.post("/mapping/start")
async def start_mapping():
    return await mapping_service.start_mapping()

@router.post("/mapping/stop")
async def stop_mapping():
    return await mapping_service.stop_mapping()

@router.get("/mapping/preview")
async def get_live_map_preview():
    """Get live map from SLAM as PNG image."""
    image_data = await mapping_service.get_live_map_image()
    if not image_data:
        # Return a placeholder or 404
        # For better UX, return a transparent or waiting image
        raise HTTPException(status_code=404, detail="Map not available yet")
    
    from fastapi.responses import Response
    from fastapi.responses import Response
    return Response(content=image_data, media_type="image/png")

@router.get("/mapping/live/metadata")
async def get_live_map_meta():
    """Get live map metadata (resolution, origin) from SLAM."""
    return await mapping_service.get_live_map_metadata()

@router.get("/maps/{map_name}/image")
async def get_map_image(map_name: str):
    """Get static map image (PGM converted to PNG)."""
    image_data = await mapping_service.get_static_map_image(map_name)
    if not image_data:
        raise HTTPException(status_code=404, detail="Map image not found")
        
    from fastapi.responses import Response
    return Response(content=image_data, media_type="image/png")

@router.get("/maps/{map_name}/metadata")
async def get_map_metadata(map_name: str):
    data = await mapping_service.get_map_metadata(map_name)
    if not data:
        raise HTTPException(status_code=404, detail="Map metadata not found")
    return data

@router.post("/mapping/save/{map_name}")
async def save_map(map_name: str):
    try:
        return await mapping_service.save_map(map_name)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# --- Navigation & Waypoints Endpoints ---

@router.get("/pose")
async def get_robot_pose():
    """Get current robot pose (x, y, theta)"""
    return await navigation_service.get_robot_pose()

@router.get("/waypoints/{map_name}")
async def get_waypoints(map_name: str):
    return {"waypoints": navigation_service.get_waypoints(map_name)}

@router.post("/waypoints")
async def add_waypoint(wp: WaypointCreate):
    return await navigation_service.add_waypoint(wp.map_name, wp.name, wp.x, wp.y, wp.theta)

@router.post("/waypoints/save_pose")
async def save_pose_as_waypoint(data: WaypointSavePose):
    try:
        return await navigation_service.save_current_pose(data.map_name, data.point_name)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/waypoints/save_rviz")
async def save_rviz_as_waypoint(data: WaypointSavePose):
    try:
        return await navigation_service.save_last_rviz_goal(data.map_name, data.point_name)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.delete("/waypoints/{map_name}/{point_name}")
async def delete_waypoint(map_name: str, point_name: str):
    success = await navigation_service.delete_waypoint(map_name, point_name)
    if not success:
        raise HTTPException(status_code=404, detail="Waypoint not found")
    return {"status": "deleted"}

@router.post("/nav/load_map/{map_name}")
async def load_map(map_name: str):
    """Load a specific map into the navigation stack (restarts AMCL)."""
    return await navigation_service.load_map(map_name)

@router.post("/nav/initial_pose")
async def set_initial_pose(pose: InitialPose):
    return await navigation_service.set_initial_pose(pose.x, pose.y, pose.theta)

@router.get("/nav/scan")
async def get_scan():
    return await navigation_service.get_scan()

@router.post("/nav/open_rviz")
async def open_rviz():
    return await navigation_service.open_rviz()

@router.post("/goto")
async def goto_pose(goal: NavGoal):
    return await navigation_service.navigate_to(goal.x, goal.y, goal.theta)

@router.post("/stop")
async def stop_navigation():
    # Placeholder
    return {"status": "stopped"}
