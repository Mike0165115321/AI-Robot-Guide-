# /api/routers/alert_api.py
"""
Alert API: WebSocket และ REST endpoints สำหรับระบบแจ้งเตือน
"""

import logging
from fastapi import APIRouter, WebSocket, WebSocketDisconnect, HTTPException
from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime, timezone

from core.services.alert_manager import alert_manager
from core.services.news_scheduler import news_scheduler

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/alerts", tags=["Alerts"])


# ====== Schemas ======

class TestAlertRequest(BaseModel):
    """Schema สำหรับ test alert"""
    severity: int = 4
    summary: str = "ทดสอบระบบแจ้งเตือน"
    category: str = "general"
    location_name: Optional[str] = None
    lat: Optional[float] = None
    lon: Optional[float] = None


class AlertResponse(BaseModel):
    """Schema สำหรับ alert"""
    alert_id: str
    severity_score: int
    summary: str
    category: str
    location_name: Optional[str] = None
    lat: Optional[float] = None
    lon: Optional[float] = None
    action_recommendation: str
    valid_until: str
    broadcasted_at: str


# ====== WebSocket Endpoint ======

@router.websocket("/ws")
async def alert_websocket(websocket: WebSocket):
    """
    WebSocket endpoint สำหรับรับ real-time alerts
    
    เชื่อมต่อ: ws://localhost:9090/alerts/ws
    """
    await alert_manager.connect(websocket)
    
    try:
        while True:
            # Keep connection alive
            # Client สามารถส่ง "ping" มาเพื่อ keep alive
            data = await websocket.receive_text()
            
            if data == "ping":
                await alert_manager.send_to_one(websocket, {"type": "pong"})
            elif data == "get_history":
                await alert_manager.send_to_one(websocket, {
                    "type": "history",
                    "alerts": alert_manager.get_recent_alerts(20)
                })
                
    except WebSocketDisconnect:
        await alert_manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"❌ [AlertWS] Error: {e}")
        await alert_manager.disconnect(websocket)


# ====== REST Endpoints ======

@router.get("/recent")
async def get_recent_alerts(limit: int = 20, min_severity: int = 1):
    """
    ดึง alerts ล่าสุด
    
    Args:
        limit: จำนวนสูงสุดที่ต้องการ
        min_severity: ระดับความสำคัญขั้นต่ำ (1-5)
    """
    alerts = alert_manager.get_recent_alerts(limit)
    
    if min_severity > 1:
        alerts = [a for a in alerts if a.get("severity_score", 0) >= min_severity]
    
    return {
        "success": True,
        "count": len(alerts),
        "alerts": alerts
    }


@router.get("/stats")
async def get_alert_stats():
    """ดึงสถิติระบบ alert"""
    return {
        "success": True,
        "stats": {
            "active_connections": alert_manager.connection_count,
            "total_alerts": alert_manager.alert_count,
            "scheduler_running": news_scheduler.running
        }
    }


@router.post("/test")
async def send_test_alert(request: TestAlertRequest):
    """
    ส่ง test alert (สำหรับ development)
    """
    test_alert = {
        "is_relevant": True,
        "category": request.category,
        "severity_score": request.severity,
        "summary": request.summary,
        "location_name": request.location_name,
        "lat": request.lat,
        "lon": request.lon,
        "valid_until": datetime.now(timezone.utc).isoformat(),
        "action_recommendation": "info_only",
        "original_source": "test"
    }
    
    await alert_manager.broadcast_alert(test_alert)
    
    return {
        "success": True,
        "message": f"Test alert sent to {alert_manager.connection_count} clients",
        "alert": test_alert
    }


@router.post("/poll-now")
async def trigger_manual_poll():
    """
    Trigger ให้ scheduler รันทันที (ดึงข่าว + วิเคราะห์)
    """
    try:
        alerts = await news_scheduler.manual_poll()
        return {
            "success": True,
            "message": f"Manual poll completed. Generated {len(alerts)} alerts.",
            "alerts": alerts
        }
    except Exception as e:
        logger.error(f"❌ [AlertAPI] Manual poll error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/scheduler/start")
async def start_scheduler():
    """เริ่ม scheduler"""
    if news_scheduler.running:
        return {"success": False, "message": "Scheduler กำลังทำงานอยู่แล้ว"}
    
    # ตั้ง callback สำหรับ broadcast
    news_scheduler.set_alert_callback(alert_manager.broadcast_alert)
    news_scheduler.start()
    
    return {"success": True, "message": "Scheduler เริ่มทำงาน"}


@router.post("/scheduler/stop")
async def stop_scheduler():
    """หยุด scheduler"""
    news_scheduler.stop()
    return {"success": True, "message": "Scheduler หยุดทำงาน"}


@router.delete("/history")
async def clear_alert_history():
    """ล้างประวัติ alerts (เฉพาะ in-memory)"""
    alert_manager.clear_history()
    return {"success": True, "message": "ล้างประวัติ alerts แล้ว"}


# ====== MongoDB Endpoints ======

@router.get("/db/recent")
async def get_alerts_from_db(limit: int = 50, min_severity: int = 1, skip: int = 0):
    """
    ดึง alerts จาก MongoDB (persisted)
    
    Args:
        limit: จำนวนสูงสุด
        min_severity: ระดับความสำคัญขั้นต่ำ
        skip: จำนวนที่ข้าม (สำหรับ pagination)
    """
    try:
        from core.services.alert_storage_service import alert_storage_service
        
        alerts = await alert_storage_service.get_recent_alerts(
            limit=limit,
            min_severity=min_severity,
            skip=skip
        )
        
        return {
            "success": True,
            "count": len(alerts),
            "alerts": alerts
        }
    except Exception as e:
        logger.error(f"❌ [AlertAPI] DB query error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/db/stats")
async def get_db_stats():
    """ดึงสถิติ alerts จาก MongoDB"""
    try:
        from core.services.alert_storage_service import alert_storage_service
        
        stats = await alert_storage_service.get_alert_stats()
        
        return {
            "success": True,
            "stats": {
                **stats,
                "active_connections": alert_manager.connection_count,
                "scheduler_running": news_scheduler.running
            }
        }
    except Exception as e:
        logger.error(f"❌ [AlertAPI] DB stats error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/db/{alert_id}/read")
async def mark_alert_as_read(alert_id: str):
    """ทำเครื่องหมายว่าอ่าน alert แล้ว"""
    try:
        from core.services.alert_storage_service import alert_storage_service
        
        success = await alert_storage_service.mark_as_read(alert_id)
        
        if success:
            return {"success": True, "message": "ทำเครื่องหมายว่าอ่านแล้ว"}
        else:
            raise HTTPException(status_code=404, detail="ไม่พบ alert นี้")
    except Exception as e:
        logger.error(f"❌ [AlertAPI] Mark as read error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

