# /core/services/alert_manager.py
"""
Alert Manager: จัดการ WebSocket connections และ broadcast alerts
"""

import asyncio
import logging
from typing import Set, Dict, Optional, List
from datetime import datetime, timezone
from fastapi import WebSocket
import json

logger = logging.getLogger(__name__)


class AlertManager:
    """
    Manager สำหรับจัดการ WebSocket connections และ broadcast alerts
    รองรับหลาย connection พร้อมกัน
    """
    
    def __init__(self):
        self.active_connections: Set[WebSocket] = set()
        self._alert_history: List[Dict] = []
        self._max_history = 100  # เก็บประวัติ alert สูงสุด 100 รายการ
        
    async def connect(self, websocket: WebSocket):
        """รับ connection ใหม่"""
        await websocket.accept()
        self.active_connections.add(websocket)
        logger.info(f"🔗 [AlertManager] Client connected. Total: {len(self.active_connections)}")
        
        # ส่ง alerts ล่าสุดให้ client ใหม่
        try:
            await websocket.send_json({
                "type": "connection_established",
                "message": "เชื่อมต่อระบบแจ้งเตือนสำเร็จ",
                "recent_alerts": self._alert_history[-10:]  # ส่ง 10 alerts ล่าสุด
            })
        except Exception as e:
            logger.error(f"❌ [AlertManager] Send welcome error: {e}")
    
    async def disconnect(self, websocket: WebSocket):
        """ปิด connection"""
        self.active_connections.discard(websocket)
        logger.info(f"🔌 [AlertManager] Client disconnected. Total: {len(self.active_connections)}")
    
    async def broadcast_alert(self, alert: Dict):
        """
        ส่ง alert ไปยังทุก connection
        
        Args:
            alert: Alert data dict
        """
        if not self.active_connections:
            logger.debug("⏭️ [AlertManager] ไม่มี client เชื่อมต่อ ข้าม broadcast")
            return
            
        # เพิ่มข้อมูล timestamp และ id
        alert["alert_id"] = f"alert_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}"
        alert["broadcasted_at"] = datetime.now(timezone.utc).isoformat()
        alert["type"] = "alert"
        
        # เก็บประวัติ
        self._alert_history.append(alert)
        if len(self._alert_history) > self._max_history:
            self._alert_history = self._alert_history[-self._max_history:]
        
        # Broadcast ไปทุก connection
        disconnected = set()
        
        for websocket in self.active_connections:
            try:
                await websocket.send_json(alert)
            except Exception as e:
                logger.error(f"❌ [AlertManager] Send error: {e}")
                disconnected.add(websocket)
        
        # ลบ connection ที่ disconnect ออก
        for ws in disconnected:
            self.active_connections.discard(ws)
        
        logger.info(f"📢 [AlertManager] Broadcasted alert to {len(self.active_connections)} clients")
    
    async def send_to_one(self, websocket: WebSocket, data: Dict):
        """ส่งข้อความไปยัง client เดียว"""
        try:
            await websocket.send_json(data)
        except Exception as e:
            logger.error(f"❌ [AlertManager] Send to one error: {e}")
            self.active_connections.discard(websocket)
    
    def get_recent_alerts(self, limit: int = 20) -> List[Dict]:
        """ดึง alerts ล่าสุด"""
        return self._alert_history[-limit:]
    
    def get_alerts_by_severity(self, min_severity: int = 1) -> List[Dict]:
        """ดึง alerts ตาม severity ขั้นต่ำ"""
        return [a for a in self._alert_history if a.get("severity_score", 0) >= min_severity]
    
    def clear_history(self):
        """ล้างประวัติ alerts"""
        self._alert_history = []
        logger.info("🗑️ [AlertManager] Cleared alert history")
    
    @property
    def connection_count(self) -> int:
        """จำนวน connections ปัจจุบัน"""
        return len(self.active_connections)
    
    @property
    def alert_count(self) -> int:
        """จำนวน alerts ในประวัติ"""
        return len(self._alert_history)


# Singleton instance
alert_manager = AlertManager()
