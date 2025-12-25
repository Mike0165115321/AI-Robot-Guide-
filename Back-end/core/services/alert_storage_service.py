# /core/services/alert_storage_service.py
"""
Alert Storage Service: บันทึก alerts ลง MongoDB
"""

import logging
from typing import Dict, List, Optional
from datetime import datetime, timezone, timedelta
from bson import ObjectId

logger = logging.getLogger(__name__)


class AlertStorageService:
    """Service สำหรับเก็บ alerts ใน MongoDB"""
    
    COLLECTION_NAME = "smart_news_alerts"
    
    def __init__(self):
        self._db = None
        self._collection = None
        
    async def _get_collection(self):
        """Lazy load MongoDB collection"""
        if self._collection is None:
            from core.database.mongodb_manager import MongoDBManager
            self._db = MongoDBManager()
            self._collection = self._db.db[self.COLLECTION_NAME]
            
            # สร้าง indexes
            await self._ensure_indexes()
            
        return self._collection
    
    async def _ensure_indexes(self):
        """สร้าง indexes สำหรับ performance"""
        try:
            collection = self._db.db[self.COLLECTION_NAME]
            
            # Index สำหรับ query ตาม severity และ created_at
            collection.create_index([("severity_score", -1), ("created_at", -1)])
            
            # Index สำหรับ TTL (ลบ alerts เก่าอัตโนมัติหลัง 30 วัน)
            collection.create_index(
                "expires_at", 
                expireAfterSeconds=0  # ลบเมื่อ expires_at ถึง
            )
            
            logger.info("✅ [AlertStorage] สร้าง indexes เรียบร้อย")
        except Exception as e:
            logger.error(f"❌ [AlertStorage] สร้าง indexes ล้มเหลว: {e}")
    
    async def save_alert(self, alert: Dict) -> Optional[str]:
        """
        บันทึก alert ลง MongoDB
        
        Args:
            alert: Alert data dict
            
        Returns:
            Alert ID หรือ None ถ้าล้มเหลว
        """
        try:
            collection = await self._get_collection()
            
            # เพิ่ม timestamps
            now = datetime.now(timezone.utc)
            
            alert_doc = {
                **alert,
                "created_at": now,
                "created_at_th": self._format_thai_datetime(now),
                "expires_at": now + timedelta(days=30),  # หมดอายุใน 30 วัน
                "is_read": False
            }
            
            # ลบ fields ที่ไม่ต้องการ
            alert_doc.pop("alert_id", None)
            alert_doc.pop("broadcasted_at", None)
            alert_doc.pop("type", None)
            
            result = collection.insert_one(alert_doc)
            alert_id = str(result.inserted_id)
            
            logger.info(f"💾 [AlertStorage] บันทึก alert: {alert_id}")
            return alert_id
            
        except Exception as e:
            logger.error(f"❌ [AlertStorage] บันทึก alert ล้มเหลว: {e}")
            return None
    
    async def save_alerts_batch(self, alerts: List[Dict]) -> int:
        """
        บันทึก alerts หลายรายการพร้อมกัน
        
        Returns:
            จำนวน alerts ที่บันทึกสำเร็จ
        """
        saved = 0
        for alert in alerts:
            if await self.save_alert(alert):
                saved += 1
        return saved
    
    async def get_recent_alerts(
        self, 
        limit: int = 50, 
        min_severity: int = 1,
        skip: int = 0
    ) -> List[Dict]:
        """
        ดึง alerts ล่าสุด
        
        Args:
            limit: จำนวนสูงสุด
            min_severity: ระดับความสำคัญขั้นต่ำ
            skip: จำนวนที่ข้าม (สำหรับ pagination)
        """
        try:
            collection = await self._get_collection()
            
            cursor = collection.find(
                {"severity_score": {"$gte": min_severity}}
            ).sort("created_at", -1).skip(skip).limit(limit)
            
            alerts = []
            for doc in cursor:
                doc["_id"] = str(doc["_id"])
                alerts.append(doc)
            
            return alerts
            
        except Exception as e:
            logger.error(f"❌ [AlertStorage] ดึง alerts ล้มเหลว: {e}")
            return []
    
    async def get_alerts_by_date(
        self, 
        date: datetime,
        limit: int = 100
    ) -> List[Dict]:
        """ดึง alerts ตามวันที่"""
        try:
            collection = await self._get_collection()
            
            # หา alerts ในวันนั้น
            start = date.replace(hour=0, minute=0, second=0, microsecond=0)
            end = start + timedelta(days=1)
            
            cursor = collection.find({
                "created_at": {"$gte": start, "$lt": end}
            }).sort("created_at", -1).limit(limit)
            
            alerts = []
            for doc in cursor:
                doc["_id"] = str(doc["_id"])
                alerts.append(doc)
            
            return alerts
            
        except Exception as e:
            logger.error(f"❌ [AlertStorage] ดึง alerts by date ล้มเหลว: {e}")
            return []
    
    async def get_alert_stats(self) -> Dict:
        """ดึงสถิติ alerts"""
        try:
            collection = await self._get_collection()
            
            total = collection.count_documents({})
            today = datetime.now(timezone.utc).replace(hour=0, minute=0, second=0, microsecond=0)
            today_count = collection.count_documents({"created_at": {"$gte": today}})
            critical = collection.count_documents({"severity_score": {"$gte": 4}})
            
            return {
                "total_alerts": total,
                "today_alerts": today_count,
                "critical_alerts": critical
            }
            
        except Exception as e:
            logger.error(f"❌ [AlertStorage] ดึง stats ล้มเหลว: {e}")
            return {"total_alerts": 0, "today_alerts": 0, "critical_alerts": 0}
    
    async def mark_as_read(self, alert_id: str) -> bool:
        """ทำเครื่องหมายว่าอ่านแล้ว"""
        try:
            collection = await self._get_collection()
            
            result = collection.update_one(
                {"_id": ObjectId(alert_id)},
                {"$set": {"is_read": True, "read_at": datetime.now(timezone.utc)}}
            )
            
            return result.modified_count > 0
            
        except Exception as e:
            logger.error(f"❌ [AlertStorage] mark_as_read ล้มเหลว: {e}")
            return False
    
    def _format_thai_datetime(self, dt: datetime) -> str:
        """แปลง datetime เป็นรูปแบบไทย"""
        # เปลี่ยน timezone เป็น Bangkok
        bangkok_tz = timezone(timedelta(hours=7))
        dt_th = dt.astimezone(bangkok_tz)
        
        thai_months = [
            "", "มกราคม", "กุมภาพันธ์", "มีนาคม", "เมษายน", "พฤษภาคม", "มิถุนายน",
            "กรกฎาคม", "สิงหาคม", "กันยายน", "ตุลาคม", "พฤศจิกายน", "ธันวาคม"
        ]
        
        # ปี พ.ศ. = ค.ศ. + 543
        thai_year = dt_th.year + 543
        
        return f"{dt_th.day} {thai_months[dt_th.month]} {thai_year} เวลา {dt_th.strftime('%H:%M')} น."


# Singleton instance
alert_storage_service = AlertStorageService()
