# /core/services/air_quality_service.py
"""
Air Quality Service: ดึงข้อมูล PM2.5 จาก WAQI (World Air Quality Index)
ฟรี 1000 calls/วัน
"""

import asyncio
import logging
from typing import Dict, Optional
import aiohttp

from core.config import settings

logger = logging.getLogger(__name__)


class AirQualityService:
    """Service สำหรับดึงข้อมูลคุณภาพอากาศ PM2.5"""
    
    # WAQI API (World Air Quality Index) - ฟรี 1000 calls/day
    WAQI_URL = "https://api.waqi.info/feed"
    
    # สถานีวัดใน/ใกล้น่าน
    STATIONS = [
        "nan",           # น่าน
        "chiangrai",     # เชียงราย (ใกล้สุด)
        "chiangmai",     # เชียงใหม่
        "@8455",         # สถานีน่าน (ID เฉพาะ)
    ]
    
    # AQI Level mapping
    AQI_LEVELS = [
        (0, 50, "ดี", "good", 1),
        (51, 100, "ปานกลาง", "moderate", 2),
        (101, 150, "ไม่ดีต่อกลุ่มเสี่ยง", "unhealthy_sensitive", 3),
        (151, 200, "ไม่ดี", "unhealthy", 4),
        (201, 300, "ไม่ดีมาก", "very_unhealthy", 5),
        (301, 500, "อันตราย", "hazardous", 5)
    ]
    
    def __init__(self):
        # ใช้ WAQI_API_KEY จาก .env
        self.api_key = getattr(settings, 'WAQI_API_KEY', None)
        if not self.api_key:
            # fallback ใช้ demo token (จำกัด)
            self.api_key = "demo"
    
    async def get_pm25(self, city: str = "nan") -> Optional[Dict]:
        """
        ดึงค่า PM2.5 ล่าสุด
        
        Args:
            city: ชื่อเมือง หรือ station ID (เช่น "@8455")
            
        Returns:
            Dict with pm25, aqi, level, severity
        """
        # ลองหลายสถานี
        stations_to_try = [city] + self.STATIONS
        
        for station in stations_to_try:
            result = await self._fetch_station(station)
            if result:
                return result
        
        logger.warning(f"⚠️ [AirQuality] ไม่พบข้อมูลจากทุกสถานี")
        return None
    
    async def _fetch_station(self, station: str) -> Optional[Dict]:
        """ดึงข้อมูลจากสถานีที่กำหนด"""
        try:
            url = f"{self.WAQI_URL}/{station}/?token={self.api_key}"
            
            timeout = aiohttp.ClientTimeout(total=10)
            async with aiohttp.ClientSession(timeout=timeout) as session:
                async with session.get(url) as response:
                    if response.status != 200:
                        return None
                    
                    data = await response.json()
                    
                    if data.get("status") != "ok":
                        return None
                    
                    result = data.get("data", {})
                    
                    # ดึง AQI หลัก
                    aqi = result.get("aqi", 0)
                    if aqi == "-" or not aqi:
                        return None
                    
                    aqi = int(aqi)
                    
                    # ดึง PM2.5 โดยตรง (ถ้ามี)
                    iaqi = result.get("iaqi", {})
                    pm25 = iaqi.get("pm25", {}).get("v", aqi)  # fallback ใช้ AQI
                    
                    # หาระดับ
                    level_th, level_en, severity = self._get_aqi_level(aqi)
                    
                    # ชื่อสถานี
                    city_info = result.get("city", {})
                    station_name = city_info.get("name", station)
                    
                    logger.info(f"✅ [AirQuality] {station_name}: PM2.5={pm25}, AQI={aqi} ({level_th})")
                    
                    return {
                        "pm25": pm25,
                        "aqi": aqi,
                        "aqi_level": level_en,
                        "aqi_level_th": level_th,
                        "severity": severity,
                        "station_name": station_name,
                        "station_id": station,
                        "source": "waqi",
                        "time": result.get("time", {}).get("s", "")
                    }
                    
        except asyncio.TimeoutError:
            logger.error(f"❌ [AirQuality] Timeout: {station}")
            return None
        except Exception as e:
            logger.error(f"❌ [AirQuality] Error ({station}): {e}")
            return None
    
    def _get_aqi_level(self, aqi: int) -> tuple:
        """แปลง AQI เป็น level และ severity"""
        for min_val, max_val, level_th, level_en, severity in self.AQI_LEVELS:
            if min_val <= aqi <= max_val:
                return level_th, level_en, severity
        return "อันตราย", "hazardous", 5
    
    async def get_simple_summary(self) -> str:
        """ดึงสรุปสั้นๆ สำหรับแสดงใน UI"""
        data = await self.get_pm25()
        if not data:
            return "ไม่มีข้อมูลคุณภาพอากาศ"
        
        return f"PM2.5: {data['pm25']} µg/m³ ({data['aqi_level_th']}) - {data['station_name']}"


# Singleton instance
air_quality_service = AirQualityService()
