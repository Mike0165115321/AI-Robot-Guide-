# /core/services/weather_service.py
"""
Weather Service: ดึงข้อมูลสภาพอากาศจาก TMD และ OpenWeatherMap
"""

import asyncio
import logging
from typing import Dict, Optional, List
from datetime import datetime, timezone
import aiohttp

from core.config import settings

logger = logging.getLogger(__name__)


class WeatherService:
    """Service สำหรับดึงข้อมูลสภาพอากาศ"""
    
    # พิกัดจังหวัดน่าน
    NAN_LAT = 18.7756
    NAN_LON = 100.7730
    
    # OpenWeatherMap API (ฟรี 1000 calls/day)
    OPENWEATHER_URL = "https://api.openweathermap.org/data/2.5/weather"
    OPENWEATHER_FORECAST_URL = "https://api.openweathermap.org/data/2.5/forecast"
    
    # TMD API (กรมอุตุนิยมวิทยา)
    TMD_URL = "https://data.tmd.go.th/nwpapi/v1/forecast/location/daily"
    
    def __init__(self):
        # ใช้ค่าจาก settings หรือ environment variables
        self.openweather_api_key = getattr(settings, 'OPENWEATHER_API_KEY', None)
        self.tmd_api_key = getattr(settings, 'TMD_API_KEY', None)
        
    async def get_current_weather(self, lat: float = None, lon: float = None) -> Optional[Dict]:
        """
        ดึงสภาพอากาศปัจจุบัน
        Priority: TMD -> OpenWeatherMap -> OpenMeteo (Fallback)
        """
        lat = lat or self.NAN_LAT
        lon = lon or self.NAN_LON
        
        # 1. ลอง TMD (แม่นยำสุดในไทย)
        if self.tmd_api_key:
            result = await self._fetch_tmd(lat, lon)
            if result:
                return result
                
        # 2. ลอง OpenWeatherMap (ถ้ามี Key)
        if self.openweather_api_key:
            result = await self._fetch_openweather(lat, lon)
            if result:
                return result

        # 3. Fallback: OpenMeteo (ฟรี, ไม่ต้องใช้ Key)
        logger.info("ℹ️ [Weather] ใช้ OpenMeteo (Fallback)...")
        return await self._fetch_openmeteo(lat, lon)

    async def _fetch_openmeteo(self, lat: float, lon: float) -> Optional[Dict]:
        """ดึงข้อมูลจาก OpenMeteo (Free, No Key)"""
        try:
            url = "https://api.open-meteo.com/v1/forecast"
            params = {
                "latitude": lat,
                "longitude": lon,
                "current": "temperature_2m,relative_humidity_2m,apparent_temperature,is_day,weather_code,wind_speed_10m",
                "timezone": "auto"
            }
            
            async with aiohttp.ClientSession() as session:
                async with session.get(url, params=params) as response:
                    if response.status != 200:
                        logger.error(f"❌ [OpenMeteo] API error: {response.status}")
                        return None
                        
                    data = await response.json()
                    current = data.get("current", {})
                    
                    # Map WMO Weather Code to description
                    # https://open-meteo.com/en/docs
                    wmo_code = current.get("weather_code", 0)
                    description = self._map_wmo_code(wmo_code)
                    
                    result = {
                        "source": "openmeteo",
                        "location": "น่าน", # OpenMeteo ไม่คืนชื่อสถานที่
                        "temperature": current.get("temperature_2m"),
                        "feels_like": current.get("apparent_temperature"),
                        "humidity": current.get("relative_humidity_2m"),
                        "description": description,
                        "wind_speed": current.get("wind_speed_10m"),
                        "fetched_at": datetime.now(timezone.utc).isoformat()
                    }
                    
                    logger.info(f"✅ [OpenMeteo] อุณหภูมิ: {result['temperature']}°C, {result['description']}")
                    return result
                    
        except Exception as e:
            logger.error(f"❌ [OpenMeteo] ข้อผิดพลาด: {e}")
            return None

    def _map_wmo_code(self, code: int) -> str:
        """แปลง WMO Weather Code เป็นคำอธิบายไทย"""
        mapping = {
            0: "ท้องฟ้าแจ่มใส",
            1: "มีเมฆบางส่วน", 2: "มีเมฆเป็นส่วนมาก", 3: "มีเมฆมาก",
            45: "มีหมอก", 48: "มีหมอกลงจัด",
            51: "ฝนตกปรอยๆ", 53: "ฝนตกปานกลาง", 55: "ฝนตกหนัก",
            61: "ฝนตกเล็กน้อย", 63: "ฝนตกปานกลาง", 65: "ฝนตกหนัก",
            80: "ฝนตกปรอยๆ", 81: "ฝนตกปานกลาง", 82: "ฝนตกหนัก",
            95: "ฝนฟ้าคะนอง", 96: "ฝนฟ้าคะนองและมีลูกเห็บ", 99: "ฝนฟ้าคะนองรุนแรง"
        }
        return mapping.get(code, "ไม่มีข้อมูล")

    async def _fetch_openweather(self, lat: float, lon: float) -> Optional[Dict]:
        """ดึงข้อมูลจาก OpenWeatherMap"""
        try:
            params = {
                "lat": lat,
                "lon": lon,
                "appid": self.openweather_api_key,
                "units": "metric",
                "lang": "th"
            }
            
            async with aiohttp.ClientSession() as session:
                async with session.get(self.OPENWEATHER_URL, params=params) as response:
                    if response.status != 200:
                        logger.error(f"❌ [OpenWeather] API error: {response.status}")
                        return None
                        
                    data = await response.json()
                    
                    result = {
                        "source": "openweathermap",
                        "location": data.get("name", "น่าน"),
                        "temperature": data.get("main", {}).get("temp"),
                        "feels_like": data.get("main", {}).get("feels_like"),
                        "humidity": data.get("main", {}).get("humidity"),
                        "description": data.get("weather", [{}])[0].get("description", ""),
                        "icon": data.get("weather", [{}])[0].get("icon", ""),
                        "wind_speed": data.get("wind", {}).get("speed"),
                        "visibility": data.get("visibility"),
                        "clouds": data.get("clouds", {}).get("all"),
                        "fetched_at": datetime.now(timezone.utc).isoformat()
                    }
                    
                    logger.info(f"✅ [OpenWeather] อุณหภูมิ: {result['temperature']}°C, {result['description']}")
                    return result
                    
        except Exception as e:
            logger.error(f"❌ [OpenWeather] ข้อผิดพลาด: {e}")
            return None
    
    async def _fetch_tmd(self, lat: float, lon: float) -> Optional[Dict]:
        """ดึงข้อมูลจาก TMD (กรมอุตุนิยมวิทยา)"""
        try:
            headers = {
                "accept": "application/json",
                "authorization": f"Bearer {self.tmd_api_key}"
            }
            
            params = {
                "lat": lat,
                "lon": lon,
                "fields": "tc,rh,cond,ws10m",
                "duration": 1
            }
            
            async with aiohttp.ClientSession() as session:
                async with session.get(self.TMD_URL, params=params, headers=headers) as response:
                    if response.status != 200:
                        logger.error(f"❌ [TMD] API error: {response.status}")
                        return None
                        
                    data = await response.json()
                    forecasts = data.get("WeatherForecasts", [])
                    
                    if not forecasts:
                        return None
                        
                    first_forecast = forecasts[0].get("forecasts", [{}])[0]
                    
                    result = {
                        "source": "tmd",
                        "location": forecasts[0].get("location", {}).get("province", "น่าน"),
                        "temperature": first_forecast.get("data", {}).get("tc"),
                        "humidity": first_forecast.get("data", {}).get("rh"),
                        "condition": first_forecast.get("data", {}).get("cond"),
                        "wind_speed": first_forecast.get("data", {}).get("ws10m"),
                        "fetched_at": datetime.now(timezone.utc).isoformat()
                    }
                    
                    logger.info(f"✅ [TMD] อุณหภูมิ: {result['temperature']}°C")
                    return result
                    
        except Exception as e:
            logger.error(f"❌ [TMD] ข้อผิดพลาด: {e}")
            return None
    
    async def get_weather_alert_level(self, weather_data: Dict) -> int:
        """
        วิเคราะห์ระดับความรุนแรงของสภาพอากาศ
        
        Returns:
            1-5 (1=ปกติ, 5=วิกฤต)
        """
        if not weather_data:
            return 1
            
        temp = weather_data.get("temperature", 25)
        humidity = weather_data.get("humidity", 50)
        wind_speed = weather_data.get("wind_speed", 0)
        
        severity = 1
        
        # อุณหภูมิสูงมาก
        if temp > 40:
            severity = max(severity, 4)
        elif temp > 38:
            severity = max(severity, 3)
            
        # ลมแรง (m/s)
        if wind_speed > 20:
            severity = max(severity, 5)
        elif wind_speed > 15:
            severity = max(severity, 4)
        elif wind_speed > 10:
            severity = max(severity, 3)
            
        return severity


# Singleton instance
weather_service = WeatherService()
