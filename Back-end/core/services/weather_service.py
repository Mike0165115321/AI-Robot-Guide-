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
        
        Args:
            lat: ละติจูด (default: น่าน)
            lon: ลองจิจูด (default: น่าน)
            
        Returns:
            Weather data dict or None
        """
        lat = lat or self.NAN_LAT
        lon = lon or self.NAN_LON
        
        # ลอง TMD ก่อน
        if self.tmd_api_key:
            result = await self._fetch_tmd(lat, lon)
            if result:
                return result
                
        # Fallback ไป OpenWeatherMap
        if self.openweather_api_key:
            return await self._fetch_openweather(lat, lon)
            
        logger.warning("⚠️ [Weather] ไม่มี API Key - กรุณาตั้งค่า TMD_API_KEY หรือ OPENWEATHER_API_KEY")
        return None
    
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
