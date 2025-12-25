# /core/services/geocoding_service.py
"""
Geocoding Service: แปลงชื่อสถานที่เป็นพิกัด (Nominatim - ฟรี)
"""

import asyncio
import logging
from typing import Dict, Optional, List
import aiohttp

logger = logging.getLogger(__name__)


class GeocodingService:
    """Service สำหรับแปลงชื่อสถานที่เป็นพิกัด"""
    
    # Nominatim API (OpenStreetMap - ฟรี)
    NOMINATIM_URL = "https://nominatim.openstreetmap.org/search"
    
    # Rate limit: 1 request/second
    RATE_LIMIT_DELAY = 1.1
    
    def __init__(self):
        self._last_request_time = 0
        
    async def geocode(self, place_name: str, country: str = "Thailand") -> Optional[Dict]:
        """
        แปลงชื่อสถานที่เป็นพิกัด
        
        Args:
            place_name: ชื่อสถานที่
            country: ประเทศ (default: Thailand)
            
        Returns:
            Dict with lat, lon, display_name or None
        """
        try:
            # Rate limiting
            await self._respect_rate_limit()
            
            # เพิ่ม "น่าน" ถ้ายังไม่มี
            if "น่าน" not in place_name and "nan" not in place_name.lower():
                search_query = f"{place_name}, น่าน, {country}"
            else:
                search_query = f"{place_name}, {country}"
            
            params = {
                "q": search_query,
                "format": "json",
                "limit": 1,
                "addressdetails": 1
            }
            
            # Nominatim ต้องการ User-Agent ที่มีรายละเอียดเพียงพอ
            headers = {
                "User-Agent": "AIRobotGuideNan/1.0 (https://github.com/nan-tourism-guide; nan.ai.guide@gmail.com)",
                "Accept-Language": "th,en"
            }
            
            timeout = aiohttp.ClientTimeout(total=10)
            async with aiohttp.ClientSession(timeout=timeout) as session:
                async with session.get(self.NOMINATIM_URL, params=params, headers=headers) as response:
                    if response.status != 200:
                        logger.error(f"❌ [Geocoding] API error: {response.status}")
                        return None
                        
                    data = await response.json()
                    
                    if not data:
                        logger.warning(f"⚠️ [Geocoding] ไม่พบพิกัดสำหรับ: {place_name}")
                        return None
                        
                    result = data[0]
                    
                    geocode_result = {
                        "lat": float(result.get("lat", 0)),
                        "lon": float(result.get("lon", 0)),
                        "display_name": result.get("display_name", ""),
                        "place_name": place_name,
                        "address": result.get("address", {}),
                        "osm_type": result.get("osm_type", ""),
                        "osm_id": result.get("osm_id", ""),
                        "importance": result.get("importance", 0)
                    }
                    
                    logger.info(f"📍 [Geocoding] {place_name} -> ({geocode_result['lat']}, {geocode_result['lon']})")
                    return geocode_result
                    
        except Exception as e:
            logger.error(f"❌ [Geocoding] ข้อผิดพลาด: {e}")
            return None
    
    async def reverse_geocode(self, lat: float, lon: float) -> Optional[Dict]:
        """
        แปลงพิกัดเป็นชื่อสถานที่
        
        Args:
            lat: ละติจูด
            lon: ลองจิจูด
            
        Returns:
            Dict with display_name, address or None
        """
        try:
            await self._respect_rate_limit()
            
            url = "https://nominatim.openstreetmap.org/reverse"
            params = {
                "lat": lat,
                "lon": lon,
                "format": "json",
                "addressdetails": 1
            }
            
            headers = {
                "User-Agent": "AIRobotGuideNan/1.0 (contact@example.com)"
            }
            
            async with aiohttp.ClientSession() as session:
                async with session.get(url, params=params, headers=headers) as response:
                    if response.status != 200:
                        return None
                        
                    data = await response.json()
                    
                    return {
                        "display_name": data.get("display_name", ""),
                        "address": data.get("address", {}),
                        "lat": lat,
                        "lon": lon
                    }
                    
        except Exception as e:
            logger.error(f"❌ [ReverseGeocode] ข้อผิดพลาด: {e}")
            return None
    
    async def geocode_batch(self, place_names: List[str]) -> List[Dict]:
        """
        แปลงหลายสถานที่พร้อมกัน (มี rate limit)
        
        Args:
            place_names: รายการชื่อสถานที่
            
        Returns:
            List of geocode results
        """
        results = []
        
        for place in place_names:
            result = await self.geocode(place)
            if result:
                results.append(result)
            # Rate limit is handled automatically
            
        return results
    
    async def _respect_rate_limit(self):
        """รอให้ครบ rate limit ก่อนส่ง request ถัดไป"""
        import time
        
        current_time = time.time()
        time_since_last = current_time - self._last_request_time
        
        if time_since_last < self.RATE_LIMIT_DELAY:
            await asyncio.sleep(self.RATE_LIMIT_DELAY - time_since_last)
            
        self._last_request_time = time.time()


# Singleton instance
geocoding_service = GeocodingService()
