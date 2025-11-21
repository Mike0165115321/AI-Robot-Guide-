# Back-end/core/services/navigation_service.py
import math
import logging
from typing import List, Dict, Any

class NavigationService:
    def __init__(self):
        logging.info("🗺️ [NavigationService] Initialized.")

    def calculate_distance(self, lat1, lon1, lat2, lon2) -> float:
        """คำนวณระยะทาง (Haversine Formula)"""
        if None in [lat1, lon1, lat2, lon2]: 
            return float('inf')
        
        R = 6371 # รัศมีโลก (กม.)
        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)
        a = (math.sin(dLat/2)**2 + 
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dLon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return round(R * c, 1)

    def generate_google_maps_links(self, dest_lat: float, dest_lon: float, user_lat: float = None, user_lon: float = None) -> Dict[str, str]:
        """สร้าง Link แผนที่ ทั้งแบบ Embed และ External"""
        # สูตร Embed มาตรฐาน (ไม่ต้องใช้ API Key)
        embed_url = f"https://maps.google.com/maps?q={dest_lat},{dest_lon}&z=15&output=embed"
        
        if user_lat and user_lon:
            # Link นำทางจากจุดปัจจุบัน
            external_link = f"https://www.google.com/maps/dir/?api=1&origin={user_lat},{user_lon}&destination={dest_lat},{dest_lon}&travelmode=driving"
        else:
            # Link ปักหมุดปลายทางเฉยๆ
            external_link = f"https://www.google.com/maps/search/?api=1&query={dest_lat},{dest_lon}"
            
        return {
            "embed_url": embed_url,
            "external_link": external_link
        }

    def sort_locations_by_distance(self, locations: List[dict], user_lat: float, user_lon: float) -> List[dict]:
        """เรียงลำดับสถานที่ตามระยะทาง"""
        for loc in locations:
            nav_data = loc.get("location_data", {})
            dist = self.calculate_distance(
                user_lat, user_lon, 
                nav_data.get("latitude"), nav_data.get("longitude")
            )
            loc["distance_km"] = dist
        
        # เรียงจากใกล้ไปไกล (เอา distance_km เป็นเกณฑ์)
        locations.sort(key=lambda x: x["distance_km"] if x["distance_km"] != float('inf') else 99999)
        return locations