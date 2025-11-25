import math
import logging
import asyncio
from typing import List, Dict, Any, Optional
from core.database.mongodb_manager import MongoDBManager
from .prompt_engine import PromptEngine

class NavigationService:
    def __init__(self, mongo_manager: MongoDBManager, prompt_engine: PromptEngine):
        self.mongo_manager = mongo_manager
        self.prompt_engine = prompt_engine
        logging.info("🗺️ [NavigationService] Initialized.")

    def calculate_distance(self, lat1, lon1, lat2, lon2) -> float:
        """คำนวณระยะทาง (Haversine Formula)"""
        if None in [lat1, lon1, lat2, lon2]: 
            return None
        
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
        locations.sort(key=lambda x: x["distance_km"] if x["distance_km"] is not None else 99999)
        return locations

    async def handle_get_directions(self, entity_slug: str, user_lat: float = None, user_lon: float = None) -> dict:
        logging.info(f"🗺️  [V-Maps] Handling Directions for: '{entity_slug}'")
        
        doc = await asyncio.to_thread(self.mongo_manager.get_location_by_slug, entity_slug)
        if not doc:
            logging.info(f"[V-Maps] Slug not found. Searching by title: '{entity_slug}'")
            doc = await asyncio.to_thread(self.mongo_manager.get_location_by_title, entity_slug)

        if not doc or not doc.get("location_data"):
            return {
                "answer": f"ขออภัยค่ะ ไม่พบพิกัดของ **{entity_slug}** ในระบบ ลองระบุชื่อให้ชัดเจนขึ้นอีกนิดนะคะ", 
                "action": None, "sources": [], "image_url": None
            }

        nav_data = doc["location_data"]
        dest_name = doc.get("title", "ปลายทาง")
        
        links = self.generate_google_maps_links(
            nav_data.get("latitude"), nav_data.get("longitude"),
            user_lat, user_lon
        )

        answer_text = self.prompt_engine.build_navigation_prompt(dest_name)

        return {
            "answer": answer_text,
            "action": "SHOW_MAP_EMBED",
            "action_payload": {
                "embed_url": links["embed_url"],
                "destination_name": dest_name,
                "external_link": links["external_link"] 
            },
            "image_url": None, "image_gallery": [], "sources": []
        }