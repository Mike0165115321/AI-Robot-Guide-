import math
import logging
import asyncio
from typing import List, Dict, Any, Optional
from bson import ObjectId
from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from .prompt_engine import PromptEngine
from core.services.knowledge_gap_service import KnowledgeGapService

# 🆕 Threshold สำหรับ Semantic Navigation Search
SEMANTIC_NAV_HIGH_CONFIDENCE = 0.70   # ใช้เลยไม่ต้องถาม
SEMANTIC_NAV_LOW_CONFIDENCE = 0.50    # ถามยืนยันก่อน

class NavigationService:
    def __init__(
        self, 
        mongo_manager: MongoDBManager, 
        prompt_engine: PromptEngine, 
        qdrant_manager: QdrantManager = None,
        knowledge_gap_service: KnowledgeGapService = None
    ):
        self.mongo_manager = mongo_manager
        self.prompt_engine = prompt_engine
        self.qdrant_manager = qdrant_manager
        self.knowledge_gap_service = knowledge_gap_service
        logging.info("🗺️ [NavigationService] Initialized (Hybrid Search Enabled).")

    def _clean_navigation_entity(self, text: str) -> str:
        """ลบคำกริยานำทางออกจากชื่อสถานที่ เช่น 'ไป วัด...', 'นำทางไป...'"""
        prefixes = [
            "นำทางไปยัง", "นำทางไปที่", "นำทางไป", 
            "ขอเส้นทางไปยัง", "ขอเส้นทางไปที่", "ขอเส้นทางไป",
            "พาไปที่", "พาไป", "ไปที่", "ไป",
            "อยากไป", "อยากดู", "อยากรู้เรื่อง"  # 🆕 เพิ่ม prefix ที่พบบ่อย
        ]
        # Sort by length descending to match longest prefix first
        prefixes.sort(key=len, reverse=True)
        
        text = text.strip()
        for prefix in prefixes:
            if text.startswith(prefix):
                # Remove prefix and strip again
                text = text[len(prefix):].strip()
                break # Only remove one prefix
        
        # 🆕 ลบคำถาม/คำท้ายที่ติดมา เช่น "รู้จักมั้ยครับ", "อยู่ไหนคะ"
        suffixes = [
            "รู้จักมั้ยครับ", "รู้จักไหมครับ", "รู้จักมั้ยคะ", "รู้จักไหมคะ",
            "รู้จักมั้ย", "รู้จักไหม", "รู้จักป่าว",
            "อยู่ไหนครับ", "อยู่ไหนคะ", "อยู่ไหน", "อยู่ที่ไหน",
            "ไปยังไงครับ", "ไปยังไงคะ", "ไปยังไง",
            "เป็นยังไงครับ", "เป็นยังไงคะ", "เป็นยังไง",
            "ดีไหมครับ", "ดีไหมคะ", "ดีไหม", "ดีมั้ย",
            "น่าไปไหม", "น่าไปมั้ย",
            "ครับ", "ค่ะ", "คะ", "นะคะ", "นะครับ"
        ]
        suffixes.sort(key=len, reverse=True)
        
        for suffix in suffixes:
            if text.endswith(suffix):
                text = text[:-len(suffix)].strip()
                break  # Only remove one suffix
        
        return text

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

    async def handle_get_directions(self, entity_slug: str, user_lat: float = None, user_lon: float = None, skip_cleaning: bool = False) -> dict:
        # 1. Clean up the entity name (remove common verbs) if needed
        if skip_cleaning:
             clean_slug = entity_slug.strip()
             logging.info(f"🗺️  [V-Maps] Skipping cleaning (LLM Trusted): '{clean_slug}'")
        else:
             clean_slug = self._clean_navigation_entity(entity_slug)
             logging.info(f"🗺️  [V-Maps] Handling Directions for: '{entity_slug}' -> Cleaned: '{clean_slug}'")
        
        # 2. Search by Slug (Exact)
        doc = await asyncio.to_thread(self.mongo_manager.get_location_by_slug, clean_slug)
        
        # 3. If not found, Search by Title (Fuzzy)
        if not doc:
            logging.info(f"[V-Maps] Slug not found. Searching by title: '{clean_slug}'")
            doc = await asyncio.to_thread(self.mongo_manager.get_location_by_title, clean_slug)

        if not doc or not doc.get("location_data"):
            # Fallback: Try searching original slug just in case cleaning removed too much
            if clean_slug != entity_slug:
                 doc = await asyncio.to_thread(self.mongo_manager.get_location_by_title, entity_slug)
            
            # 🆕 [Hybrid Navigation] Qdrant Semantic Search Fallback!
            if (not doc or not doc.get("location_data")) and self.qdrant_manager:
                logging.info(f"🔍 [Hybrid Nav] MongoDB not found, trying Qdrant semantic search for: '{clean_slug}'")
                
                try:
                    # ค้นหาแบบ Semantic (ไม่ filter เพราะ payload ไม่มี doc_type)
                    qdrant_results = await self.qdrant_manager.search_similar(
                        query_text=f"{clean_slug} จังหวัดน่าน",
                        top_k=3
                    )
                    
                    if qdrant_results:
                        top_result = qdrant_results[0]
                        top_score = top_result.score if hasattr(top_result, 'score') else 0.0
                        payload = top_result.payload if hasattr(top_result, 'payload') else {}
                        
                        # 🆕 ดึง title จาก payload หรือ parse จาก text_content
                        matched_title = payload.get("title")
                        if not matched_title:
                            # Parse from text_content: "หัวข้อ: วัดพระธาตุแช่แห้ง..."
                            text_content = payload.get("text_content", "")
                            if "หัวข้อ:" in text_content:
                                matched_title = text_content.split("หัวข้อ:")[1].split("(")[0].strip()
                            else:
                                matched_title = text_content[:50].strip() or "Unknown"
                        
                        mongo_id = payload.get("mongo_id")
                        
                        logging.info(f"🎯 [Hybrid Nav] Qdrant found: '{matched_title}' (score: {top_score:.4f}, mongo_id: {mongo_id})")
                        
                        # ✅ High Confidence: ใช้เลย!
                        if top_score >= SEMANTIC_NAV_HIGH_CONFIDENCE and mongo_id:
                            logging.info(f"✅ [Hybrid Nav] High confidence match! Fetching from MongoDB: '{matched_title}'")
                            # Fetch full doc from MongoDB using mongo_id
                            fetched_doc = await asyncio.to_thread(
                                lambda mid=mongo_id: self.mongo_manager.get_collection("nan_locations").find_one({"_id": ObjectId(mid)})
                            )
                            if fetched_doc and fetched_doc.get("location_data"):
                                # ✅ SUCCESS! ใช้ doc ที่ fetch มา
                                doc = fetched_doc
                                doc["_semantic_match"] = True
                                doc["_original_query"] = clean_slug
                                logging.info(f"✅ [Hybrid Nav] Successfully fetched: '{doc.get('title')}' with location_data!")
                        
                        # ⚠️ Medium Confidence: ถามยืนยัน
                        elif top_score >= SEMANTIC_NAV_LOW_CONFIDENCE and mongo_id:
                            logging.info(f"⚠️ [Hybrid Nav] Medium confidence - asking confirmation for: '{matched_title}'")
                            return {
                                "answer": f"คุณหมายถึง **{matched_title}** หรือเปล่าคะ? 🤔\n\nกดเพื่อยืนยันนำทางไปที่นี่ค่ะ",
                                "action": "CONFIRM_NAVIGATION",
                                "action_payload": {
                                    "suggested_entity": matched_title,
                                    "mongo_id": mongo_id,
                                    "original_query": clean_slug,
                                    "confidence": round(top_score, 4)
                                },
                                "sources": [], "image_url": None, "image_gallery": []
                            }
                        else:
                            logging.info(f"❌ [Hybrid Nav] Low confidence ({top_score:.4f}) - not using")
                            
                except Exception as e:
                    logging.error(f"❌ [Hybrid Nav] Qdrant search failed: {e}")
            
            # ❌ ไม่เจอทั้ง MongoDB และ Qdrant
            if not doc or not doc.get("location_data"):
                not_found_answer = f"ขออภัยค่ะ ไม่พบพิกัดของ **{clean_slug}** ในระบบ ลองระบุชื่อสถานที่อีกครั้งนะคะ"
                
                # 🧠 [Self-Correcting RAG] Log navigation failures to Knowledge Gaps!
                if self.knowledge_gap_service:
                    await self.knowledge_gap_service.log_unanswered(
                        query=f"นำทางไป {clean_slug}",
                        score=0.0,  # Not found = 0 score
                        ai_response=not_found_answer,
                        context=f"[NAVIGATION] ไม่พบสถานที่: {entity_slug} -> cleaned: {clean_slug}"
                    )
                    logging.info(f"🧠 [Knowledge Gap] Logged NOT FOUND navigation: '{clean_slug}'")
                
                return {
                    "answer": not_found_answer, 
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