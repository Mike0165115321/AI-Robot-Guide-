"""
Image Sync Service - สแกนไฟล์รูปภาพจาก static/images/ และบันทึกลง MongoDB
ใช้ Exact Match เท่านั้น เพื่อป้องกันการแสดงภาพผิด
"""
import logging
import random
import re
import asyncio
from typing import List, Dict, Optional
from core.database.mongodb_manager import MongoDBManager
from core.config import settings
from core.tools.image_search_tool import image_search_tool_instance
from core.services.image_sync_service import ImageSyncService

class ImageService:
    def __init__(self, mongo_manager: MongoDBManager):
        self.mongo_manager = mongo_manager
        self.collection = self.mongo_manager.get_collection("image_metadata")
        self.prefixed_image_map: Dict[str, List[str]] = {}
        self.all_image_files: List[str] = []
        
        # Initialize sync service and run sync at startup
        self.sync_service = ImageSyncService(mongo_manager)
        self._run_initial_sync()
        
        # Load cache after sync
        self.refresh_cache()

    def _run_initial_sync(self):
        """สแกนและซิงค์รูปภาพตอน startup"""
        try:
            result = self.sync_service.sync_images()
            logging.info(f"✅ [ImageService] Startup sync สำเร็จ: {result['total_images']} รูป")
        except Exception as e:
            logging.warning(f"⚠️ [ImageService] Startup sync ล้มเหลว: {e}")

    def refresh_cache(self):
        """Loads all image metadata from MongoDB into memory."""
        if self.collection is None:
            logging.warning("⚠️ ImageService: ไม่พบ collection 'image_metadata'")
            return

        try:
            all_docs = list(self.collection.find({}))
            self.prefixed_image_map = {}
            self.all_image_files = []

            for doc in all_docs:
                url = doc.get("url")
                prefix = doc.get("prefix")
                if url:
                    self.all_image_files.append(url)
                    if prefix:
                        if prefix not in self.prefixed_image_map:
                            self.prefixed_image_map[prefix] = []
                        self.prefixed_image_map[prefix].append(url)
            
            logging.info(f"✅ ImageService: โหลดรูปภาพ {len(self.all_image_files)} รูป พร้อม prefix {len(self.prefixed_image_map)} รายการ")
        except Exception as e:
            logging.error(f"❌ ImageService: เกิดข้อผิดพลาดในการรีเฟรช cache: {e}")

    def find_all_images_by_prefix(self, prefix: str) -> List[str]:
        """ค้นหารูปภาพตาม prefix (Exact Match เท่านั้น)"""
        if not prefix: 
            return []
        
        # Exact match from cache
        cached_files = self.prefixed_image_map.get(prefix, [])
        if cached_files:
            matching_files = list(cached_files)
            matching_files.sort()  # Sort instead of shuffle for consistency
            return matching_files
        
        return []

    def get_location_images(self, doc: Dict[str, any]) -> List[str]:
        """
        Exact Match เท่านั้น - ไม่ใช้ fuzzy matching
        ลำดับการค้นหา:
        1. image_prefix จาก metadata
        2. slug + "-" เป็น prefix
        ถ้าไม่เจอ คืนค่า [] (แสดง placeholder)
        """
        # 1. Try Image Prefix from metadata (exact match)
        prefix = doc.get("metadata", {}).get("image_prefix")
        if prefix:
            imgs = self.find_all_images_by_prefix(prefix)
            if imgs:
                return imgs
            
        # 2. Try slug as prefix (exact match)
        slug = doc.get("slug")
        if slug:
            slug_prefix = slug + "-"
            imgs = self.find_all_images_by_prefix(slug_prefix)
            if imgs:
                return imgs
        
        # ไม่พบรูป - คืนค่าว่างให้ frontend แสดง placeholder
        return []

    def find_random_image(self) -> str | None:
        if not self.all_image_files: 
            return None
        return random.choice(self.all_image_files)

    def construct_full_image_url(self, image_path: str | None) -> str | None:
        if not image_path: 
            return None
        if image_path.startswith(('http://', 'https://')):
            return image_path
        if image_path.startswith('/'):
            host = settings.API_HOST
            if host == "0.0.0.0":
                host = "127.0.0.1" # Browser cannot connect to 0.0.0.0
            return f"http://{host}:{settings.API_PORT}{image_path}"
        return image_path


    async def inject_images_into_text(self, text: str) -> str:
        if not text: return ""
        pattern = r"\{\{IMAGE:\s*(.*?)\}\}"
        matches = re.findall(pattern, text)
        
        for keyword in matches:
            image_url = None
            safe_keyword = keyword.replace(" ", "-").lower()
            
            # 1. Search in cache (Exact or Partial match on keys)
            image_found = False
            for prefix, paths in self.prefixed_image_map.items():
                if (safe_keyword in prefix.lower() or prefix.lower().replace("-", " ") in keyword.lower()) and paths:
                    image_url = random.choice(paths)
                    image_found = True
                    break
            
            # 2. If not found, try to look up in MongoDB (Title -> DB Image URLs -> Slug -> Local Image)
            if not image_found:
                try:
                    # Try to find location by title (Thai name)
                    doc = await asyncio.to_thread(self.mongo_manager.get_location_by_title, keyword)
                    if doc:
                        # 🆕 PRIORITY: Check if DB document has explicit 'image_urls' (Admin overrides)
                        db_image_urls = doc.get("image_urls", [])
                        if db_image_urls and isinstance(db_image_urls, list) and len(db_image_urls) > 0:
                             raw_url = random.choice(db_image_urls)
                             image_url = self.construct_full_image_url(str(raw_url))
                             image_found = True
                             logging.info(f"✅ [ImageService] Found explicit DB URL for '{keyword}': {image_url}")

                        if not image_found:
                            # If no explicit URLs, try to find images using its slug
                            slug = doc.get("slug")
                            image_prefix = doc.get("metadata", {}).get("image_prefix")
                            
                            target_prefix = image_prefix or (f"{slug}-" if slug else None)
                            
                            if target_prefix:
                                # Try to find in cache using the looked-up prefix
                                for prefix, paths in self.prefixed_image_map.items():
                                    if prefix.startswith(target_prefix) or target_prefix.startswith(prefix):
                                         if paths:
                                             image_url = random.choice(paths)
                                             image_found = True
                                             logging.info(f"✅ [ImageService] Mapped '{keyword}' -> Slug/Prefix '{target_prefix}' -> Found Local Image")
                                             break
                except Exception as e:
                    logging.warning(f"⚠️ [ImageService] DB Lookup failed for '{keyword}': {e}")

            # 3. Fallback to Google Search
            if not image_url and not image_found:
                try:
                    search_q = f"{keyword} จังหวัดน่าน"
                    google_urls = await image_search_tool_instance.get_image_urls(search_q, max_results=1)
                    if google_urls:
                        image_url = google_urls[0]
                except Exception as e:
                    logging.warning(f"การค้นหารูปภาพเพื่อแทรกเนื้อหาล้มเหลวสำหรับ {keyword}: {e}")
            
            if image_url:
                full_url = self.construct_full_image_url(image_url)
                replacement = f"\n\n![{keyword}]({full_url})\n\n"
            else:
                replacement = ""
            
            text = re.sub(r"\{\{IMAGE:\s*" + re.escape(keyword) + r"\}\}", replacement, text, count=1)
        
        return text
