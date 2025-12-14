import logging
import random
import re
import asyncio
from typing import List, Dict, Optional
from core.database.mongodb_manager import MongoDBManager
from core.config import settings
from core.tools.image_search_tool import image_search_tool_instance

class ImageService:
    def __init__(self, mongo_manager: MongoDBManager):
        self.mongo_manager = mongo_manager
        self.collection = self.mongo_manager.get_collection("image_metadata")
        self.prefixed_image_map: Dict[str, List[str]] = {}
        self.all_image_files: List[str] = []
        
        # Load cache on init
        self.refresh_cache()

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
        if not prefix: return []
        # Try cache first
        cached_files = self.prefixed_image_map.get(prefix, [])
        if cached_files:
            matching_files = list(cached_files)
            random.shuffle(matching_files)
            return matching_files
        
        # Fallback to DB query if not in cache (though cache should have everything)
        # This might happen if new images are added without refresh
        try:
            docs = list(self.collection.find({"prefix": prefix}))
            urls = [d["url"] for d in docs if "url" in d]
            random.shuffle(urls)
            return urls
        except Exception:
            return []

    def get_location_images(self, doc: Dict[str, any]) -> List[str]:
        """
        Determines the best image URLs for a location document.
        Mirrors logic in frontend utils.js (Prefix -> Slug)
        With fuzzy matching for partial prefix matches.
        """
        # 1. Try Image Prefix (exact match)
        prefix = doc.get("metadata", {}).get("image_prefix")
        imgs = self.find_all_images_by_prefix(prefix)
        if imgs:
            return imgs
            
        # 2. Fallback: Try using slug as prefix
        slug = doc.get("slug")
        if slug:
            slug_prefix = slug + "-"
            imgs = self.find_all_images_by_prefix(slug_prefix)
            if imgs:
                return imgs
            
            # 3. Fuzzy match: look for any prefix STARTING with slug
            for cached_prefix in self.prefixed_image_map.keys():
                if cached_prefix.startswith(slug + "-"):
                    return self.prefixed_image_map[cached_prefix]
            
            # 4. ✅ NEW: Static file fallback - check if physical file exists
            import os
            static_path = os.path.join(os.path.dirname(__file__), "..", "..", "static", "images")
            static_path = os.path.abspath(static_path)
            
            for i in range(1, 6):  # Check up to 5 images
                filename = f"{slug}-0{i}.jpg"
                full_path = os.path.join(static_path, filename)
                if os.path.exists(full_path):
                    # Return static URL if file exists
                    return [f"/static/images/{filename}"]
                    
        return []

    def find_random_image(self) -> str | None:
        if not self.all_image_files: return None
        return random.choice(self.all_image_files)

    def construct_full_image_url(self, image_path: str | None) -> str | None:
        if not image_path: return None
        if image_path.startswith(('http://', 'https://')):
            return image_path
        if image_path.startswith('/'):
            return f"http://{settings.API_HOST}:{settings.API_PORT}{image_path}"
        return image_path

    async def inject_images_into_text(self, text: str) -> str:
        if not text: return ""
        pattern = r"\{\{IMAGE:\s*(.*?)\}\}"
        matches = re.findall(pattern, text)
        
        for keyword in matches:
            image_url = None
            safe_keyword = keyword.replace(" ", "-").lower()
            
            # Search in cache
            for prefix, paths in self.prefixed_image_map.items():
                if (safe_keyword in prefix.lower() or prefix.lower().replace("-", " ") in keyword.lower()) and paths:
                    image_url = random.choice(paths)
                    break
            
            # Fallback to Google Search
            if not image_url:
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
