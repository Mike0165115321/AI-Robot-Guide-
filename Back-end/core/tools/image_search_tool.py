# /core/tools/image_search_tool.py (ไฟล์ใหม่)

import asyncio
import logging
from typing import List
from core.tools.google_search import google_search_instance 

class ImageSearchTool:
    def __init__(self):
        logging.info("🛠️ Image Search Tool initialized.")
        self.search_tool = google_search_instance

    async def get_image_urls(self, query: str, max_results: int = 3) -> List[str]:
        """
        ค้นหารูปภาพจาก Google แบบ Async (ผ่าน to_thread)
        และคืนค่าเป็น List ของ URL เท่านั้น
        """
        logging.info(f"🖼️ [ImageTool] Searching Google Images for: '{query}'")
        try:
            google_results = await asyncio.to_thread(
                self.search_tool.search_images,
                query=query,
                max_results=max_results
            )
            
            image_urls = [img.image_url for img in google_results if img.image_url]
            logging.info(f"✅ [ImageTool] Found {len(image_urls)} images.")
            return image_urls

        except Exception as e:
            logging.error(f"❌ [ImageTool] Error during Google Image Search: {e}", exc_info=True)
            return []

image_search_tool_instance = ImageSearchTool()