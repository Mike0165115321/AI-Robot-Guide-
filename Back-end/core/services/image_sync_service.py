"""
Image Sync Service - สแกนไฟล์รูปภาพจาก static/images/ และบันทึกลง MongoDB
ใช้ Exact Match เท่านั้น เพื่อป้องกันการแสดงภาพผิด
"""
import os
import re
import logging
from pathlib import Path
from typing import Dict, List, Tuple
from core.database.mongodb_manager import MongoDBManager

# Pattern: filename-01.jpg, filename-02.jpg, etc.
IMAGE_PATTERN = re.compile(r'^(.+)-(\d{2})\.(jpg|jpeg|png|webp)$', re.IGNORECASE)

class ImageSyncService:
    def __init__(self, mongo_manager: MongoDBManager):
        self.mongo_manager = mongo_manager
        self.collection = mongo_manager.get_collection("image_metadata")
        self.static_images_path = Path(__file__).resolve().parent.parent.parent / "static" / "images"
    
    def scan_images(self) -> Dict[str, List[str]]:
        """
        สแกนไฟล์รูปทั้งหมดใน static/images/
        คืนค่า dict ของ prefix -> list of URLs
        """
        prefix_map: Dict[str, List[str]] = {}
        
        if not self.static_images_path.exists():
            logging.warning(f"⚠️ [ImageSync] ไม่พบโฟลเดอร์ {self.static_images_path}")
            return prefix_map
        
        for filename in os.listdir(self.static_images_path):
            match = IMAGE_PATTERN.match(filename)
            if match:
                prefix = match.group(1) + "-"  # e.g., "krua-huen-horm-"
                url = f"/static/images/{filename}"
                
                if prefix not in prefix_map:
                    prefix_map[prefix] = []
                prefix_map[prefix].append(url)
        
        # Sort URLs within each prefix (01, 02, 03...)
        for prefix in prefix_map:
            prefix_map[prefix].sort()
        
        logging.info(f"✅ [ImageSync] สแกนพบรูปภาพ {sum(len(v) for v in prefix_map.values())} รูป จาก {len(prefix_map)} prefixes")
        return prefix_map
    
    def sync_to_database(self, prefix_map: Dict[str, List[str]]) -> Tuple[int, int]:
        """
        บันทึกข้อมูลรูปภาพลง MongoDB collection 'image_metadata'
        คืนค่า (inserted_count, updated_count)
        """
        if self.collection is None:
            logging.error("❌ [ImageSync] ไม่สามารถเชื่อมต่อ MongoDB ได้")
            return (0, 0)
        
        inserted = 0
        updated = 0
        
        for prefix, urls in prefix_map.items():
            for url in urls:
                # Upsert: update if exists, insert if not
                result = self.collection.update_one(
                    {"url": url},
                    {"$set": {"url": url, "prefix": prefix}},
                    upsert=True
                )
                if result.upserted_id:
                    inserted += 1
                elif result.modified_count > 0:
                    updated += 1
        
        logging.info(f"✅ [ImageSync] บันทึกสำเร็จ - ใหม่: {inserted}, อัพเดท: {updated}")
        return (inserted, updated)
    
    def sync_images(self) -> Dict[str, any]:
        """
        สแกนและบันทึกรูปภาพทั้งหมด
        คืนค่าสรุปผลการ sync
        """
        logging.info("🔄 [ImageSync] เริ่มสแกนและซิงค์รูปภาพ...")
        
        prefix_map = self.scan_images()
        inserted, updated = self.sync_to_database(prefix_map)
        
        return {
            "success": True,
            "total_prefixes": len(prefix_map),
            "total_images": sum(len(v) for v in prefix_map.values()),
            "inserted": inserted,
            "updated": updated
        }
    
    def get_images_by_prefix(self, prefix: str) -> List[str]:
        """
        ดึง URLs ทั้งหมดที่มี prefix ตรงกัน (Exact Match)
        """
        if not prefix or self.collection is None:
            return []
        
        try:
            docs = list(self.collection.find({"prefix": prefix}))
            urls = [doc["url"] for doc in docs if "url" in doc]
            urls.sort()
            return urls
        except Exception as e:
            logging.error(f"❌ [ImageSync] เกิดข้อผิดพลาดในการดึงรูป prefix={prefix}: {e}")
            return []
