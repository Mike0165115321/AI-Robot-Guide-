
from pymongo import MongoClient
from bson import ObjectId
from bson.errors import InvalidId
import logging
import asyncio
import re
import difflib
from core.config import settings
from typing import List, Dict, Any, Optional
from datetime import datetime # 🚀 [เพิ่ม]

class MongoDBManager:
    def __init__(self):
        try:
            self.client = MongoClient(
                settings.MONGO_URI, 
                serverSelectionTimeoutMS=5000,
                connectTimeoutMS=5000,
                socketTimeoutMS=5000
            )
            self.db = self.client[settings.MONGO_DATABASE_NAME]
            self.client.server_info()
            print("✅ การเชื่อมต่อ MongoDB สำเร็จ")
        except Exception as e:
            print(f"❌ เชื่อมต่อ MongoDB ล้มเหลว: {e}")
            self.client = None
            self.db = None

    def get_collection(self, collection_name: str):
        if self.db is not None:
            return self.db[collection_name]
        return None

    def get_locations_by_ids(self, ids: list) -> list:
        collection = self.get_collection("nan_locations")
        if collection is None:
            return []
        
        try:
            from bson.objectid import ObjectId
            object_ids = [ObjectId(i) for i in ids if ObjectId.is_valid(i)]
            return list(collection.find({"_id": {"$in": object_ids}}))
        except Exception as e:
            print(f"❌ Error fetching locations by IDs: {e}")
            return []

    def get_locations_by_titles(self, titles: list) -> list:
        """
        ดึงข้อมูลสถานที่หลายแห่งพร้อมกันโดยใช้ชื่อ (title)
        """
        collection = self.get_collection("nan_locations")
        if collection is None:
            return []
        
        try:
            # ใช้ $in operator เพื่อค้นหาทีเดียว
            return list(collection.find({"title": {"$in": titles}}))
        except Exception as e:
            print(f"❌ Error fetching locations by titles: {e}")
            return []
        
    def add_location(self, location_data: dict, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                result = collection.insert_one(location_data)
                print(f"📄 เพิ่มสถานที่ใหม่ด้วยรหัส: {result.inserted_id}")
                return str(result.inserted_id)
            except Exception as e:
                print(f"❌ เกิดข้อผิดพลาดในการเพิ่มสถานที่: {e}")
                return None
        return None
    
    def get_location_by_id(self, mongo_id: str, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try: return collection.find_one({"_id": ObjectId(mongo_id)})
            except InvalidId:
                print(f"❌ รูปแบบรหัส MongoDB ไม่ถูกต้อง: '{mongo_id}'")
                return None
            except Exception as e:
                print(f"❌ เกิดข้อผิดพลาดในการค้นหาเอกสารด้วยรหัส '{mongo_id}': {e}")
                return None
        return None

    def get_location_by_slug(self, slug: str, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try: return collection.find_one({"slug": slug})
            except Exception as e:
                print(f"❌ เกิดข้อผิดพลาดในการค้นหาเอกสารด้วย Slug '{slug}': {e}") # ใช้ Slug ทับศัพท์เพราะเป็น term เฉพาะทาง
                return None
        return None

    def get_location_by_title(self, title: str, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                # 1. Try Exact Regex Match first
                query = {"title": {"$regex": re.escape(title), "$options": "i"}} 
                result = collection.find_one(query)
                if result: return result
                
                # 2. Try Fuzzy Match (fallback for typos)
                return self._get_location_by_fuzzy_title(title, collection)
            except Exception as e:
                print(f"❌ เกิดข้อผิดพลาดในการค้นหาเอกสารด้วยชื่อเรื่อง '{title}': {e}")
                return None
        return None

    def _get_location_by_fuzzy_title(self, query_title: str, collection):
        """
        Helper for fuzzy matching titles using Prefix Logic to handle long titles with suffixes.
        """
        try:
            # Fetch all titles (optimized projection)
            all_docs = list(collection.find({}, {"title": 1}))
            
            best_match = None
            best_ratio = 0.0
            cutoff = 0.6 if len(query_title) > 5 else 0.8
            
            for doc in all_docs:
                title = doc.get("title", "")
                # Compare against prefix of title (with some slack e.g. +2 chars)
                # This helps when Target is "Name (Description)" and Query is "Name" (or typo of Name)
                compare_len = len(query_title) + 2
                title_prefix = title[:compare_len]
                
                ratio = difflib.SequenceMatcher(None, query_title, title_prefix).ratio()
                
                if ratio > best_ratio and ratio > cutoff:
                    best_ratio = ratio
                    best_match = title

            if best_match:
                print(f"🎯 [Fuzzy Match] '{query_title}' matched with '{best_match}' (Ratio: {best_ratio:.2f})")
                return collection.find_one({"title": best_match})
                
        except Exception as e:
             print(f"⚠️ [Fuzzy] Error during fuzzy search: {e}")
        return None

    def get_all_locations(self, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try: return list(collection.find({}))
            except Exception as e:
                print(f"❌ เกิดข้อผิดพลาดในการดึงข้อมูลสถานที่ทั้งหมด: {e}")
                return []
        return []

    def get_locations_paginated(self, skip: int = 0, limit: int = 10, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                total_count = collection.count_documents({})
                cursor = collection.find({}).skip(skip).limit(limit)
                items = list(cursor)
                return items, total_count
            except Exception as e:
                print(f"❌ เกิดข้อผิดพลาดในการดึงข้อมูลสถานที่แบบแบ่งหน้า: {e}")
                return [], 0
        return [], 0

    def update_location(self, mongo_id: str, new_data: dict, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                result = collection.update_one({"_id": ObjectId(mongo_id)}, {"$set": new_data})
                return result.modified_count
            except InvalidId:
                print(f"❌ ไม่สามารถอัปเดตได้: รูปแบบรหัส MongoDB ไม่ถูกต้อง: '{mongo_id}'")
                return 0
            except Exception as e:
                print(f"❌ เกิดข้อผิดพลาดในการอัปเดตเอกสารด้วยรหัส: {e}")
                return 0
        return 0

    def update_location_by_slug(self, slug: str, new_data: dict, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                result = collection.update_one({"slug": slug}, {"$set": new_data})
                return result.modified_count
            except Exception as e:
                print(f"❌ เกิดข้อผิดพลาดในการอัปเดตเอกสารด้วย Slug '{slug}': {e}")
                return 0
        return 0
    def delete_location(self, mongo_id: str, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                result = collection.delete_one({"_id": ObjectId(mongo_id)})
                return result.deleted_count
            except InvalidId:
                print(f"❌ ไม่สามารถลบได้: รูปแบบรหัส MongoDB ไม่ถูกต้อง: '{mongo_id}'")
                return 0
            except Exception as e:
                print(f"❌ เกิดข้อผิดพลาดในการลบเอกสารด้วยรหัส: {e}")
                return 0
        return 0

    def delete_location_by_slug(self, slug: str, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                result = collection.delete_one({"slug": slug})
                return result.deleted_count
            except Exception as e:
                print(f"❌ เกิดข้อผิดพลาดในการลบเอกสารด้วย Slug '{slug}': {e}")
                return 0
        return 0
    
    def delete_locations_by_sheet_id(self, sheet_id: str, collection_name: str = "nan_locations") -> int:
        """
        ลบข้อมูลทั้งหมดที่ sync มาจาก Sheet ที่ระบุ
        ใช้สำหรับการยกเลิกการเชื่อมต่อและลบข้อมูลพร้อมกัน
        
        Args:
            sheet_id: ID ของ Google Sheet
            collection_name: ชื่อ collection
            
        Returns:
            จำนวนเอกสารที่ลบ
        """
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                # Find documents with matching sheet_id in metadata
                result = collection.delete_many({
                    "metadata.sheet_id": sheet_id,
                    "metadata.synced_from": "google_sheets"
                })
                print(f"✅ ลบข้อมูลจาก Sheet '{sheet_id}' จำนวน {result.deleted_count} รายการ")
                return result.deleted_count
            except Exception as e:
                print(f"❌ เกิดข้อผิดพลาดในการลบข้อมูลจาก Sheet '{sheet_id}': {e}")
                return 0
        return 0

    def log_analytics_event(self, log_data: dict, collection_name: str = "analytics_logs"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                collection.insert_one(log_data)
                print(f"✅ บันทึกเหตุการณ์ Analytics (หัวข้อ: {log_data.get('interest_topic')}, ที่มา: {log_data.get('user_origin')})")
            except Exception as e:
                print(f"❌ เกิดข้อผิดพลาดในการบันทึกเหตุการณ์ Analytics: {e}")
        else:
            print("❌ เกิดข้อผิดพลาดในการบันทึก Analytics: ไม่พบ Collection 'analytics_logs'")
            
    def get_distinct_categories(self, collection_name: str = "nan_locations") -> List[str]:
        """
        (Sync Function) ดึงรายชื่อ 'category' ทั้งหมดที่ไม่ซ้ำกัน
        """
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                print("🧠 [DB] กำลังค้นหาหมวดหมู่ทั้งหมด...")
                categories = collection.distinct("category")
                
                # กรองค่าที่เป็น None หรือค่าว่างออก
                valid_categories = [cat for cat in categories if cat]
                
                print(f"✅ [DB] พบ {len(valid_categories)} หมวดหมู่")
                return valid_categories
            except Exception as e:
                print(f"❌ เกิดข้อผิดพลาดในการดึงข้อมูลหมวดหมู่: {e}")
                return []
        return []

    def get_analytics_summary(self, days: int = 30) -> dict:
        """
        ดึงสรุปข้อมูล Analytics ย้อนหลังตามจำนวนวัน (default 30 วัน)
        คืนค่าเป็น dict ที่มี key: origin_stats, province_stats, interest_stats, total_conversations
        """
        collection = self.get_collection("analytics_logs")
        if collection is None:
            return {"origin_stats": [], "province_stats": [], "interest_stats": [], "total_conversations": 0}

        try:
            # ต้อง import datetime ที่นี่เพื่อให้มั่นใจว่าใช้งานได้
            from datetime import datetime, timedelta, timezone
            
            # 1. กำหนดช่วงเวลา (ย้อนหลัง X วัน)
            cutoff_date = datetime.now(timezone.utc) - timedelta(days=days)
            
            # Filter พื้นฐาน: เอาเฉพาะข้อมูลที่ใหม่กว่า cutoff_date
            match_stage = {"$match": {"timestamp": {"$gte": cutoff_date}}}

            # 2. Pipeline สำหรับหา User Origin (นักท่องเที่ยวมาจากไหน - ประเทศ)
            origin_pipeline = [
                match_stage,
                {"$match": {"user_origin": {"$ne": None}}},  # ไม่เอาค่า Null
                {"$group": {"_id": "$user_origin", "count": {"$sum": 1}}}, # จัดกลุ่มและนับ
                {"$sort": {"count": -1}}, # เรียงจากมากไปน้อย
                {"$limit": 10} # เอาแค่ Top 10 อันดับแรก
            ]
            
            # 3. Pipeline สำหรับหา User Province (มาจากจังหวัดไหน - สำหรับคนไทย)
            province_pipeline = [
                match_stage,
                {"$match": {"user_province": {"$ne": None}}},  # ไม่เอาค่า Null
                {"$group": {"_id": "$user_province", "count": {"$sum": 1}}},
                {"$sort": {"count": -1}},
                {"$limit": 15}  # Top 15 จังหวัด
            ]
            
            # 4. Pipeline สำหรับหา Interest Topic (เขาสนใจเรื่องอะไร - หมวดหมู่)
            interest_pipeline = [
                match_stage,
                {"$match": {"interest_topic": {"$ne": None}}}, # ไม่เอาค่า Null
                {"$group": {"_id": "$interest_topic", "count": {"$sum": 1}}},
                {"$sort": {"count": -1}},
                {"$limit": 10}
            ]
            
            # 5. Pipeline สำหรับหา Top Locations (สถานที่ยอดฮิต)
            location_pipeline = [
                match_stage,
                {"$match": {"location_title": {"$ne": None}}},  # ไม่เอาค่า Null
                {"$group": {"_id": "$location_title", "count": {"$sum": 1}}},
                {"$sort": {"count": -1}},
                {"$limit": 10}
            ]

            # 6. นับจำนวนข้อความทั้งหมดในช่วงเวลา (นับจาก analytics_logs)
            total_count = collection.count_documents({"timestamp": {"$gte": cutoff_date}})
            # 5. [NEW] Pipeline for Feedback (Like/Dislike)
            feedback_pipeline = [
                match_stage,
                {"$group": {"_id": "$feedback_type", "count": {"$sum": 1}}}
            ]
            
            # Execute Pipelines in Parallel (conceptually, sequential here)
            origin_stats = list(collection.aggregate(origin_pipeline))
            province_stats = list(collection.aggregate(province_pipeline))
            interest_stats = list(collection.aggregate(interest_pipeline))
            
            # Location Stats (Top questioned locations) - ใช้ pipeline เดียวกับ interest แต่เปลี่ยน field
            location_pipeline = [
                match_stage,
                {"$match": {"location_title": {"$ne": None}}},
                {"$group": {"_id": "$location_title", "count": {"$sum": 1}}},
                {"$sort": {"count": -1}},
                {"$limit": 10}
            ]
            location_stats = list(collection.aggregate(location_pipeline))
            
            # Count total
            total_conversations = collection.count_documents(match_stage["$match"])
            
            # Execute Feedback Pipeline
            feedback_collection = self.get_collection("feedback_logs")
            feedback_stats = []
            if feedback_collection is not None:
                feedback_stats = list(feedback_collection.aggregate(feedback_pipeline))
            
            # Default sample data for province if empty (ยังไม่มีการเก็บข้อมูล)
            if not province_stats:
                province_stats = [
                    {"_id": "กรุงเทพมหานคร", "count": 0},
                    {"_id": "เชียงใหม่", "count": 0},
                    {"_id": "น่าน", "count": 0},
                    {"_id": "ลำปาง", "count": 0},
                    {"_id": "แพร่", "count": 0},
                ]

            return {
                "origin_stats": origin_stats,
                "province_stats": province_stats,
                "interest_stats": interest_stats,
                "location_stats": location_stats,
                "total_conversations": total_conversations,
                "feedback_stats": feedback_stats  # Returns list like: [{"_id": "like", "count": 10}, ...]
            }

        except Exception as e:
            print(f"❌ เกิดข้อผิดพลาดในการรวบรวมข้อมูล Analytics: {e}")
            return {"origin_stats": [], "province_stats": [], "interest_stats": [], "location_stats": [], "total_conversations": 0, "feedback_stats": []}

    def get_top_locations(self, limit: int = 5, days: int = 30) -> list:
        """
        ดึงรายชื่อสถานที่ยอดฮิต (Top Locations) จาก Analytics Logs
        คืนค่าเป็น list of dict: [{"_id": "วัดภูมินทร์", "count": 10}, ...]
        """
        collection = self.get_collection("analytics_logs")
        if collection is None:
            return []
            
        try:
            from datetime import datetime, timedelta, timezone
            cutoff_date = datetime.now(timezone.utc) - timedelta(days=days)
            
            pipeline = [
                {"$match": {"timestamp": {"$gte": cutoff_date}, "location_title": {"$ne": None}}},
                {"$group": {"_id": "$location_title", "count": {"$sum": 1}}},
                {"$sort": {"count": -1}},
                {"$limit": limit}
            ]
            
            results = list(collection.aggregate(pipeline))
            return results
        except Exception as e:
            print(f"❌ Error getting top locations: {e}")
            return []

    def get_recommended_attractions(self, limit: int = 5) -> list:
        """
        🆕 ดึงสถานที่ท่องเที่ยวแนะนำสำหรับ Broad Query
        กรองเฉพาะ category ที่เป็นสถานที่ท่องเที่ยว ไม่รวมอำเภอ
        Returns: list of documents
        """
        collection = self.get_collection("nan_locations")
        if collection is None:
            return []
        
        try:
            # 🔧 Categories ที่เป็นสถานที่ท่องเที่ยว (ภาษาไทย - ตาม JSONL data)
            # ⚠️ ไม่รวม: 'ข้อมูลอำเภอ', 'ข้อมูลภาพรวมจังหวัด', 'ข้อมูลเศรษฐกิจ'
            TOURIST_CATEGORIES = [
                # สถานที่ท่องเที่ยว
                "ข้อมูลสถานที่ท่องเที่ยวทางวัฒนธรรมและศาสนา",
                "ข้อมูลสถานที่ท่องเที่ยวทางธรรมชาติ",
                "ข้อมูลสถานที่ที่เกี่ยวข้องกับประวัติศาสตร์",
                "ข้อมูลท่องเที่ยวเชิงธรรมชาติ",
                "ข้อมูลท่องเที่ยวเชิงผจญภัย",
                "ข้อมูลสถานที่ท่องเที่ยว",
                "ข้อมูลสถานที่ท่องเที่ยวเชิงวิถีชีวิตและภูมิปัญญา",
                "ข้อมูลสถานที่ท่องเที่ยวทางวัฒนธรรมและประวัติศาสตร์",
                "ข้อมูลสถานที่ท่องเที่ยวทางธรรมชาติและชุมชน",
                "ข้อมูลสถานที่ท่องเที่ยวทางธรรมชาติและจุดถ่ายภาพ",
                "ข้อมูลท่องเที่ยวเชิงธรรมชาติและทิวทัศน์",
                "ข้อมูลสถานที่ท่องเที่ยวและตลาด",
                # ร้านอาหาร/คาเฟ่
                "ข้อมูลร้านอาหารและแหล่งของกิน",
                "ข้อมูลร้านอาหารและคาเฟ่",
                "ข้อมูลร้านอาหารและที่พัก",
                # วัฒนธรรม
                "ข้อมูลวัฒนธรรม",
                "ข้อมูลประวัติศาสตร์เมืองน่าน",
                "ข้อมูลชนพื้นเมือง",
                # ช้อปปิ้ง
                "ข้อมูลแหล่งช้อปปิ้งและสินค้าที่ระลึก",
                "ข้อมูลแหล่งช้อปปิ้งและบริการ",
                "ข้อมูลร้านอาหารและแหล่งช้อปปิ้ง",
            ]
            
            # Query: หาสถานที่ท่องเที่ยวที่มีรูปภาพ
            pipeline = [
                {"$match": {
                    "category": {"$in": TOURIST_CATEGORIES}
                }},
                {"$sample": {"size": limit}}  # Random selection
            ]
            
            results = list(collection.aggregate(pipeline))
            print(f"🎯 [DB] พบสถานที่ท่องเที่ยวแนะนำ {len(results)} แห่ง")
            for doc in results:
                print(f"   - {doc.get('title')} ({doc.get('category')})")
            return results
        except Exception as e:
            print(f"❌ Error getting recommended attractions: {e}")
            return []