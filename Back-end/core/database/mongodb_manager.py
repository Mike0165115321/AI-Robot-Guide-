from pymongo import MongoClient
from bson import ObjectId
from bson.errors import InvalidId
import re
from core.config import settings
from typing import List # 🚀 [เพิ่ม]

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
            print("✅ MongoDB connection successful.")
        except Exception as e:
            print(f"❌ Failed to connect to MongoDB: {e}")
            self.client = None
            self.db = None

    def get_collection(self, collection_name: str):
        if self.db is not None:
            return self.db[collection_name]
        return None

    # ... (โค้ด get_locations_by_ids, add_location, ... , delete_location_by_slug ... เหมือนเดิม) ...
    def get_locations_by_ids(self, mongo_ids: list[str], collection_name: str = "nan_locations") -> list[dict]:
        collection = self.get_collection(collection_name)
        if collection is None or not mongo_ids: return []
        try:
            valid_object_ids = []
            for mid in mongo_ids:
                try: valid_object_ids.append(ObjectId(mid))
                except InvalidId: print(f"⚠️ Warning: Invalid MongoDB ID ignored: {mid}")
            if not valid_object_ids: return []
            cursor = collection.find({"_id": {"$in": valid_object_ids}})
            docs_map = {str(doc["_id"]): doc for doc in cursor}
            ordered_docs = []
            for mid in mongo_ids:
                doc = docs_map.get(mid)
                if doc: ordered_docs.append(doc)
            return ordered_docs
        except Exception as e:
            print(f"❌ Error fetching multiple locations by IDs: {e}")
            return []
        
    def add_location(self, location_data: dict, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                result = collection.insert_one(location_data)
                print(f"📄 Added new location with ID: {result.inserted_id}")
                return str(result.inserted_id)
            except Exception as e:
                print(f"❌ Error adding location: {e}")
                return None
        return None
    
    def get_location_by_id(self, mongo_id: str, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try: return collection.find_one({"_id": ObjectId(mongo_id)})
            except InvalidId:
                print(f"❌ Invalid MongoDB ID format: '{mongo_id}'")
                return None
            except Exception as e:
                print(f"❌ Error finding document by ID '{mongo_id}': {e}")
                return None
        return None

    def get_location_by_slug(self, slug: str, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try: return collection.find_one({"slug": slug})
            except Exception as e:
                print(f"❌ Error finding document by slug '{slug}': {e}")
                return None
        return None

    def get_location_by_title(self, title: str, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                return collection.find_one({"title": {"$regex": f"^{re.escape(title)}$", "$options": "i"}})
            except Exception as e:
                print(f"❌ Error finding document by case-insensitive title '{title}': {e}")
                return None
        return None

    def get_all_locations(self, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try: return list(collection.find({}))
            except Exception as e:
                print(f"❌ Error fetching all locations: {e}")
                return []
        return []

    def update_location(self, mongo_id: str, new_data: dict, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                result = collection.update_one({"_id": ObjectId(mongo_id)}, {"$set": new_data})
                return result.modified_count
            except InvalidId:
                print(f"❌ Cannot update: Invalid MongoDB ID format: '{mongo_id}'")
                return 0
            except Exception as e:
                print(f"❌ Error updating document by ID: {e}")
                return 0
        return 0

    def update_location_by_slug(self, slug: str, new_data: dict, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                result = collection.update_one({"slug": slug}, {"$set": new_data})
                return result.modified_count
            except Exception as e:
                print(f"❌ Error updating document by slug '{slug}': {e}")
                return 0
        return 0
    def delete_location(self, mongo_id: str, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                result = collection.delete_one({"_id": ObjectId(mongo_id)})
                return result.deleted_count
            except InvalidId:
                print(f"❌ Cannot delete: Invalid MongoDB ID format: '{mongo_id}'")
                return 0
            except Exception as e:
                print(f"❌ Error deleting document by ID: {e}")
                return 0
        return 0

    def delete_location_by_slug(self, slug: str, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                result = collection.delete_one({"slug": slug})
                return result.deleted_count
            except Exception as e:
                print(f"❌ Error deleting document by slug '{slug}': {e}")
                return 0
        return 0

    def log_analytics_event(self, log_data: dict, collection_name: str = "analytics_logs"):
        # (โค้ด Analytics ที่เราทำเสร็จแล้ว)
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                collection.insert_one(log_data)
                print(f"✅ Logged analytics event (Topic: {log_data.get('interest_topic')}, Origin: {log_data.get('user_origin')})")
            except Exception as e:
                print(f"❌ Error logging analytics event: {e}")
        else:
            print("❌ Error logging analytics: Collection 'analytics_logs' not available.")
            
    # --- 🚀 [ฟังก์ชัน V.1.5 ที่เราเพิ่ม] ---
    def get_distinct_categories(self, collection_name: str = "nan_locations") -> List[str]:
        """
        (Sync Function) ดึงรายชื่อ 'category' ทั้งหมดที่ไม่ซ้ำกัน
        """
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                print("🧠 [DB] Querying distinct categories...")
                categories = collection.distinct("category")
                
                # กรองค่าที่เป็น None หรือค่าว่างออก
                valid_categories = [cat for cat in categories if cat]
                
                print(f"✅ [DB] Found {len(valid_categories)} distinct categories.")
                return valid_categories
            except Exception as e:
                print(f"❌ Error getting distinct categories: {e}")
                return []
        return []