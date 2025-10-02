# /core/database/mongodb_manager.py (ฉบับสมบูรณ์สำหรับตรวจสอบ)

from pymongo import MongoClient
from bson import ObjectId 
from core.config import settings

class MongoDBManager:
    def __init__(self):
        try:
            self.client = MongoClient(settings.MONGO_URI, serverSelectionTimeoutMS=5000)
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

    def add_location(self, location_data: dict, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            result = collection.insert_one(location_data)
            print(f"📄 Added new location with ID: {result.inserted_id}")
            return str(result.inserted_id)
        return None
    
    # ====================================================================
    # --- ตรวจสอบให้แน่ใจว่าฟังก์ชันนี้อยู่ในไฟล์ ---
    # ====================================================================
    def get_location_by_id(self, mongo_id: str, collection_name: str = "nan_locations"):
        """
        ค้นหาข้อมูลด้วย MongoDB _id
        """
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                object_id_to_find = ObjectId(mongo_id)
                return collection.find_one({"_id": object_id_to_find})
            except Exception as e:
                print(f"❌ Error finding document by ID '{mongo_id}': {e}")
                return None
        return None
    # ====================================================================

    def get_location_by_title(self, title: str, collection_name: str = "nan_locations"):
        """
        ค้นหาข้อมูลจากฟิลด์ title
        """
        collection = self.get_collection(collection_name)
        if collection is not None:
            return collection.find_one({"title": title})
        return None

    def get_all_locations(self, collection_name: str = "nan_locations"):
        """
        ดึงข้อมูลสถานที่ทั้งหมดจาก collection ที่ระบุ
        """
        collection = self.get_collection(collection_name)
        if collection is not None:
            return list(collection.find({}))
        return []

    def update_location(self, mongo_id: str, new_data: dict, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                object_id_to_update = ObjectId(mongo_id)
                result = collection.update_one(
                    {"_id": object_id_to_update},
                    {"$set": new_data}
                )
                return result.modified_count
            except Exception as e:
                print(f"❌ Error updating document by ID: {e}")
                return 0
        return 0

    def delete_location(self, mongo_id: str, collection_name: str = "nan_locations"):
        collection = self.get_collection(collection_name)
        if collection is not None:
            try:
                object_id_to_delete = ObjectId(mongo_id)
                result = collection.delete_one({"_id": object_id_to_delete})
                return result.deleted_count
            except Exception as e:
                print(f"❌ Error deleting document by ID: {e}")
                return 0
        return 0