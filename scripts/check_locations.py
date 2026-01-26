"""
🔍 Check MongoDB Locations
ค้นหาสถานที่ใน Collection 'nan_locations' ที่มีคำว่า 'เขาน้อย' หรือ 'พระธาตุ'
"""
import asyncio
import os
import sys
from dotenv import load_dotenv

# Setup path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../Back-end')))

from core.database.mongodb_manager import MongoDBManager

# Load Env (for MONGO_DETAILS usually in .env)
load_dotenv()

async def main():
    mongo = MongoDBManager()
    
    # Keyword to search
    keywords = ["เขาน้อย", "พระธาตุเขาน้อย"] 
    
    print(f"📡 Connecting to MongoDB...")
    collection = mongo.get_collection("nan_locations")
    
    if collection is None:
        print("❌ Connect Failed")
        return

    for kw in keywords:
        print(f"\n🔎 Searching for keyword: '{kw}'")
        # Regex search in title or slug
        query = {
            "$or": [
                {"title": {"$regex": kw, "$options": "i"}},
                {"slug": {"$regex": kw, "$options": "i"}}
            ]
        }
        
        count = await asyncio.to_thread(collection.count_documents, query)
        print(f"   Found: {count} documents")
        
        if count > 0:
            cursor = collection.find(query, {"title": 1, "slug": 1, "category": 1, "location_data": 1})
            results = await asyncio.to_thread(list, cursor)
            for res in results:
                print(f"   ✅ Found: {res.get('title')} (Slug: {res.get('slug')})")
                print(f"      LocData: {res.get('location_data')}")

if __name__ == "__main__":
    asyncio.run(main())
