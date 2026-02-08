import pymongo
import sys

# Hardcode localhost connection (Standard for local dev)
MONGO_URI = "mongodb://localhost:27017/"
DB_NAME = "ai_robot_guide_db" # เดาชื่อ DB จาก project name หรือ default
# หรือลอง list database names ดู

try:
    client = pymongo.MongoClient(MONGO_URI, serverSelectionTimeoutMS=2000)
    client.server_info() # Check connection
    print("✅ Connected to MongoDB")
    
    # List Databases to be sure
    dbs = client.list_database_names()
    print(f"📂 Databases: {dbs}")
    
    # Pick the right DB (Standard naming usually)
    target_db = next((db for db in dbs if 'guide' in db or 'robot' in db or 'nan' in db), dbs[0])
    print(f"👉 Using Query DB: {target_db}")
    db = client[target_db]
    
    col_name = "nan_locations"
    if col_name not in db.list_collection_names():
        print(f"❌ Collection '{col_name}' not found!")
        sys.exit(1)
        
    collection = db[col_name]
    
    # Search for "เขาน้อย"
    print(f"\n🔎 Searching for 'เขาน้อย' in {col_name}...")
    query = {"title": {"$regex": "เขาน้อย", "$options": "i"}}
    
    docs = list(collection.find(query))
    print(f"Found {len(docs)} documents.")
    
    for doc in docs:
        print(f"---")
        print(f"📌 Title: {doc.get('title')}")
        print(f"   Slug:  {doc.get('slug')}")
        print(f"   Image: {doc.get('image_url')}")
        print(f"   LocData: {doc.get('location_data')}")
        
except Exception as e:
    print(f"❌ Error: {e}")
