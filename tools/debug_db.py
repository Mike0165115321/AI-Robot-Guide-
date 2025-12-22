
from pymongo import MongoClient
import os
import sys

# Define settings manually to avoid dependency issues during debug
MONGO_URI = "mongodb://localhost:27017/"
DB_NAME = "nanaiguide"

def check_db():
    print(f"🔌 Connecting to MongoDB at {MONGO_URI}...")
    try:
        client = MongoClient(MONGO_URI, serverSelectionTimeoutMS=2000)
        client.server_info() # Trigger connection
        print("✅ Connection Successful!")
        
        db = client[DB_NAME]
        cols = db.list_collection_names()
        print(f"📂 Collections found: {cols}")
        
        if "nan_locations" in cols:
            count = db.nan_locations.count_documents({})
            print(f"📊 Total documents in 'nan_locations': {count}")
            
            # Check for doc_type="Location" used in navigation_list
            nav_count = db.nan_locations.count_documents({"doc_type": "Location"})
            print(f"📍 Documents with doc_type='Location': {nav_count}")
            
            # Sample one document
            if count > 0:
                print("📝 Sample Document:")
                print(db.nan_locations.find_one({}, {"_id": 0, "title": 1, "doc_type": 1, "category": 1}))
        else:
            print("❌ Collection 'nan_locations' NOT found!")
            
    except Exception as e:
        print(f"❌ Connection Failed: {e}")

if __name__ == "__main__":
    check_db()
