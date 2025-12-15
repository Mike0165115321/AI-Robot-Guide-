
import sys
import os
import json

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Back-end")))

from core.database.mongodb_manager import MongoDBManager

def verify_analytics():
    try:
        mongo = MongoDBManager()
        db = mongo.db
        
        print(f"✅ Connected to Database: {db.name}")
        
        collection = db['analytics_logs']
        count = collection.count_documents({})
        print(f"📊 Total Analytics Entries: {count}")
        
        if count > 0:
            print("\n📋 Latest Analytics Entry:")
            cursor = collection.find().sort('timestamp', -1).limit(1)
            for doc in cursor:
                # print(f"   - Full Doc: {doc}")
                print(f"   - ID: {doc.get('_id')}")
                print(f"   - Query: {doc.get('user_query')[:50] if doc.get('user_query') else 'None'}...")
                print(f"   - Timestamp: {doc.get('timestamp')}")
                if 'timestamp' in doc:
                    print(f"   - Timestamp Type: {type(doc['timestamp'])}")
        else:
            print("⚠️ No analytics entries found.")
            
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    verify_analytics()
