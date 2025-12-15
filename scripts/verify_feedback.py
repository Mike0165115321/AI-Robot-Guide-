
import sys
import os

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Back-end")))

from core.database.mongodb_manager import MongoDBManager

def verify_feedback():
    try:
        mongo = MongoDBManager()
        db = mongo.db
        
        print(f"✅ Connected to Database: {db.name}")
        
        collection = db['feedback_logs']
        count = collection.count_documents({})
        print(f"📊 Total Feedback Entries: {count}")
        
        if count > 0:
            print("\n📋 Latest Feedback Entry:")
            cursor = collection.find().sort('_id', -1).limit(1)
            for doc in cursor:
                print(f"   - ID: {doc.get('_id')}")
                print(f"   - Type: {doc.get('feedback_type')}")
                print(f"   - Query: {doc.get('user_query')[:50]}...")
                print(f"   - Timestamp: {doc.get('timestamp')}")
        else:
            print("⚠️ No feedback entries found.")
            
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    verify_feedback()
