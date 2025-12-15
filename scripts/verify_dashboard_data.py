
import sys
import os
import json

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Back-end")))

from core.database.mongodb_manager import MongoDBManager

def verify_dashboard_data():
    try:
        mongo = MongoDBManager()
        print(f"✅ Connected to Database: {mongo.db.name}")
        
        # Test get_analytics_summary
        summary = mongo.get_analytics_summary(days=30)
        
        print("\n📊 Dashboard Data Summary:")
        print(f"   - Total Conversations: {summary.get('total_conversations')}")
        print(f"   - Location Stats Count: {len(summary.get('location_stats'))}")
        
        feedback_stats = summary.get('feedback_stats', [])
        print(f"   - Feedback Stats: {json.dumps(feedback_stats, ensure_ascii=False)}")
        
        if feedback_stats:
            print("\n✅ Feedback data is available!")
        else:
            print("\n⚠️ Feedback data is empty (might be no logs yet).")

    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    verify_dashboard_data()
