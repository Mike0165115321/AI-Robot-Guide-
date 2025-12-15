
import asyncio
import sys
import os
from datetime import datetime, timezone

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Back-end")))

from core.database.mongodb_manager import MongoDBManager
from core.services.analytics_service import AnalyticsService

async def trigger_manual_log():
    try:
        print("🚀 Starting manual log test...")
        mongo = MongoDBManager()
        service = AnalyticsService(mongo)
        
        # Simulate a log interaction
        await service.log_interaction(
            session_id="test_session_manual_001",
            user_query="Test Manual Query",
            response="This is a test response",
            topic="Testing",
            location_title="Test Location",
            user_origin="Thailand",
            user_province="Nan"
        )
        print("✅ log_interaction called successfully.")
        
        # Verify immediately
        collection = mongo.db['analytics_logs']
        latest = await asyncio.to_thread(collection.find_one, {"session_id": "test_session_manual_001"})
        
        if latest:
            print(f"✅ Verified in DB! ID: {latest['_id']}")
        else:
            print("❌ Failed to find the log entry in DB.")
            
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    asyncio.run(trigger_manual_log())
