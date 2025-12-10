
import logging
from datetime import datetime, timezone
from core.database.mongodb_manager import MongoDBManager

class AnalyticsService:
    def __init__(self, mongo_manager: MongoDBManager):
        self.mongo_manager = mongo_manager
        self.collection = self.mongo_manager.get_collection("analytics_logs")
    
    async def log_interaction(self, 
                              session_id: str, 
                              user_query: str, 
                              response: str, 
                              topic: str = None, 
                              user_origin: str = None, 
                              sentiment: str = None):
        """
        Logs a single interaction to the analytics_logs collection.
        This is fire-and-forget (should be awaited but not block critical path if possible).
        """
        if self.collection is None:
            logging.warning("Analytics collection not available, skipping log.")
            return

        try:
            log_entry = {
                "session_id": session_id,
                "timestamp": datetime.now(timezone.utc),
                "user_query": user_query,
                "ai_response": response,
                "interest_topic": topic,      # e.g., "Culture", "Food"
                "user_origin": user_origin,   # e.g., "China", "Bangkok" (if detected)
                "sentiment": sentiment,       # e.g., "Positive", "Neutral"
                "meta": {
                    "query_length": len(user_query) if user_query else 0,
                    "response_length": len(response) if response else 0
                }
            }
            
            # Using insert_one directly (could be batched in high-load systems)
            self.collection.insert_one(log_entry)
            logging.debug(f"📊 Analytics Logged: {topic} | {user_origin}")

        except Exception as e:
            logging.error(f"❌ Failed to log analytics: {e}")

    async def get_dashboard_summary(self, days: int = 30):
        """
        Wrapper to get aggregated stats.
        Note: The heavy lifting is currently in MongoDBManager.get_analytics_summary.
        We can move it here later for better separation of concerns.
        """
        # User requested ONLY aggregated stats (charts/totals)
        # Detailed logs are NOT required.
        summary = self.mongo_manager.get_analytics_summary(days)
        return summary
    
    # Kept method for future use if needed, but not called in summary anymore
    async def get_recent_logs(self, limit: int = 50):
        """Fetches the most recent chat logs."""
        if self.collection is None: return []
        try:
            cursor = self.collection.find({}).sort("timestamp", -1).limit(limit)
            logs = list(cursor)
            # Convert ObjectId to str for JSON serialization
            for log in logs:
                if "_id" in log: log["_id"] = str(log["_id"])
            return logs
        except Exception as e:
            logging.error(f"❌ Error fetching interaction logs: {e}")
            return []
