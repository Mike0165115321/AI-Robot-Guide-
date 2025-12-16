
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
                              location_title: str = None,
                              user_origin: str = None, 
                              user_province: str = None,
                              sentiment: str = None):
        """
        Logs a single interaction to the analytics_logs collection.
        This is fire-and-forget (should be awaited but not block critical path if possible).
        """
        if self.collection is None:
            logging.warning("ไม่พบคอลเลกชัน Analytics ข้ามการบันทึก log")
            return

        try:
            log_entry = {
                "session_id": session_id,
                "timestamp": datetime.now(timezone.utc),
                "user_query": user_query,
                "ai_response": response,
                "interest_topic": topic,          # e.g., "Culture", "Food" (category)
                "location_title": location_title, # e.g., "วัดภูมินทร์" (specific place name)
                "user_origin": user_origin,       # e.g., "China", "Japan" (for foreigners)
                "user_province": user_province,   # e.g., "กรุงเทพฯ" (for Thai visitors)
                "sentiment": sentiment,           # e.g., "Positive", "Neutral"
                "meta": {
                    "query_length": len(user_query) if user_query else 0,
                    "response_length": len(response) if response else 0
                }
            }
            
            # Using insert_one directly (could be batched in high-load systems)
            # Use asyncio.to_thread to avoid blocking the main event loop
            import asyncio
            await asyncio.to_thread(self.collection.insert_one, log_entry)
            
            logging.info(f"📊 [Analytics] บันทึกสำเร็จ: '{user_query[:30]}...' -> Topic: {topic}")

        except Exception as e:
            logging.error(f"❌ [Analytics] บันทึก analytics ล้มเหลว: {e}", exc_info=True)

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
    
    async def get_trending_locations(self, limit: int = 5) -> list:
        """
        Retrieves top trending locations from analytics logs.
        Used by RAG to enhance broad queries.
        """
        try:
            # Use asyncio.to_thread for database I/O
            import asyncio
            trending = await asyncio.to_thread(self.mongo_manager.get_top_locations, limit=limit, days=30)
            return [t["_id"] for t in trending] # Return list of names only e.g. ["วัดภูมินทร์", "วัดพระธาตุแช่แห้ง"]
        except Exception as e:
            logging.error(f"❌ [Analytics] Failed to get trending locations: {e}")
            return []
    


    async def log_feedback(self, session_id: str, query: str, response: str, feedback_type: str, reason: str = None):
        """
        Logs user feedback (Like/Dislike).
        """
        try:
            feedback_collection = self.mongo_manager.get_collection("feedback_logs")
            if feedback_collection is None:
                logging.warning("ไม่พบคอลเลกชัน feedback_logs ข้ามการบันทึก feedback")
                return

            log_entry = {
                "session_id": session_id,
                "timestamp": datetime.now(timezone.utc),
                "user_query": query,
                "ai_response": response,
                "feedback_type": feedback_type, # "like" or "dislike"
                "reason": reason
            }
            feedback_collection.insert_one(log_entry)
            logging.info(f"👍👎 [Feedback] Recorded: {feedback_type} for Session: {session_id}")
            
            # TODO: If dislike, trigger Self-Correction logic here (Future Phase)
            
        except Exception as e:
            logging.error(f"❌ บันทึก feedback ล้มเหลว: {e}")
