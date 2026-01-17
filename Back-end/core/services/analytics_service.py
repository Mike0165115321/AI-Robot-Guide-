
import logging
import asyncio
from datetime import datetime, timezone, timedelta
from typing import Dict, List, Any, Optional
from collections import Counter
import re
from core.database.mongodb_manager import MongoDBManager


class AnalyticsService:
    """
    📊 Enhanced Analytics Service
    
    รองรับการเก็บและวิเคราะห์ข้อมูลขั้นสูง:
    - User Behavior (Peak hours, Session duration)
    - Query Trends (Top queries, Word frequency)
    - AI Performance (Response time, Confidence distribution)
    - Knowledge Health (Gap stats integration)
    """
    
    def __init__(self, mongo_manager: MongoDBManager):
        self.mongo_manager = mongo_manager
        self.collection = self.mongo_manager.get_collection("analytics_logs")
        logging.info("📊 Analytics Service initialized (Enhanced)")
    
    async def log_interaction(self, 
                              session_id: str, 
                              user_query: str, 
                              response: str, 
                              topic: str = None, 
                              location_title: str = None,
                              user_origin: str = None, 
                              user_province: str = None,
                              sentiment: str = None,
                              # 🆕 Enhanced fields
                              response_time_ms: float = None,
                              confidence_score: float = None,
                              intent: str = None,
                              ai_mode: str = None):
        """
        Logs a single interaction to the analytics_logs collection.
        Enhanced with response time, confidence, and intent tracking.
        """
        if self.collection is None:
            logging.warning("ไม่พบคอลเลกชัน Analytics ข้ามการบันทึก log")
            return

        try:
            now = datetime.now(timezone.utc)
            
            log_entry = {
                "session_id": session_id,
                "timestamp": now,
                "hour": now.hour,  # 🆕 For hourly analysis
                "day_of_week": now.weekday(),  # 🆕 For weekly patterns
                "user_query": user_query,
                "ai_response": response,
                "interest_topic": topic,
                "location_title": location_title,
                "user_origin": user_origin,
                "user_province": user_province,
                "sentiment": sentiment,
                # 🆕 Enhanced metrics
                "response_time_ms": response_time_ms,
                "confidence_score": confidence_score,
                "intent": intent,
                "ai_mode": ai_mode,
                "meta": {
                    "query_length": len(user_query) if user_query else 0,
                    "response_length": len(response) if response else 0
                }
            }
            
            await asyncio.to_thread(self.collection.insert_one, log_entry)
            logging.info(f"📊 [Analytics] บันทึกสำเร็จ: '{user_query[:30]}...' -> Topic: {topic}")

        except Exception as e:
            logging.error(f"❌ [Analytics] บันทึก analytics ล้มเหลว: {e}", exc_info=True)

    async def get_dashboard_summary(self, days: int = 30):
        """Wrapper to get aggregated stats."""
        summary = self.mongo_manager.get_analytics_summary(days)
        return summary
    
    async def get_trending_locations(self, limit: int = 5) -> list:
        """Retrieves top trending locations from analytics logs."""
        try:
            trending = await asyncio.to_thread(self.mongo_manager.get_top_locations, limit=limit, days=30)
            return [t["_id"] for t in trending]
        except Exception as e:
            logging.error(f"❌ [Analytics] Failed to get trending locations: {e}")
            return []

    async def log_feedback(self, session_id: str, query: str, response: str, feedback_type: str, reason: str = None):
        """Logs user feedback (Like/Dislike)."""
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
                "feedback_type": feedback_type,
                "reason": reason
            }
            feedback_collection.insert_one(log_entry)
            logging.info(f"👍👎 [Feedback] Recorded: {feedback_type} for Session: {session_id}")
            
        except Exception as e:
            logging.error(f"❌ บันทึก feedback ล้มเหลว: {e}")

    # ========== 🆕 Enhanced Analytics Methods ==========

    async def get_hourly_usage(self, days: int = 7) -> List[Dict[str, Any]]:
        """
        📊 วิเคราะห์การใช้งานตามชั่วโมง (Peak Hours Analysis)
        
        Returns: [{hour: 0-23, count: N}, ...]
        """
        if self.collection is None:
            return []

        try:
            cutoff = datetime.now(timezone.utc) - timedelta(days=days)
            
            pipeline = [
                {"$match": {"timestamp": {"$gte": cutoff}}},
                {"$group": {
                    "_id": "$hour",
                    "count": {"$sum": 1}
                }},
                {"$sort": {"_id": 1}}
            ]
            
            result = await asyncio.to_thread(list, self.collection.aggregate(pipeline))
            
            # Fill missing hours with 0
            hour_map = {r["_id"]: r["count"] for r in result}
            return [{"hour": h, "count": hour_map.get(h, 0)} for h in range(24)]
            
        except Exception as e:
            logging.error(f"❌ [Analytics] Hourly usage failed: {e}")
            return []

    async def get_daily_usage(self, days: int = 30) -> List[Dict[str, Any]]:
        """
        📈 วิเคราะห์การใช้งานรายวัน (Trend Analysis)
        
        Returns: [{date: "YYYY-MM-DD", count: N}, ...]
        """
        if self.collection is None:
            return []

        try:
            cutoff = datetime.now(timezone.utc) - timedelta(days=days)
            
            pipeline = [
                {"$match": {"timestamp": {"$gte": cutoff}}},
                {"$group": {
                    "_id": {
                        "$dateToString": {"format": "%Y-%m-%d", "date": "$timestamp"}
                    },
                    "count": {"$sum": 1}
                }},
                {"$sort": {"_id": 1}}
            ]
            
            result = await asyncio.to_thread(list, self.collection.aggregate(pipeline))
            return [{"date": r["_id"], "count": r["count"]} for r in result]
            
        except Exception as e:
            logging.error(f"❌ [Analytics] Daily usage failed: {e}")
            return []

    async def get_top_queries(self, limit: int = 10, days: int = 30) -> List[Dict[str, Any]]:
        """
        💬 คำถามยอดนิยม (Top Queries)
        
        Returns: [{query: "...", count: N}, ...]
        """
        if self.collection is None:
            return []

        try:
            cutoff = datetime.now(timezone.utc) - timedelta(days=days)
            
            pipeline = [
                {"$match": {"timestamp": {"$gte": cutoff}, "user_query": {"$ne": None}}},
                {"$group": {
                    "_id": "$user_query",
                    "count": {"$sum": 1}
                }},
                {"$sort": {"count": -1}},
                {"$limit": limit}
            ]
            
            result = await asyncio.to_thread(list, self.collection.aggregate(pipeline))
            return [{"query": r["_id"], "count": r["count"]} for r in result]
            
        except Exception as e:
            logging.error(f"❌ [Analytics] Top queries failed: {e}")
            return []

    async def get_word_frequency(self, limit: int = 30, days: int = 30) -> List[Dict[str, Any]]:
        """
        ☁️ คำที่ถูกใช้บ่อย (Word Cloud Data)
        
        Returns: [{word: "...", count: N}, ...]
        """
        if self.collection is None:
            return []

        try:
            cutoff = datetime.now(timezone.utc) - timedelta(days=days)
            
            # Fetch recent queries
            cursor = self.collection.find(
                {"timestamp": {"$gte": cutoff}, "user_query": {"$ne": None}},
                {"user_query": 1, "_id": 0}
            ).limit(1000)
            
            docs = await asyncio.to_thread(list, cursor)
            
            # Tokenize and count words
            stop_words = {"ที่", "ไหน", "อะไร", "ยังไง", "เป็น", "มี", "ได้", "ไหม", "ครับ", "ค่ะ", "นะ", "หน่อย", "และ", "หรือ", "แต่", "เพราะ", "ถ้า", "ว่า", "ให้", "กับ", "ของ"}
            word_counter = Counter()
            
            for doc in docs:
                query = doc.get("user_query", "")
                # Simple Thai tokenization (space + common patterns)
                words = re.findall(r'[\u0E00-\u0E7F]+|[a-zA-Z]+', query.lower())
                for word in words:
                    if len(word) >= 2 and word not in stop_words:
                        word_counter[word] += 1
            
            return [{"word": w, "count": c} for w, c in word_counter.most_common(limit)]
            
        except Exception as e:
            logging.error(f"❌ [Analytics] Word frequency failed: {e}")
            return []

    async def get_intent_distribution(self, days: int = 30) -> List[Dict[str, Any]]:
        """
        🎯 การกระจายตัวของ Intent (Intent Distribution)
        
        Returns: [{intent: "INFORMATIONAL", count: N}, ...]
        """
        if self.collection is None:
            return []

        try:
            cutoff = datetime.now(timezone.utc) - timedelta(days=days)
            
            pipeline = [
                {"$match": {"timestamp": {"$gte": cutoff}, "intent": {"$ne": None}}},
                {"$group": {
                    "_id": "$intent",
                    "count": {"$sum": 1}
                }},
                {"$sort": {"count": -1}}
            ]
            
            result = await asyncio.to_thread(list, self.collection.aggregate(pipeline))
            return [{"intent": r["_id"], "count": r["count"]} for r in result]
            
        except Exception as e:
            logging.error(f"❌ [Analytics] Intent distribution failed: {e}")
            return []

    async def get_ai_performance(self, days: int = 7) -> Dict[str, Any]:
        """
        ⚡ AI Performance Metrics
        
        Returns: {
            avg_response_time_ms: N,
            avg_confidence: N,
            low_confidence_rate: N,
            total_queries: N
        }
        """
        if self.collection is None:
            return {}

        try:
            cutoff = datetime.now(timezone.utc) - timedelta(days=days)
            
            pipeline = [
                {"$match": {"timestamp": {"$gte": cutoff}}},
                {"$group": {
                    "_id": None,
                    "avg_response_time": {"$avg": "$response_time_ms"},
                    "avg_confidence": {"$avg": "$confidence_score"},
                    "total": {"$sum": 1},
                    "low_conf_count": {
                        "$sum": {"$cond": [{"$lt": ["$confidence_score", 0.45]}, 1, 0]}
                    }
                }}
            ]
            
            result = await asyncio.to_thread(list, self.collection.aggregate(pipeline))
            
            if result:
                data = result[0]
                total = data.get("total", 1)
                return {
                    "avg_response_time_ms": round(data.get("avg_response_time") or 0, 0),
                    "avg_confidence": round((data.get("avg_confidence") or 0) * 100, 1),
                    "low_confidence_rate": round((data.get("low_conf_count", 0) / total) * 100, 1) if total > 0 else 0,
                    "total_queries": total
                }
            
            return {
                "avg_response_time_ms": 0,
                "avg_confidence": 0,
                "low_confidence_rate": 0,
                "total_queries": 0
            }
            
        except Exception as e:
            logging.error(f"❌ [Analytics] AI performance failed: {e}")
            return {}

    async def get_knowledge_health(self) -> Dict[str, Any]:
        """
        🧠 Knowledge Health (Integration with Knowledge Gap Service)
        
        Returns: {
            pending_gaps: N,
            resolved_today: N,
            total_gaps: N,
            answer_rate: N%
        }
        """
        try:
            gap_collection = self.mongo_manager.get_collection("unanswered_questions")
            if gap_collection is None:
                return {"pending_gaps": 0, "resolved_today": 0, "total_gaps": 0, "answer_rate": 100}
            
            today_start = datetime.now(timezone.utc).replace(hour=0, minute=0, second=0, microsecond=0)
            
            pending = await asyncio.to_thread(
                gap_collection.count_documents, {"status": "PENDING"}
            )
            resolved = await asyncio.to_thread(
                gap_collection.count_documents, {"status": "RESOLVED"}
            )
            resolved_today = await asyncio.to_thread(
                gap_collection.count_documents,
                {"status": "RESOLVED", "resolved_at": {"$gte": today_start}}
            )
            total = pending + resolved
            
            return {
                "pending_gaps": pending,
                "resolved_today": resolved_today,
                "total_gaps": total,
                "answer_rate": round((resolved / total) * 100, 1) if total > 0 else 100
            }
            
        except Exception as e:
            logging.error(f"❌ [Analytics] Knowledge health failed: {e}")
            return {"pending_gaps": 0, "resolved_today": 0, "total_gaps": 0, "answer_rate": 100}

    async def get_session_stats(self, days: int = 7) -> Dict[str, Any]:
        """
        👥 Session Statistics
        
        Returns: {
            unique_sessions: N,
            avg_queries_per_session: N,
            returning_users: N%
        }
        """
        if self.collection is None:
            return {}

        try:
            cutoff = datetime.now(timezone.utc) - timedelta(days=days)
            
            pipeline = [
                {"$match": {"timestamp": {"$gte": cutoff}, "session_id": {"$ne": None}}},
                {"$group": {
                    "_id": "$session_id",
                    "count": {"$sum": 1}
                }},
                {"$group": {
                    "_id": None,
                    "unique_sessions": {"$sum": 1},
                    "total_queries": {"$sum": "$count"},
                    "multi_query_sessions": {
                        "$sum": {"$cond": [{"$gt": ["$count", 1]}, 1, 0]}
                    }
                }}
            ]
            
            result = await asyncio.to_thread(list, self.collection.aggregate(pipeline))
            
            if result:
                data = result[0]
                unique = data.get("unique_sessions", 1)
                return {
                    "unique_sessions": unique,
                    "avg_queries_per_session": round(data.get("total_queries", 0) / unique, 1),
                    "engagement_rate": round((data.get("multi_query_sessions", 0) / unique) * 100, 1) if unique > 0 else 0
                }
            
            return {"unique_sessions": 0, "avg_queries_per_session": 0, "engagement_rate": 0}
            
        except Exception as e:
            logging.error(f"❌ [Analytics] Session stats failed: {e}")
            return {}

    async def get_enhanced_dashboard(self, days: int = 30) -> Dict[str, Any]:
        """
        📊 Enhanced Dashboard - รวมทุกข้อมูลในที่เดียว
        
        Returns combined analytics data for the enhanced dashboard.
        """
        try:
            # Fetch all data in parallel
            basic_summary, hourly, daily, top_queries, word_freq, intent_dist, ai_perf, knowledge, sessions = await asyncio.gather(
                self.get_dashboard_summary(days),
                self.get_hourly_usage(min(days, 7)),
                self.get_daily_usage(days),
                self.get_top_queries(10, days),
                self.get_word_frequency(30, days),
                self.get_intent_distribution(days),
                self.get_ai_performance(min(days, 7)),
                self.get_knowledge_health(),
                self.get_session_stats(min(days, 7)),
                return_exceptions=True
            )
            
            # Handle any exceptions
            def safe_result(r):
                return r if not isinstance(r, Exception) else {}
            
            return {
                # Existing data
                **safe_result(basic_summary),
                # New enhanced data
                "hourly_usage": safe_result(hourly) if not isinstance(hourly, Exception) else [],
                "daily_usage": safe_result(daily) if not isinstance(daily, Exception) else [],
                "top_queries": safe_result(top_queries) if not isinstance(top_queries, Exception) else [],
                "word_frequency": safe_result(word_freq) if not isinstance(word_freq, Exception) else [],
                "intent_distribution": safe_result(intent_dist) if not isinstance(intent_dist, Exception) else [],
                "ai_performance": safe_result(ai_perf),
                "knowledge_health": safe_result(knowledge),
                "session_stats": safe_result(sessions)
            }
            
        except Exception as e:
            logging.error(f"❌ [Analytics] Enhanced dashboard failed: {e}")
            return {}

