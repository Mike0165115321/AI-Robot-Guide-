# Back-end/core/services/session_manager.py
import logging
from datetime import datetime, timezone
from typing import Dict, Any, Optional, List
from core.database.mongodb_manager import MongoDBManager

class SessionManager:
    def __init__(self, mongo_manager: MongoDBManager):
        self.mongo = mongo_manager
        self.collection = self.mongo.get_collection("chat_sessions")
        logging.info("🧠 [SessionManager] Initialized with MongoDB persistence.")

    async def get_session(self, session_id: str) -> Dict[str, Any]:
        if not session_id:
            return {}
            
        session = self.collection.find_one({"session_id": session_id})
        if not session:
            new_session = {
                "session_id": session_id,
                "created_at": datetime.now(timezone.utc),
                "turn_count": 0,
                "last_topic": None,       # จำ Topic ล่าสุดสำหรับการนำทาง
                "history": [],            # เก็บประวัติการคุย (Short-term memory)
                "user_preferences": {}    # เก็บสิ่งที่ผู้ใช้ชอบ (Long-term memory)
            }
            self.collection.insert_one(new_session)
            return new_session
        return session

    async def update_turn(self, session_id: str, user_query: str, ai_response: str, topic: str = None):
        """บันทึกบทสนทนาและอัปเดต Topic ล่าสุด"""
        if not session_id: return

        update_data = {
            "$inc": {"turn_count": 1},
            "$push": {
                "history": {
                    "$each": [
                        {"role": "user", "content": user_query, "timestamp": datetime.now(timezone.utc)},
                        {"role": "ai", "content": ai_response, "timestamp": datetime.now(timezone.utc)}
                    ],
                    "$slice": -10  # เก็บแค่ 10 ข้อความล่าสุดใน Memory เพื่อไม่ให้ Prompt บวม
                }
            },
            "$set": {"last_active": datetime.now(timezone.utc)}
        }

        if topic:
            update_data["$set"]["last_topic"] = topic

        self.collection.update_one({"session_id": session_id}, update_data)

    async def get_last_topic(self, session_id: str) -> Optional[str]:
        """ดึง Topic ล่าสุดที่คุยกัน (ใช้สำหรับฟีเจอร์ 'นำทางไปที่นั่นหน่อย')"""
        session = await self.get_session(session_id)
        return session.get("last_topic")