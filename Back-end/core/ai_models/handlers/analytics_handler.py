# /core/handlers/analytics_handler.py (ไฟล์ใหม่)
import asyncio
import logging
import json
from datetime import datetime, timezone
from typing import Dict, Any, Optional, Callable, Awaitable

from core.database.mongodb_manager import MongoDBManager
from core.ai_models.query_interpreter import QueryInterpreter

class AnalyticsHandler:
    def __init__(self, 
                mongo_manager: MongoDBManager, 
                query_interpreter: QueryInterpreter,
                orchestrator_callback: Callable[..., Awaitable[dict]]):
        """
        สร้าง Handler สำหรับจัดการ Logic ด้าน Analytics โดยเฉพาะ
        
        Args:
            mongo_manager: instance ของ MongoDBManager
            query_interpreter: instance ของ QueryInterpreter (เพื่อใช้ LLM สกัดข้อมูล)
            orchestrator_callback: ฟังก์ชัน answer_query จาก RAGOrchestrator เพื่อใช้ "เรียกกลับ"
        """
        self.mongo_manager = mongo_manager
        self.query_interpreter = query_interpreter
        self.orchestrator_callback = orchestrator_callback
        self.analytics_log_collection = self.mongo_manager.get_collection("analytics_logs")
        logging.info("✅ Analytics Handler initialized.")

    async def _log_analytics_event_async(self, log_data: dict):
        """ (Async Wrapper) เรียกใช้ฟังก์ชัน log_analytics_event (ที่เป็น Sync) ใน Thread แยก """
        if self.analytics_log_collection is None:
            logging.warning("Cannot log analytics: collection not available.")
            return
        try:
            await asyncio.to_thread(
                self.mongo_manager.log_analytics_event, 
                log_data, 
                collection_name="analytics_logs"
            )
        except Exception as e:
            logging.error(f"❌ [Analytics] Async logging failed: {e}", exc_info=True)

    async def _extract_analytics_data_with_llm(self, user_answer: str) -> Dict[str, Any]:
        system_prompt = f"""You are an entity extractor. Analyze the user's text, which is a response to the question "Where are you from? OR What are you interested in?".
You MUST return a JSON object with two keys: "user_origin" (str or null) and "interest_topic" (str or null).
- If the user mentions a place (country, city, region), put it in "user_origin".
- If the user mentions a topic (e.g., temples, food, nature, cafes), put it in "interest_topic".
- If the user asks a question, extract the main topic (e.g., "วัดภูมินทร์ไปไง" -> "Temple").
- If you can't tell, return null for both.

EXAMPLES:
- Input: "มาจากญี่ปุ่นครับ" -> {{"user_origin": "Japan", "interest_topic": null}}
- Input: "อยากไปคาเฟ่สวยๆ" -> {{"user_origin": null, "interest_topic": "Cafe"}}
- Input: "คนไทยนี่แหละ" -> {{"user_origin": "Thailand", "interest_topic": null}}
- Input: "ไม่บอก" -> {{"user_origin": "Declined", "interest_topic": null}}
- Input: "วัดภูมินทร์ไปไง" -> {{"user_origin": null, "interest_topic": "Temple"}}
"""
        
        extracted_data_str = await self.query_interpreter._get_groq_response(system_prompt, user_answer)
        
        try:
            data = json.loads(extracted_data_str)
            return data
        except Exception as e:
            logging.error(f"Failed to parse analytics JSON from LLM: {e}")
            return {"user_origin": None, "interest_topic": None}

    async def handle_analytics_response(self, user_answer: str, session_id: str, mode: str) -> dict:
        """
        (เมธอดหลัก) จัดการคำตอบที่ผู้ใช้ตอบกลับมาหลังจากถูกถามคำถามต้อนรับ
        """
        logging.info(f"📊 [AnalyticsHandler] Processing response '{user_answer}' for Session '{session_id}'")
        
        extracted_data = await self._extract_analytics_data_with_llm(user_answer)
        
        log_data = {
            "session_id": session_id,
            "timestamp": datetime.now(timezone.utc),
            "raw_query": user_answer,
            "user_origin": extracted_data.get("user_origin"),
            "interest_topic": extracted_data.get("interest_topic"),
            "detected_language": "th"
        }
        
        is_implicit_query = False
        if log_data["interest_topic"]:
            is_implicit_query = True 
        elif "วัด" in user_answer: 
            log_data["interest_topic"] = "Temple"
            is_implicit_query = True

        asyncio.create_task(self._log_analytics_event_async(log_data))

        if is_implicit_query:
            logging.info(f"📊 [AnalyticsHandler] Response '{user_answer}' was an implicit query. Re-routing...")
            return await self.orchestrator_callback(query=user_answer, mode=mode, session_id=session_id)
        
        else:
            return {
                "answer": "ขอบคุณสำหรับข้อมูลค่ะ! ไม่ทราบว่าสนใจสอบถามเรื่องไหนในน่านเป็นพิเศษไหมคะ?",
                "action": None,
                "sources": [],
                "image_url": None,
                "image_gallery": [],
            }