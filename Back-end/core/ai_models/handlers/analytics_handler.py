# /core/ai_models/handlers/analytics_handler.py

import asyncio
import logging
import json
from datetime import datetime, timezone
from typing import Dict, Any, Optional, Callable, Awaitable

from core.database.mongodb_manager import MongoDBManager
from core.ai_models.query_interpreter import QueryInterpreter
from core.services.language_detector import language_detector  # 🌐 Auto-detect language

class AnalyticsHandler:
    def __init__(self, 
                mongo_manager: MongoDBManager, 
                query_interpreter: QueryInterpreter,
                orchestrator_callback: Callable[..., Awaitable[dict]]):
        """
        สร้าง Handler สำหรับจัดการ Logic ด้าน Analytics โดยเฉพาะ
        """
        self.mongo_manager = mongo_manager
        self.query_interpreter = query_interpreter
        self.orchestrator_callback = orchestrator_callback
        self.analytics_log_collection = self.mongo_manager.get_collection("analytics_logs")
        self.lang_detector = language_detector  # 🌐 Language detector instance
        logging.info("✅ Analytics Handler initialized.")

    async def _log_analytics_event_async(self, log_data: dict):
        """ (Async Wrapper) เรียกใช้ฟังก์ชัน log_analytics_event ใน Thread แยก """
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
You MUST return a JSON object with three keys: "user_origin" (str or null), "user_province" (str or null), and "interest_topic" (str or null).
- "user_origin": The country/nationality mentioned (e.g., "Japan", "Thailand", "China")
- "user_province": If Thai, extract the Thai province (จังหวัด) mentioned (e.g., "กรุงเทพ", "เชียงใหม่", "ขอนแก่น", "สุราษฎร์ธานี")
- "interest_topic": The topic of interest (e.g., temples, food, nature, cafes)
- If the user asks a question, extract the main topic (e.g., "วัดภูมินทร์ไปไง" -> "Temple").
- If you can't tell, return null for that field.

EXAMPLES:
- Input: "มาจากญี่ปุ่นครับ" -> {{"user_origin": "Japan", "user_province": null, "interest_topic": null}}
- Input: "มาจากกรุงเทพครับ" -> {{"user_origin": "Thailand", "user_province": "กรุงเทพ", "interest_topic": null}}
- Input: "คนเชียงใหม่ค่ะ" -> {{"user_origin": "Thailand", "user_province": "เชียงใหม่", "interest_topic": null}}
- Input: "มาจากขอนแก่น อยากดูวัดสวยๆ" -> {{"user_origin": "Thailand", "user_province": "ขอนแก่น", "interest_topic": "Temple"}}
- Input: "อยากไปคาเฟ่สวยๆ" -> {{"user_origin": null, "user_province": null, "interest_topic": "Cafe"}}
- Input: "คนไทยนี่แหละ" -> {{"user_origin": "Thailand", "user_province": null, "interest_topic": null}}
- Input: "ไม่บอก" -> {{"user_origin": "Declined", "user_province": null, "interest_topic": null}}
- Input: "วัดภูมินทร์ไปไง" -> {{"user_origin": null, "user_province": null, "interest_topic": "Temple"}}
"""
        
        extracted_data_str = await self.query_interpreter._get_groq_response(system_prompt, user_answer)
        
        try:
            data = json.loads(extracted_data_str)
            return data
        except Exception as e:
            logging.error(f"Failed to parse analytics JSON from LLM: {e}")
            return {"user_origin": None, "user_province": None, "interest_topic": None}

    def _get_boost_keywords(self, origin: str) -> str:
        if not origin: return ""
        origin_lower = origin.lower()
        
        keywords = []

        if any(x in origin_lower for x in ["จีน", "china", "chinese"]):
            keywords.append("ไทลื้อ สิบสองปันนา ประวัติศาสตร์การอพยพ จีนฮ่อ")

        if any(x in origin_lower for x in ["ลาว", "laos", "lao"]):
            keywords.append("ชายแดนลาว ด่านห้วยโก๋น อำเภอทุ่งช้าง อำเภอเฉลิมพระเกียรติ ความสัมพันธ์ล้านช้าง")

        if any(x in origin_lower for x in ["พม่า", "myanmar", "burma"]):
            keywords.append("ประวัติศาสตร์เมืองน่านยุคพม่าปกครอง ศิลปะล้านนาผสมพม่า")

        if any(x in origin_lower for x in ["ญี่ปุ่น", "japan", "japanese"]):
            keywords.append("คาเฟ่สไตล์ญี่ปุ่น Hani Creativespace มินิมอล ชาเขียวมัทฉะ")

        if any(x in origin_lower for x in ["ยุโรป", "europe", "america", "usa", "uk", "ฝรั่ง", "ตะวันตก", "western"]):
            keywords.append("สถาปัตยกรรมตะวันตก พิพิธภัณฑสถานแห่งชาติน่าน(หอคำ) ภาพจิตรกรรมวัดภูมินทร์(ชาวต่างชาติ) สวิตเซอร์แลนด์เมืองน่าน(ห้วยงิม)")

        if any(x in origin_lower for x in ["สุโขทัย", "sukhothai"]):
            keywords.append("วัดพระธาตุแช่แห้ง วัดพระธาตุช้างค้ำ ความสัมพันธ์สุโขทัย ศิลปะสุโขทัย")
        
        if any(x in origin_lower for x in ["กรุงเทพ", "bangkok", "กทม", "เมืองหลวง"]):
            keywords.append("ธรรมชาติ สโลว์ไลฟ์ ดอยเสมอดาว บ่อเกลือ พักผ่อน คาเฟ่")

        return " ".join(keywords)

    async def handle_analytics_response(self, user_answer: str, session_id: str, mode: str) -> dict:
        """
        (เมธอดหลัก) จัดการคำตอบที่ผู้ใช้ตอบกลับมาหลังจากถูกถามคำถามต้อนรับ
        """
        logging.info(f"📊 [AnalyticsHandler] Processing response '{user_answer}' for Session '{session_id}'")
        
        # 🌐 Auto-detect language from user message
        detected_lang = self.lang_detector.detect(user_answer)
        lang_info = self.lang_detector.get_language_info(detected_lang)
        logging.info(f"🌐 [Analytics] Detected language: {detected_lang} ({lang_info['name']})")
        
        extracted_data = await self._extract_analytics_data_with_llm(user_answer)
        
        # 🌐 Auto-infer origin from language if not explicitly stated
        user_origin = extracted_data.get("user_origin")
        if not user_origin and detected_lang != "th":
            # Map language to likely origin
            lang_to_origin = {
                "en": "English-speaking",
                "zh": "China",
                "ja": "Japan",
                "hi": "India",
                "ru": "Russia",
                "ms": "Malaysia",
            }
            user_origin = lang_to_origin.get(detected_lang)
            logging.info(f"🌐 [Analytics] Inferred origin from language: {user_origin}")
        
        log_data = {
            "session_id": session_id,
            "timestamp": datetime.now(timezone.utc),
            "raw_query": user_answer,
            "user_origin": user_origin or extracted_data.get("user_origin"),
            "user_province": extracted_data.get("user_province"),  # เก็บจังหวัด (สำหรับคนไทย)
            "interest_topic": extracted_data.get("interest_topic"),
            "detected_language": detected_lang,  # 🌐 Real detected language
            "language_name": lang_info["name"],   # e.g., "English", "Japanese"
        }
        
        is_implicit_query = False
        if log_data["interest_topic"]:
            is_implicit_query = True 
        elif "วัด" in user_answer or "เที่ยว" in user_answer or "อยากไป" in user_answer: 
            is_implicit_query = True

        asyncio.create_task(self._log_analytics_event_async(log_data))

        if is_implicit_query:
            origin = log_data.get("user_origin")
            boost_keywords = self._get_boost_keywords(origin)
            
            final_query = user_answer
            if boost_keywords:
                final_query = f"{user_answer} (บริบทเสริม: {boost_keywords})"
                logging.info(f"🚀 [Analytics] Boosted Query: '{final_query}'")

            return await self.orchestrator_callback(query=final_query, mode=mode, session_id=session_id)
        
        else:
            # ผู้ใช้บอกแค่ที่มา → ใช้ LLM ตอบพร้อม boost keywords ตาม origin
            origin = log_data.get('user_origin')
            province = log_data.get('user_province')
            
            # สร้าง context สำหรับ LLM
            boost_keywords = self._get_boost_keywords(origin) if origin else ""
            
            if origin or province:
                location_text = province if province else origin
                # ส่งไปให้ LLM ตอบพร้อม context เสริม
                welcome_query = f"ผมมาจาก{location_text}ครับ แนะนำสถานที่เที่ยวน่านหน่อย"
                if boost_keywords:
                    welcome_query = f"{welcome_query} (บริบทเสริม: {boost_keywords})"
                logging.info(f"🚀 [Analytics] Welcome Query with Boost: '{welcome_query}'")
                return await self.orchestrator_callback(query=welcome_query, mode=mode, session_id=session_id)
            else:
                # ถ้าไม่มีข้อมูล origin ให้ตอบสั้นๆ
                return {
                    "answer": "ยินดีต้อนรับสู่น่านค่ะ! 🎉 ถามอะไรก็ได้เลยนะคะ จะแนะนำวัด คาเฟ่ ร้านอาหาร หรือธรรมชาติดีคะ?",
                    "action": None,
                    "sources": [],
                    "image_url": None,
                    "image_gallery": [],
                }

    async def log_interest_event(self, session_id: str, topic: str, query: str):
        """
        บันทึกความสนใจของผู้ใช้ (Interest) จากการถามปกติ (ไม่ใช่ Welcome Flow)
        พร้อม Auto-detect language และ infer origin
        """
        if not topic: return
        
        # 🌐 Auto-detect language and infer origin
        detected_lang = self.lang_detector.detect(query)
        lang_info = self.lang_detector.get_language_info(detected_lang)
        
        # Infer origin from language (if not Thai)
        user_origin = None
        if detected_lang != "th":
            lang_to_origin = {
                "en": "English-speaking",
                "zh": "China", 
                "ja": "Japan",
                "hi": "India",
                "ru": "Russia",
                "ms": "Malaysia",
            }
            user_origin = lang_to_origin.get(detected_lang)

        log_data = {
            "session_id": session_id,
            "timestamp": datetime.now(timezone.utc),
            "raw_query": query,
            "user_origin": user_origin,  # 🌐 Inferred from language
            "interest_topic": topic,
            "detected_language": detected_lang,  # 🌐 Real detected language
            "language_name": lang_info["name"],
            "event_type": "query_interest"  # ระบุว่าเป็น event จากการถาม
        }
        
        # บันทึกลง DB แบบ Fire-and-forget
        asyncio.create_task(self._log_analytics_event_async(log_data))