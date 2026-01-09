import logging
import json
import asyncio
from groq import AsyncGroq
from typing import Dict, Any, Optional, List
from .key_manager import groq_key_manager
from core.config import settings
from core.ai_models.frontline_handler import frontline_handler

class QueryInterpreter:
    _PRE_CORRECTION_MAP = {
        "หวัดดีคับ": "สวัสดี",
        "ดีคับ": "ดีครับ",
        "ขอบคุน": "ขอบคุณ",
        "เปิดเพง": "เปิดเพลง",
        "วัดพูมิน": "วัดภูมินทร์",
        "วัดพูมินทร์": "วัดภูมินทร์",
        "วัดภูมิน": "วัดภูมินทร์",
        "วัดแช่แห้ง": "วัดพระธาตุแช่แห้ง",
        "พระทาดแช่แห้ง": "พระธาตุแช่แห้ง",
        "ดอยเสมอเดา": "ดอยเสมอดาว",
        "เสมอดาว": "ดอยเสมอดาว",
        "ปู่ม่านย่าม่าน": "ปู่ม่านย่าม่าน",
    }

    _CANNED_RESPONSES = {
        "THANKS": {"intent": "SMALL_TALK", "entity": None, "is_complex": False, "sub_queries": [""]},
        "FAREWELL": {"intent": "SMALL_TALK", "entity": None, "is_complex": False, "sub_queries": [""]}
    }
    _QUERY_MAP = {
        "ขอบคุณ": "THANKS", "ขอบใจ": "THANKS", "ขอบคุณครับ": "THANKS", "ขอบคุณค่ะ": "THANKS",
        "ลาก่อน": "FAREWELL", "ไปแล้วนะ": "FAREWELL", "บ๊ายบาย": "FAREWELL",
    }
    def __init__(self):
        self.model_to_use = settings.GROQ_LLAMA_MODEL
        api_key = groq_key_manager.get_key()
        if not api_key:
            logging.error("🚨 [Interpreter] วิกฤต: ไม่พบ Groq API Key ในการเริ่มต้นทำงาน")
            self.client = None
        else:
            self.client = AsyncGroq(api_key=api_key)
        logging.info(f"🧠 Query Interpreter (V6.4 - Pre-correction) เริ่มทำงานด้วยโมเดล: {self.model_to_use}")

        # 🆕 No more Groq Client here.
        # 🆕 No more Groq Client here.
        # FrontlineHandler is a singleton, so we just use the imported instance.
        logging.info("🧠 [QueryInterpreter] Initialized (Switched to Frontline/Google Assistant)")

    async def interpret_and_route(self, query: str) -> Dict[str, Any]:
        """
        Interprets the user query using Frontline Gatekeeper (Google Assistant).
        """
        corrected_query = query # Define this early for use in logging/returns
        
        corrected_query = query # Define this early
        
        # 🚀 0. FAST TRACK: Check for Greetings locally (Speed Optimization)
        # Bypasses both Google Assistant and RAG for instant "Sawasdee" response
        greetings = ["สวัสดี", "หวัดดี", "ดีคับ", "ทักทาย", "hello", "hi"]
        if any(g in query.lower() for g in greetings):
             logging.info(f"⚡ [Interpreter] Fast-Track Greeting Detected: {query}")
             return {
                "intent": "SMALL_TALK",
                "reply": "สวัสดีครับ มีอะไรให้น้องน่านช่วยไหมครับ?",
                "entity": None,
                "is_complex": False,
                "sub_queries": [],
                "location_filter": {},
                "category": None
            }

        # 1. Ask Frontline (Google Assistant)
        # This is the "Gatekeeper" step. Fails fast if it's a simple task.
        frontline_result = await frontline_handler.process_query(query)
        
        f_intent = frontline_result.get("intent")
        
        # ✅ EARLY RETURN: If Frontline knows the answer (Small Talk / Music / News), return immediately!
        if f_intent and f_intent != "RAG_QUERY":
            logging.info(f"⚡ [Interpreter] Frontline gatekeeper handled query: {f_intent}")
            # Ensure structure matches what RAGOrchestrator expects
            frontline_result["corrected_query"] = query
            frontline_result["sub_queries"] = []
            frontline_result["entity"] = None
            frontline_result["is_complex"] = False
            return frontline_result
            
        f_reply = frontline_result.get("reply")
        f_meta = frontline_result.get("metadata", {})
        
        # Default Interpretation Structure
        interpretation = {
            "intent": "INFORMATIONAL", # Default to RAG
            "corrected_query": query,
            "entity": None,
            "is_complex": False,
            "sub_queries": [query],
            "location_filter": {},
            "category": None,
            "reply": None # New field for direct answers
        }
        """
    1.  **INFORMATIONAL (ค่าเริ่มต้น):**
    - ใช้สำหรับ **ทุกคำถาม** ที่เกี่ยวกับจังหวัดน่าน, อากาศ, ร้านอาหาร, ที่พัก, สถานที่ท่องเที่ยว, ประวัติศาสตร์, วัฒนธรรม, การเดินทาง
    - แม้จะเป็นคำถามสั้นๆ เช่น "ที่นั่นสวยไหม", "มีกาแฟไหม", "หิวข้าว" ให้ถือเป็น INFORMATIONAL เพื่อให้ระบบค้นหาข้อมูลจริง
    - ห้ามใช้ SMALL_TALK กับคำถามที่ต้องการข้อมูลสถานที่หรือความรู้
    2.  **SMALL_TALK:**
    - ใช้สำหรับ **การทักทายทั่วไป** (สวัสดี, สบายดีไหม), คำถามส่วนตัวเกี่ยวกับ AI (ชื่ออะไร, ชอบสีอะไร), หรือการพูดคุยเล่นที่ไม่เกี่ยวกับข้อมูลจังหวัดน่าน
    - ถ้าผู้ใช้ชมว่า "เก่งมาก", "ขอบคุณ" ให้ถือเป็น SMALL_TALK
    3.  **PLAY_MUSIC:** สั่งเปิดเพลง หรือขอฟังเพลง
    4.  **SYSTEM_COMMAND:** สั่งงานระบบ (ตอนนี้อาจจะไม่ค่อยมี)
    5.  **WELCOME_GREETING:** คำทักทายแรกเริ่ม (เช่น สวัสดีคับ)
        """

        # **entity:**
        # - "PLAY_MUSIC" -> ชื่อเพลง/ศิลปิน
        # - "INFORMATIONAL" -> สถานที่ หรือ key word ที่ต้องการค้นหา
        # - "NAVIGATE_TO" -> สถานที่ปลายทาง
        # - "WELCOME_GREETING" -> null
        # - "SMALL_TALK" -> null

        # **sub_queries:** แตกคำถามเป็นข่อยๆ เพื่อค้นหาใน RAG (เฉพาะภาษาไทย)
        # **is_complex:** True ถ้าคำถามซับซ้อนต้องใช้หลาย steps หรือการวิเคราะห์สูง
        # **location_filter:** {"district": "อำเภอ...", "subdistrict": "ตำบล..."} (ถ้าระบุเจาะจง) (เช่น "วัดสวยๆ") ให้ส่ง `null`.
        # - อื่นๆ -> `null`

        # **category** (Dynamic):
        # - ระบุหมวดหมู่ภาษาอังกฤษตัวเล็ก เช่น: `accommodation`, `food`, `attraction`, `souvenir`, `culture`, `cafe`, `nature`.
        # - ถ้าไม่แน่ใจให้ `null`.
        # - **สำหรับอำเภอ:** ถ้าถาม "ในเมือง" -> `"district": "เมืองน่าน"`. ถามภาพรวมทั้งจังหวัด -> `"district": null`.

        # **ตัวอย่างการตัดสินใจ:**
        # Examples (for context in prompt):
        # "หิวข้าว แนะนำหน่อย" -> `intent: INFORMATIONAL`, `category: food`
        # "น่านมีอะไรน่าเที่ยว" -> `intent: INFORMATIONAL`, `category: attraction`
        # "เธอชื่ออะไร" -> `intent: SMALL_TALK`

        logging.info(f"✍️🧠 [Interpreter] Frontline returned fallback/RAG. Proceeding with default INFORMATIONAL intent for: '{corrected_query}'")
        
        # Determine intent for RAG
        # If Frontline explicitly said RAG_QUERY, we use that (or map to INFORMATIONAL)
        if f_intent == "RAG_QUERY":
             interpretation["intent"] = "INFORMATIONAL" # Map to what RAG expects
        
        logging.info(f"✅ [Interpreter] Final Routing: {interpretation['intent']}")
        return interpretation

    async def close(self):
        """Gracefully close resources."""
        if self.client:
            await self.client.close()
            logging.info("🧠 [Interpreter] Groq Client closed.")
        


