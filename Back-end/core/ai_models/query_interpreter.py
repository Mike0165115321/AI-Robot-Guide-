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
            
        # 2. If Frontline returns RAG_QUERY (or fallback), ask Groq LLM for deep analysis
        logging.info(f"🧠 [Interpreter] Frontline fallback. Asking Groq ({self.model_to_use}) to analyze: '{corrected_query}'")
        
        system_prompt = """You are the 'Brain' of an AI Guide Robot named 'Nong Nan' (น้องน่าน).
Your job is to interpret the user's intent and extract structural data for the RAG system.

### OUTPUT FORMAT (JSON ONLY):
{
  "intent": "String",       // INFORMATIONAL, PLAY_MUSIC, NAVIGATE_TO, SMALL_TALK, WELCOME_GREETING, CALCULATE
  "entity": "String|null",  // Specific place/song/object mentioned (Clean text, no politeness particles)
  "category": "String|null",// attraction, accommodation, food, souvenir, culture, cafe, nature
  "sub_queries": ["Str"],   // Break down complex queries into searchable keywords (Thai)
  "corrected_query": "Str", // Fix typos if necessary
  "is_complex":Boolean,     // True if multi-step reasoning is needed
  "location_filter": {      // Extract district if mentioned
     "district": "String|null" // e.g., "เมืองน่าน", "ปัว"
  }
}

### INTENT RULES:
- **NAVIGATE_TO**: User wants to go somewhere, asks for route/map/location.
  - Query: "อยากไปวัดพระธาตุเขาน้อย รู้จักมั้ยครับ" -> intent: "NAVIGATE_TO", entity: "วัดพระธาตุเขาน้อย" (Cut 'อยากไป', 'รู้จักมั้ย')
  - Query: "พาไปร้านกาแฟหน่อย" -> intent: "NAVIGATE_TO", entity: "ร้านกาแฟ", category: "cafe"
  - Query: "วัดภูมินทร์อยู่ไหน" -> intent: "NAVIGATE_TO", entity: "วัดภูมินทร์"
- **INFORMATIONAL**: General knowledge, history, description, "what is it?".
  - Query: "วัดภูมินทร์สร้างเมื่อไหร่" -> intent: "INFORMATIONAL", entity: "วัดภูมินทร์"
  - Query: "แนะนำที่เที่ยวปัว" -> intent: "INFORMATIONAL", entity: null, location_filter: {"district": "ปัว"}, category: "attraction"
- **PLAY_MUSIC**: asking to play a song.
- **CALCULATE**: Math questions (e.g. 50*3)
- **SMALL_TALK**: Greeting, personal questions.

### ENTITY EXTRACTION RULES:
- STRICTLY REMOVE all action verbs (ไป, อยากไป, พาไป) and politeness particles (ครับ, ค่ะ, รู้จักไหม).
- Return ONLY the official name of the place/object.

### EXAMPLE:
User: "อยากไปดอยเสมอดาว รู้จักป่าวครับ"
JSON:
{
  "intent": "NAVIGATE_TO",
  "entity": "ดอยเสมอดาว",
  "category": "nature",
  "sub_queries": ["ดอยเสมอดาว", "ที่กางเต็นท์ดอยเสมอดาว"],
  "corrected_query": "อยากไปดอยเสมอดาว",
  "is_complex": false,
  "location_filter": {}
}
"""

        try:
            if not self.client:
                 raise Exception("Groq Client is not initialized")

            response = await self.client.chat.completions.create(
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": corrected_query}
                ],
                model=self.model_to_use,
                temperature=0.1, # Low temp for strict JSON
                response_format={"type": "json_object"}
            )
            
            content = response.choices[0].message.content
            interpretation = json.loads(content)
            
            # Fallback/Safety Check
            if not interpretation.get("intent"): interpretation["intent"] = "INFORMATIONAL"
            if not interpretation.get("sub_queries"): interpretation["sub_queries"] = [corrected_query]
            
            logging.info(f"✅ [Interpreter] Groq Analysis: {interpretation}")
            return interpretation

        except Exception as e:
            logging.error(f"❌ [Interpreter] Groq Interpretation Failed: {e}")
            # Fallback to simple logic
            return {
                "intent": "INFORMATIONAL",
                "corrected_query": corrected_query,
                "entity": None,
                "is_complex": False,
                "sub_queries": [corrected_query],
                "location_filter": {}
            }

    async def close(self):
        """Gracefully close resources."""
        if self.client:
            await self.client.close()
        


