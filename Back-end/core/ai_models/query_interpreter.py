import logging
import json
import asyncio
from groq import AsyncGroq
from typing import Dict, Any, Optional, List
from .key_manager import groq_key_manager
from core.config import settings

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
        "GREETING": {"intent": "SMALL_TALK", "entity": None, "is_complex": False, "sub_queries": [""]},
        "THANKS": {"intent": "SMALL_TALK", "entity": None, "is_complex": False, "sub_queries": [""]},
        "FAREWELL": {"intent": "SMALL_TALK", "entity": None, "is_complex": False, "sub_queries": [""]}
    }
    _QUERY_MAP = {
        "สวัสดี": "GREETING", "หวัดดี": "GREETING", "ดีครับ": "GREETING", "ดีค่ะ": "GREETING",
        "ขอบคุณ": "THANKS", "ขอบใจ": "THANKS", "ขอบคุณครับ": "THANKS", "ขอบคุณค่ะ": "THANKS",
        "ลาก่อน": "FAREWELL", "ไปแล้วนะ": "FAREWELL", "บ๊ายบาย": "FAREWELL",
    }

    _ENTITY_ALIASES = {
        "วัดภูมินทร์": "วัดภูมินทร์",
        "ปู่ม่านย่าม่าน": "ภาพกระซิบรักบันลือโลก",
        "ภาพกระซิบรัก": "ภาพกระซิบรักบันลือโลก",
        "ภาพกระซิบรักบันลือโลก": "ภาพกระซิบรักบันลือโลก",
        "ดอยเสมอดาว": "ดอยเสมอดาว",
        "วัดพระธาตุแช่แห้ง": "วัดพระธาตุแช่แห้ง",
    }
    _INTERNAL_KNOWLEDGE_BASE = {
        "วัดภูมินทร์": "วัดภูมินทร์เป็นวัดหลวงที่มีชื่อเสียงที่สุดในจังหวัดน่าน มีจุดเด่นคือพระอุโบสถจตุรมุขและภาพจิตรกรรมฝาผนังปู่ม่านย่าม่าน หรือภาพกระซิบรักบันลือโลกอันโด่งดังครับ",
        "ภาพกระซิบรักบันลือโลก": "เป็นภาพจิตรกรรมฝาผนังอันเลื่องชื่อภายในวิหารของวัดภูมินทร์ วาดโดยหนานบัวผัน แสดงภาพชายหญิงคู่หนึ่งกำลังกระซิบสนทนากันอย่างใกล้ชิด เป็นสัญลักษณ์ของความรักที่โรแมนติกครับ",
        "ดอยเสมอดาว": "ดอยเสมอดาวเป็นจุดชมวิวและกางเต็นท์ยอดนิยมในอุทยานแห่งชาติศรีน่าน ขึ้นชื่อเรื่องการชมทะเลหมอกในตอนเช้าและดูดาวที่สวยงามเต็มท้องฟ้าในตอนกลางคืนครับ",
        "วัดพระธาตุแช่แห้ง": "วัดพระธาตุแช่แห้งเป็นพระธาตุประจำเมืองน่านและเป็นพระธาตุประจำปีเถาะ มีองค์พระธาตุเป็นสีทองอร่ามสวยงาม เป็นศูนย์รวมความศรัทธาของชาวน่านครับ"
    }

    def __init__(self):
        self.model_to_use = settings.GROQ_LLAMA_MODEL
        api_key = groq_key_manager.get_key()
        if not api_key:
            logging.error("🚨 [Interpreter] CRITICAL: No Groq API key found on init.")
            self.client = None
        else:
            self.client = AsyncGroq(api_key=api_key)
        logging.info(f"🧠 Query Interpreter (V6.4 - Pre-correction) initialized with model: {self.model_to_use}")

    async def close(self):
        """Closes the AsyncGroq client."""
        if self.client:
            logging.info("⏳ [Interpreter] Closing Groq client...")
            try:
                await self.client.close()
                logging.info("✅ [Interpreter] Groq client closed.")
            except Exception as e:
                logging.error(f"❌ Error closing Groq client: {e}")

    def _normalize_query(self, query: str) -> str:
        """Strips whitespace and common Thai particles for matching."""
        q = query.strip().lower()
        particles = ["ครับ", "ค่ะ", "จ้ะ", "จ้า", "นะ", "หน่อย", "สิ"]
        for p in particles:
            if q.endswith(p):
                q = q[:-len(p)].strip()
        return q

    async def _get_groq_response(self, system_prompt: str, user_query: str) -> Optional[str]:
        if not self.client:
            logging.error("❌ [Interpreter] Groq client not initialized (No API key).")
            return None
        try:
            chat_completion = await self.client.chat.completions.create(
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_query}
                ],
                model=self.model_to_use,
                temperature=0.0,
                response_format={"type": "json_object"},
            )
            return chat_completion.choices[0].message.content.strip()
        except Exception as e:
            logging.error(f"❌ [Interpreter] Groq API Error: {e}", exc_info=True)
            return None

    async def interpret_and_route(self, query: str) -> Dict[str, Any]:
        original_query = query.strip()
        if not original_query:
            return {
                "corrected_query": "", "intent": "SMALL_TALK", "entity": None, 
                "is_complex": False, "sub_queries": [""]
            }

        normalized_for_correction = self._normalize_query(original_query)
        corrected_query = self._PRE_CORRECTION_MAP.get(normalized_for_correction, original_query)
        if corrected_query != original_query:
            logging.info(f"✅ [Interpreter] Pre-corrected: '{original_query}' -> '{corrected_query}'")

        normalized_for_canned = self._normalize_query(corrected_query)
        if normalized_for_canned in self._QUERY_MAP:
            logging.info(f"✅ [Interpreter] Canned response for '{corrected_query}'")
            response_key = self._QUERY_MAP[normalized_for_canned]
            response = self._CANNED_RESPONSES[response_key].copy()
            response["corrected_query"] = corrected_query
            return response

        fallback_result = {
            "corrected_query": corrected_query, "intent": "INFORMATIONAL", "entity": None,
            "is_complex": False, "sub_queries": [corrected_query]
        }
        system_prompt = f"""You are an expert Thai language interpreter, router, and query decomposer for a Nan province tourism guide AI.
Your task is to analyze a noisy user query.
You MUST return a JSON object with exactly these 5 keys: "corrected_query", "intent", "entity", "is_complex", "sub_queries".

1.  **corrected_query**: Reconstruct the query into a clear, natural Thai sentence.
2.  **intent**: Classify into ONE: "INFORMATIONAL", "PLAY_MUSIC", "SYSTEM_COMMAND", "SMALL_TALK".
3.  **entity**: 
    - If "PLAY_MUSIC", extract song/artist.
    - If "SYSTEM_COMMAND", extract app name.
    - If "SMALL_TALK", return null.
    - If "INFORMATIONAL" AND `is_complex: true`, return null.
    - If "INFORMATIONAL" AND `is_complex: false`, extract the SINGLE main topic (e.g., "วัดภูมินทร์", "ปู่ม่านย่าม่าน", "ดอยเสมอดาว"). If no single topic, return null.
4.  **is_complex**: (Boolean) Is this a complex question that requires multiple separate information retrievals? 
    - `true` if it compares items (A vs B), asks for multiple distinct topics (A and B), or has sequential logic.
    - `false` if it's a simple, single-topic question.
5.  **sub_queries**: (List of strings)
    - If `is_complex: false`, return a list containing only the `corrected_query`.
    - If `is_complex: true`, break the `corrected_query` down into the simplest possible sub-queries.

**EXAMPLES (Crucial):**
* Input: "วัด พูมิน ไปไง"
  Output: {{"corrected_query": "วัดภูมินทร์ไปยังไง", "intent": "INFORMATIONAL", "entity": "วัดภูมินทร์", "is_complex": false, "sub_queries": ["วัดภูมินทร์ไปยังไง"]}}
* Input: "ขอดูรูปปู่ม่านย่าม่าน"
  Output: {{"corrected_query": "ขอดูรูปปู่ม่านย่าม่าน", "intent": "INFORMATIONAL", "entity": "ปู่ม่านย่าม่าน", "is_complex": false, "sub_queries": ["ขอดูรูปปู่ม่านย่าม่าน"]}}
* Input: "วัดสวยๆ ในน่านมีที่ไหนบ้าง"
  Output: {{"corrected_query": "วัดสวยๆ ในน่านมีที่ไหนบ้าง", "intent": "INFORMATIONAL", "entity": null, "is_complex": false, "sub_queries": ["วัดสวยๆ ในน่านมีที่ไหนบ้าง"]}}
* Input: "เปิดเพงเศร้าๆ น่อย"
  Output: {{"corrected_query": "เปิดเพลงเศร้าๆ หน่อย", "intent": "PLAY_MUSIC", "entity": "เพลงเศร้าๆ", "is_complex": false, "sub_queries": ["เปิดเพลงเศร้าๆ หน่อย"]}}
* Input: "สวัสดี"
  Output: {{"corrected_query": "สวัสดี", "intent": "SMALL_TALK", "entity": null, "is_complex": false, "sub_queries": ["สวัสดี"]}}
* Input: "วัดภูมินทร์กับวัดแช่แห้งต่างกันยังไง แล้ววัดไหนจอดรถง่ายกว่า?"
  Output: {{"corrected_query": "วัดภูมินทร์กับวัดพระธาตุแช่แห้งแตกต่างกันยังไง และวัดไหนมีที่จอดรถสะดวกกว่า?", "intent": "INFORMATIONAL", "entity": null, "is_complex": true, "sub_queries": ["เปรียบเทียบ วัดภูมินทร์ และ วัดพระธาตุแช่แห้ง", "ที่จอดรถ วัดภูมินทร์", "ที่จอดรถ วัดพระธาตุแช่แห้ง"]}}
* Input: "ประวัติศาสตร์น่าน และ ชนเผ่าที่น่าสนใจ"
  Output: {{"corrected_query": "ประวัติศาสตร์น่าน และ ชนเผ่าที่น่าสนใจ", "intent": "INFORMATIONAL", "entity": null, "is_complex": true, "sub_queries": ["ประวัติศาสตร์จังหวัดน่าน", "ชนเผ่าที่น่าสนใจในจังหวัดน่าน"]}}
"""
        
        logging.info(f"✍️🧠 [Interpreter] Interpreting with LLM: '{corrected_query}'")
        response_str = await self._get_groq_response(system_prompt, corrected_query)
        if not response_str:
            return fallback_result

        try:
            result = json.loads(response_str)
            if not all(k in result for k in ["corrected_query", "intent", "is_complex", "sub_queries"]):
                raise ValueError("Missing required keys")
            if "entity" not in result: result["entity"] = None

            if (result.get("intent") == "INFORMATIONAL" and not result.get("is_complex") and result.get("entity")):
                entity = result["entity"]
                canonical_entity = self._ENTITY_ALIASES.get(entity.strip().lower(), entity.strip().lower())
                
                if canonical_entity in self._INTERNAL_KNOWLEDGE_BASE:
                    logging.info(f"✅ [Interpreter] Found DIRECT ANSWER for entity '{canonical_entity}'")
                    result["direct_answer"] = self._INTERNAL_KNOWLEDGE_BASE[canonical_entity]

            logging.info(f"✅ [Interpreter] LLM Result: {result}")
            return result
        except Exception as e:
            logging.error(f"❌ [Interpreter] Failed to parse LLM JSON: {e}. Response: {response_str}")
            return fallback_result