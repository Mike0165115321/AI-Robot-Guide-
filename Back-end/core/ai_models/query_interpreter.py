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

    async def close(self):
        """Closes the AsyncGroq client."""
        if self.client:
            logging.info("⏳ [Interpreter] กำลังปิดการเชื่อมต่อ Groq...")
            try:
                await self.client.close()
                logging.info("✅ [Interpreter] ปิดการเชื่อมต่อ Groq เรียบร้อยแล้ว")
            except Exception as e:
                logging.error(f"❌ เกิดข้อผิดพลาดในการปิด Groq client: {e}")

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
            logging.error("❌ [Interpreter] Groq client (ไม่พบ API Key)")
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
            logging.error(f"❌ [Interpreter] เกิดข้อผิดพลาดกับ Groq API: {e}", exc_info=True)
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
            logging.info(f"✅ [Interpreter] แก้ไขคำผิดเบื้องต้น: '{original_query}' -> '{corrected_query}'")

        normalized_for_canned = self._normalize_query(corrected_query)
        if normalized_for_canned in self._QUERY_MAP:
            logging.info(f"✅ [Interpreter] ใช้คำตอบสำเร็จรูปสำหรับ '{corrected_query}'")
            response_key = self._QUERY_MAP[normalized_for_canned]
            response = self._CANNED_RESPONSES[response_key].copy()
            response["corrected_query"] = corrected_query
            return response

        fallback_result = {
            "corrected_query": corrected_query, "intent": "INFORMATIONAL", "entity": None,
            "is_complex": False, "sub_queries": [corrected_query]
        }
        
        system_prompt = f"""คุณคือผู้เชี่ยวชาญด้านภาษาและการตีความเจตนา (Intent Classification) สำหรับระบบ AI แนะนำการท่องเที่ยวน่าน
หน้าที่ของคุณคือวิเคราะห์ข้อความของผู้ใช้ (ซึ่งอาจมีคำผิดหรือความกำกวม)
คุณต้องตอบกลับเป็น JSON Object ที่มี 5 keys ดังนี้เท่านั้น: "corrected_query", "intent", "entity", "is_complex", "sub_queries".

1.  **corrected_query**: เรียบเรียงประโยคใหม่ให้เป็นภาษาไทยที่ถูกต้อง เป็นธรรมชาติ และชัดเจน
2.  **intent**: ระบุเจตนา เพียง 1 อย่างจาก: "INFORMATIONAL", "PLAY_MUSIC", "SYSTEM_COMMAND", "SMALL_TALK", "WELCOME_GREETING".
3.  **entity**: 
    - ถ้าเป็น "PLAY_MUSIC", ให้ระบุชื่อเพลง/ศิลปิน
    - ถ้าเป็น "SYSTEM_COMMAND", ให้ระบุชื่อแอป
    - ถ้าเป็น "SMALL_TALK" หรือ "WELCOME_GREETING", ให้ส่งค่า null
    - ถ้าเป็น "INFORMATIONAL" และ `is_complex: true`, ให้ส่งค่า null
    - ถ้าเป็น "INFORMATIONAL" และ `is_complex: false`, ให้ระบุชื่อสถานที่หรือหัวข้อหลักเพียงหนึ่งเดียว (เช่น "วัดภูมินทร์", "ปู่ม่านย่าม่าน"). ถ้าไม่มีหัวข้อที่เจาะจง ให้ส่งค่า null
4.  **is_complex**: (Boolean) เป็นคำถามซับซ้อนที่ต้องค้นหาข้อมูลแยกกันหลายส่วนหรือไม่?
    - `true` ถ้าเป็นการเปรียบเทียบ (A vs B), ถามหลายหัวข้อ (A และ B), หรือมีลำดับเหตุผล
    - `false` ถ้าเป็นคำถามหัวข้อเดียวง่ายๆ
5.  **sub_queries**: (List of strings)
    - ถ้า `is_complex: false`, ให้ใส่ `corrected_query` อันเดียวใน list
    - ถ้า `is_complex: true`, ให้แตก `corrected_query` ออกเป็นคำถามย่อยๆ ที่ง่ายที่สุดสำหรับการค้นหา

**ตัวอย่าง (สำคัญมาก):**
* Input: "วัด พูมิน ไปไง"
Output: {{"corrected_query": "วัดภูมินทร์ไปยังไง", "intent": "INFORMATIONAL", "entity": "วัดภูมินทร์", "is_complex": false, "sub_queries": ["วัดภูมินทร์ไปยังไง"]}}
* Input: "ขอดูรูปปู่ม่านย่าม่าน"
Output: {{"corrected_query": "ขอดูรูปปู่ม่านย่าม่าน", "intent": "INFORMATIONAL", "entity": "ปู่ม่านย่าม่าน", "is_complex": false, "sub_queries": ["ขอดูรูปปู่ม่านย่าม่าน"]}}
* Input: "วัดสวยๆ ในน่านมีที่ไหนบ้าง"
Output: {{"corrected_query": "วัดสวยๆ ในน่านมีที่ไหนบ้าง", "intent": "INFORMATIONAL", "entity": null, "is_complex": false, "sub_queries": ["วัดสวยๆ ในน่านมีที่ไหนบ้าง"]}}
* Input: "เปิดเพงเศร้าๆ น่อย"
Output: {{"corrected_query": "เปิดเพลงเศร้าๆ หน่อย", "intent": "PLAY_MUSIC", "entity": "เพลงเศร้าๆ", "is_complex": false, "sub_queries": ["เปิดเพลงเศร้าๆ หน่อย"]}}
* Input: "สวัสดี"
Output: {{"corrected_query": "สวัสดี", "intent": "WELCOME_GREETING", "entity": null, "is_complex": false, "sub_queries": ["สวัสดี"]}}
* Input: "เปิดเครื่องคิดเลข"
Output: {{"corrected_query": "เปิดเครื่องคิดเลข", "intent": "SYSTEM_COMMAND", "entity": "เครื่องคิดเลข", "is_complex": false, "sub_queries": ["เปิดเครื่องคิดเลข"]}}
* Input: "ช่วยเปิดโน้ตแพดให้หน่อย"
Output: {{"corrected_query": "เปิดโน้ตแพด", "intent": "SYSTEM_COMMAND", "entity": "โน้ตแพด", "is_complex": false, "sub_queries": ["เปิดโน้ตแพด"]}}
* Input: "วัดภูมินทร์กับวัดแช่แห้งต่างกันยังไง แล้ววัดไหนจอดรถง่ายกว่า?"
Output: {{"corrected_query": "วัดภูมินทร์กับวัดพระธาตุแช่แห้งแตกต่างกันยังไง และวัดไหนมีที่จอดรถสะดวกกว่า?", "intent": "INFORMATIONAL", "entity": null, "is_complex": true, "sub_queries": ["เปรียบเทียบ วัดภูมินทร์ และ วัดพระธาตุแช่แห้ง", "ที่จอดรถ วัดภูมินทร์", "ที่จอดรถ วัดพระธาตุแช่แห้ง"]}}
* Input: "ประวัติศาสตร์น่าน และ ชนเผ่าที่น่าสนใจ"
Output: {{"corrected_query": "ประวัติศาสตร์น่าน และ ชนเผ่าที่น่าสนใจ", "intent": "INFORMATIONAL", "entity": null, "is_complex": true, "sub_queries": ["ประวัติศาสตร์จังหวัดน่าน", "ชนเผ่าที่น่าสนใจในจังหวัดน่าน"]}}
"""

        logging.info(f"✍️🧠 [Interpreter] กำลังวิเคราะห์ด้วย LLM โดยใช้ข้อความ: '{corrected_query}'")
        response_str = await self._get_groq_response(system_prompt, corrected_query)
        if not response_str:
            return fallback_result

        try:
            result = json.loads(response_str)
            if not all(k in result for k in ["corrected_query", "intent", "is_complex", "sub_queries"]):
                raise ValueError("Missing required keys")
            if "entity" not in result: result["entity"] = None

            logging.info(f"✅ [Interpreter] ผลลัพธ์จาก LLM: {result}")
            return result
        except Exception as e:
            logging.error(f"❌ [Interpreter] ไม่สามารถแปลง JSON จาก LLM ได้: {e}. คำตอบที่ได้: {response_str}")
            return fallback_result
        


