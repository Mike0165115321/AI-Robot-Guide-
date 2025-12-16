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
            "is_complex": False, "sub_queries": [corrected_query],
            "location_filter": {} # New field
        }
        
        system_prompt = f"""คุณคือผู้เชี่ยวชาญด้านภาษาและการตีความเจตนา (Intent Classification) สำหรับระบบ AI แนะนำการท่องเที่ยวน่าน
หน้าทีของคุณคือวิเคราะห์ข้อความของผู้ใช้ (ซึ่งอาจมีคำผิดหรือความกำกวม)
คุณต้องตอบกลับเป็น JSON Object ที่มี 7 keys ดังนี้เท่านั้น: "corrected_query", "intent", "entity", "is_complex", "sub_queries", "location_filter", "category".

1.  **corrected_query**: เรียบเรียงประโยคใหม่ให้เป็นภาษาไทยที่ถูกต้อง เป็นธรรมชาติ และชัดเจน
2.  **intent**: ระบุเจตนา เพียง 1 อย่างจาก: "INFORMATIONAL", "PLAY_MUSIC", "SYSTEM_COMMAND", "SMALL_TALK", "WELCOME_GREETING".
3.  **entity**: 
    - ถ้าเป็น "PLAY_MUSIC", ให้ระบุชื่อเพลง/ศิลปิน
    - ถ้าเป็น "SYSTEM_COMMAND", ให้ระบุชื่อแอป
    - ถ้าเป็น "SMALL_TALK" หรือ "WELCOME_GREETING", ให้ส่งค่า null
    - ถ้าเป็น "INFORMATIONAL" และ `is_complex: true`, ให้ส่งค่า null
    - ถ้าเป็น "INFORMATIONAL" และ `is_complex: false`, ให้ระบุชื่อสถานที่หรือหัวข้อหลักเพียงหนึ่งเดียว (เช่น "วัดภูมินทร์", "ปู่ม่านย่าม่าน"). ถ้าไม่มีหัวข้อที่เจาะจง ให้ส่งค่า null
4.  **is_complex**: (Boolean) เป็นคำถามซับซ้อนที่ต้องค้นหาข้อมูลแยกกันหลายส่วนหรือไม่?
5.  **sub_queries**: (List of strings) แตกคำถามย่อย
6.  **location_filter**: (Object) ระบุขอบเขตพื้นที่ {{ "district": "...", "sub_district": "..." }}
7.  **category**: (String/Null) ระบุหมวดหมู่ของสถานที่ที่ผู้ใช้ต้องการค้นหา (Dynamic Category)
    - พยายามระบุเป็นคำภาษาอังกฤษตัวเล็ก (lowercase) เช่น: "accommodation" (ที่พัก/โรงแรม), "food" (อาหาร/เครื่องดื่ม), "attraction" (สถานที่ท่องเที่ยว), "souvenir" (ของฝาก), "culture" (วัฒนธรรม/ประเพณี)
    - ไม่ต้องยึดติดกับคำเหล่านี้ ถ้าเจอหมวดหมู่อื่นที่ชัดเจน ให้ระบุได้เลย (เช่น "cafe", "temple", "nature")
    - ถ้าไม่แน่ใจ หรือเป็นคำถามทั่วไป ให้ส่งค่า null
    - **สำคัญมาก:** กรณี "อำเภอ":
        - หากผู้ใช้ระบุ "ในน่าน" หรือ "จังหวัดน่าน" (หมายถึงภาพรวม) -> ให้ส่ง "district": null
        - หากผู้ใช้ระบุ "ในตัวเมือง", "อำเภอเมือง", "ในเมืองน่าน" หรือชื่อสถานที่ในตัวเมือง -> ให้ส่ง "district": "เมืองน่าน" (อย่าส่งว่า "น่าน" เฉยๆ)
        - ต้องตัดคำว่า "อำเภอ", "อ.", "เขต" ออกเสมอ (เช่น "อ.ปัว" -> "ปัว")

**ตัวอย่าง:**
* Input: "วัด พูมิน ไปไง"
Output: {{"corrected_query": "วัดภูมินทร์ไปยังไง", "intent": "INFORMATIONAL", "entity": "วัดภูมินทร์", "is_complex": false, "sub_queries": ["วัดภูมินทร์ไปยังไง"], "location_filter": {{"district": "เมืองน่าน"}}, "category": "attraction"}}

* Input: "ร้านกาแฟ ในน่าน"
Output: {{"corrected_query": "ร้านกาแฟในจังหวัดน่านแนะนำ", "intent": "INFORMATIONAL", "entity": null, "is_complex": false, "sub_queries": ["ร้านกาแฟ น่าน"], "location_filter": {{}}, "category": "cafe"}}

* Input: "ร้านกาแฟ แถวสันติสุข มีไรบ้าง"
Output: {{"corrected_query": "ร้านกาแฟแถวอำเภอสันติสุขมีที่ไหนบ้าง", "intent": "INFORMATIONAL", "entity": null, "is_complex": false, "sub_queries": ["ร้านกาแฟ อำเภอสันติสุข"], "location_filter": {{"district": "สันติสุข"}}, "category": "cafe"}}

* Input: "ขอที่พัก ในปัว ดีๆ"
Output: {{"corrected_query": "ขอที่พักในอำเภอปัว", "intent": "INFORMATIONAL", "entity": null, "is_complex": false, "sub_queries": ["ที่พักแนะนำ อำเภอปัว"], "location_filter": {{"district": "ปัว"}}, "category": "accommodation"}}
"""

        logging.info(f"✍️🧠 [Interpreter] กำลังวิเคราะห์ด้วย LLM โดยใช้ข้อความ: '{corrected_query}'")
        response_str = await self._get_groq_response(system_prompt, corrected_query)
        if not response_str:
            return fallback_result

        try:
            result = json.loads(response_str)
            # Relaxed validation: Check for essential keys
            if not all(k in result for k in ["corrected_query", "intent"]):
                 raise ValueError("Missing essential keys")
            
            # Normalize missing keys
            if "entity" not in result: result["entity"] = None
            if "is_complex" not in result: result["is_complex"] = False
            if "sub_queries" not in result: result["sub_queries"] = [result["corrected_query"]]
            if "location_filter" not in result: result["location_filter"] = {}
            if "category" not in result: result["category"] = None

            logging.info(f"✅ [Interpreter] ผลลัพธ์จาก LLM: {result}")
            return result
        except Exception as e:
            logging.error(f"❌ [Interpreter] ไม่สามารถแปลง JSON จาก LLM ได้: {e}. คำตอบที่ได้: {response_str}")
        


