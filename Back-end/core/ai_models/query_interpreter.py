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
**กฎการตัดสินใจ (Intent Definitions):**
1.  **INFORMATIONAL (สำคัญมาก):**
    - ใช้สำหรับ **ทุกคำถาม** ที่เกี่ยวกับจังหวัดน่าน, อากาศ, ร้านอาหาร, ที่พัก, สถานที่ท่องเที่ยว, ประวัติศาสตร์, วัฒนธรรม, การเดินทาง
    - แม้จะเป็นคำถามสั้นๆ เช่น "ที่นั่นสวยไหม", "มีกาแฟไหม", "หิวข้าว" ให้ถือเป็น INFORMATIONAL เพื่อให้ระบบค้นหาข้อมูลจริง
    - ห้ามใช้ SMALL_TALK กับคำถามที่ต้องการข้อมูลสถานที่หรือความรู้
2.  **SMALL_TALK:**
    - ใช้สำหรับ **การทักทายทั่วไป** (สวัสดี, สบายดีไหม), คำถามส่วนตัวเกี่ยวกับ AI (ชื่ออะไร, ชอบสีอะไร), หรือการพูดคุยเล่นที่ไม่เกี่ยวกับข้อมูลจังหวัดน่าน
    - ถ้าผู้ใช้ชมว่า "เก่งมาก", "ขอบคุณ" ให้ถือเป็น SMALL_TALK
3.  **PLAY_MUSIC:** สั่งเปิดเพลง หรือขอฟังเพลง
4.  **SYSTEM_COMMAND:** สั่งงานระบบ (ตอนนี้อาจจะไม่ค่อยมี)
5.  **WELCOME_GREETING:** คำทักทายแรกเริ่ม (เช่น สวัสดีคับ)

**entity:**
- "PLAY_MUSIC" -> ชื่อเพลง/ศิลปิน
- "INFORMATIONAL" -> **สำคัญ:**
    - `is_complex: true` -> `entity: null`
    - `is_complex: false` -> ระบุชื่อสถานที่/หัวข้อหลักเพียง 1 อย่าง (เช่น "วัดภูมินทร์"). ถ้าไม่เจาะจง (เช่น "วัดสวยๆ") ให้ส่ง `null`.
- อื่นๆ -> `null`

**category** (Dynamic):
- ระบุหมวดหมู่ภาษาอังกฤษตัวเล็ก เช่น: `accommodation`, `food`, `attraction`, `souvenir`, `culture`, `cafe`, `nature`.
- ถ้าไม่แน่ใจให้ `null`.
- **สำหรับอำเภอ:** ถ้าถาม "ในเมือง" -> `"district": "เมืองน่าน"`. ถามภาพรวมทั้งจังหวัด -> `"district": null`.

**ตัวอย่างการตัดสินใจ:**
* "หิวข้าว แนะนำหน่อย" -> `intent: INFORMATIONAL`, `category: food` (ไม่ใช่ Small Talk!)
* "น่านมีอะไรน่าเที่ยว" -> `intent: INFORMATIONAL`, `category: attraction`
* "เธอชื่ออะไร" -> `intent: SMALL_TALK`
* "อากาศร้อนไหม" -> `intent: INFORMATIONAL` (เกี่ยวกับสภาพอากาศน่าน)
* "รักนะจุ๊บๆ" -> `intent: SMALL_TALK`
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
        


