# Back-end/core/ai_models/groq_handler.py
"""
Groq AI Handler (Llama) - สำหรับ Fast Mode
พร้อม Key Rotation, Retry Logic และ Multi-language Support
"""

import logging
from typing import List, Dict, Any
from groq import AsyncGroq
from core.config import settings
from core.ai_models.key_manager import groq_key_manager

MAX_RETRIES = 4  # ลองใหม่เท่ากับจำนวน keys

def _get_groq_client() -> AsyncGroq:
    """สร้าง Groq client ด้วย key ที่หมุนอัตโนมัติ"""
    api_key = groq_key_manager.get_key()
    if not api_key:
        raise RuntimeError("No Groq API keys available")
    masked = api_key[:8] + "..." + api_key[-4:]
    logging.info(f"🔑 [Groq Handler] กำลังใช้คีย์: {masked}")
    return AsyncGroq(api_key=api_key)


async def get_groq_response(
    messages: List[Dict[str, str]], 
    model_name: str = None,
    temperature: float = 0.3,
    max_tokens: int = 1024,
    json_mode: bool = False
) -> str:
    """
    ฟังก์ชันกลางสำหรับเรียก Groq (Llama)
    พร้อม Key Rotation เมื่อเจอ Rate Limit
    """
    if model_name is None:
        model_name = settings.GROQ_LLAMA_MODEL
    
    last_error = None
    
    for attempt in range(MAX_RETRIES):
        try:
            groq_client = _get_groq_client()
            
            kwargs = {
                "model": model_name,
                "messages": messages,
                "temperature": temperature,
                "max_tokens": max_tokens,
            }
            if json_mode:
                kwargs["response_format"] = {"type": "json_object"}

            response = await groq_client.chat.completions.create(**kwargs)
            logging.info(f"✅ [Groq Handler] สร้างคำตอบสำเร็จ")
            return response.choices[0].message.content
            
        except Exception as e:
            last_error = e
            error_str = str(e).lower()
            
            # ตรวจจับ rate limit error
            rate_limit_keywords = ["rate", "429", "quota", "exceeded", "limit", "exhausted"]
            is_rate_limit = any(keyword in error_str for keyword in rate_limit_keywords)
            
            if is_rate_limit:
                logging.warning(f"⚠️ [Groq Handler] ติด Rate limit, กำลังหมุนคีย์... (รอบที่ {attempt + 1}/{MAX_RETRIES})")
                continue
            else:
                logging.error(f"❌ [Groq Handler] เกิดข้อผิดพลาด: {e}")
                break
    
    logging.error(f"❌ [Groq Handler] การลองใหม่ทั้งหมดล้มเหลว: {last_error}")
    return f"ขออภัยค่ะ ระบบ Groq ขัดข้องชั่วคราว ({str(last_error)[:50]})"


async def get_small_talk_response(user_query: str) -> str:
    """
    สำหรับ Small Talk / การสนทนาทั่วไป
    ใช้ Language Detector ตรวจจับภาษาและโหลด persona prompt
    """
    from core.services.language_detector import language_detector
    
    # ตรวจจับภาษาจาก user query
    detected_lang = language_detector.detect(user_query)
    lang_info = language_detector.get_language_info(detected_lang)
    
    # โหลด persona prompt ตามภาษา (ใช้ persona_groq เพราะ fast mode)
    persona = language_detector.get_prompt("persona_groq", detected_lang)
    
    print(f"💬 ═══════════════════════════════════════════")
    print(f"💬 [SMALL TALK] ภาษา: {detected_lang} ({lang_info['name']})")  
    print(f"💬 [SMALL TALK] จะตอบกลับเป็นภาษา: {lang_info['name']}")
    print(f"💬 ═══════════════════════════════════════════")
    
    system_prompt = f"""{persona}

กฎสำหรับ Small Talk:
1. ตอบสั้นๆ กระชับ (2-3 ประโยค)
2. ถ้ามีคนบอกว่ามาจากที่ไหน ให้ต้อนรับอย่างอบอุ่น
3. เป็นมิตร น่ารัก
"""
    
    return await get_groq_response(
        [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_query}
        ],
        model_name=settings.GROQ_SMALL_TALK_MODEL,
        temperature=0.7 
    )
