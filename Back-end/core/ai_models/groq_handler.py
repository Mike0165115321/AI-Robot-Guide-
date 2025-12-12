# Back-end/core/ai_models/groq_handler.py
"""
Groq AI Handler (Llama) - สำหรับ Fast Mode
แยกออกมาจาก llm_handler.py เพื่อให้จัดการง่าย
"""

import logging
from typing import List, Dict, Any
from groq import AsyncGroq
from core.config import settings

# Initialize Groq client
groq_client = AsyncGroq(api_key=settings.GROQ_API_KEYS[0] if settings.GROQ_API_KEYS else None)


async def get_groq_response(
    messages: List[Dict[str, str]], 
    model_name: str = None,
    temperature: float = 0.3,
    max_tokens: int = 1024,
    json_mode: bool = False
) -> str:
    """
    ฟังก์ชันกลางสำหรับเรียก Groq (Llama)
    """
    if model_name is None:
        model_name = settings.GROQ_LLAMA_MODEL
        
    try:
        kwargs = {
            "model": model_name,
            "messages": messages,
            "temperature": temperature,
            "max_tokens": max_tokens,
        }
        if json_mode:
            kwargs["response_format"] = {"type": "json_object"}

        response = await groq_client.chat.completions.create(**kwargs)
        logging.info(f"✅ [Groq] Response generated successfully")
        return response.choices[0].message.content
        
    except Exception as e:
        logging.error(f"❌ [Groq] Error: {e}")
        return f"ขออภัยค่ะ ระบบ Groq ขัดข้องชั่วคราว ({str(e)[:50]})"


async def get_small_talk_response(user_query: str) -> str:
    """
    สำหรับ Small Talk / การสนทนาทั่วไป
    ใช้ model ที่เร็วกว่า
    """
    system_prompt = """คุณคือน้องน่าน ไกด์ท่องเที่ยวจังหวัดน่าน พูดภาษาไทย เป็นกันเอง

กฎสำคัญ:
1. ตอบสั้นๆ กระชับ (2-3 ประโยค)
2. อย่าถามคำถามกลับ - แค่ตอบให้เป็นมิตรแล้วจบ
3. ถ้ามีคนบอกว่ามาจากที่ไหน ให้ต้อนรับอย่างอบอุ่น
4. อย่าถามว่า "มาจากไหน" หรือ "สนใจอะไร" ซ้ำอีก

ตัวอย่างที่ดี:
- "มาจากจีนครับ" → "ยินดีต้อนรับค่ะ! น่านมีวัฒนธรรมไทลื้อที่น่าสนใจนะคะ 🎉"
- "มาจากกรุงเทพ" → "ยินดีต้อนรับค่ะ! หนีมากรุงเทพมาพักผ่อนที่น่านนิดนึงนะคะ 😊"
"""
    
    return await get_groq_response(
        [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_query}
        ],
        model_name=settings.GROQ_SMALL_TALK_MODEL,
        temperature=0.7 
    )
