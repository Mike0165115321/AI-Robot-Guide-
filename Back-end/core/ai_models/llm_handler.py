# Back-end/core/ai_models/llm_handler.py

import logging
import os
import json
from typing import List, Dict, Any, Optional
from groq import AsyncGroq
from core.config import settings

groq_client = AsyncGroq(api_key=settings.GROQ_API_KEYS[0])

async def get_llm_response(
    messages: List[Dict[str, str]], 
    model_name: str = settings.GROQ_LLAMA_MODEL,
    temperature: float = 0.3,
    max_tokens: int = 1024,
    json_mode: bool = False
) -> str:
    """
    ฟังก์ชันกลางสำหรับเรียก LLM
    รับ messages เป็น list ของ dict เช่น:
    [{"role": "system", "content": "..."}, {"role": "user", "content": "..."}]
    """
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
        return response.choices[0].message.content
    except Exception as e:
        logging.error(f"❌ [LLM Handler] Error calling Groq: {e}")
        return "ขออภัยค่ะ ระบบ AI ขัดข้องชั่วคราว (LLM Error)"

async def get_llama_response_direct_async(user_query: str) -> str:
    """
    สำหรับ Small Talk / การสนทนาทั่วไป
    สำคัญ: มี system prompt ที่บอกให้ไม่ถามคำถามกลับ
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
    
    return await get_llm_response(
        [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_query}
        ],
        model_name=settings.GROQ_SMALL_TALK_MODEL,
        temperature=0.7 
    )

async def get_groq_rag_response_async(user_query: str, context: str, insights: str = "", turn_count: int = 1) -> Dict[str, Any]:
    system_msg = f"คุณคือน้องน่าน ไกด์นำเที่ยว\nข้อมูลอ้างอิง:\n{context}"
    if insights:
        system_msg += f"\n\nข้อมูลเพิ่มเติมจากสถิติ:\n{insights}"

    response_text = await get_llm_response([
        {"role": "system", "content": system_msg},
        {"role": "user", "content": user_query}
    ])
    
    return {"answer": response_text, "sources_used": []}