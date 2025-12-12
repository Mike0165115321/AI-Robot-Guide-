# Back-end/core/ai_models/llm_handler.py

import logging
import os
import json
from typing import List, Dict, Any, Optional
from groq import AsyncGroq
from core.config import settings
from core.ai_models.key_manager import groq_key_manager, gemini_key_manager

# 🔑 ไม่สร้าง client ล่วงหน้า - สร้างใหม่ทุกครั้งพร้อม key ที่หมุน
def _get_groq_client() -> AsyncGroq:
    """สร้าง Groq client ด้วย key ที่หมุนอัตโนมัติ"""
    api_key = groq_key_manager.get_key()
    if not api_key:
        raise RuntimeError("No Groq API keys available")
    masked = api_key[:8] + "..." + api_key[-4:]
    logging.info(f"🔑 [Groq] Using key: {masked}")
    return AsyncGroq(api_key=api_key)

MAX_RETRIES = 4  # ลองใหม่เท่ากับจำนวน keys

async def get_llm_response(
    messages: List[Dict[str, str]], 
    model_name: str = settings.GROQ_LLAMA_MODEL,
    temperature: float = 0.3,
    max_tokens: int = 1024,
    json_mode: bool = False
) -> str:
    """
    ฟังก์ชันกลางสำหรับเรียก LLM พร้อม Key Rotation
    รับ messages เป็น list ของ dict เช่น:
    [{"role": "system", "content": "..."}, {"role": "user", "content": "..."}]
    """
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
            return response.choices[0].message.content
            
        except Exception as e:
            last_error = e
            error_str = str(e).lower()
            
            # ถ้าเป็น rate limit error ให้ลอง key ถัดไป
            rate_limit_keywords = ["rate", "429", "quota", "exceeded", "limit", "exhausted"]
            is_rate_limit = any(keyword in error_str for keyword in rate_limit_keywords)
            
            if is_rate_limit:
                logging.warning(f"⚠️ [Groq] Rate limit hit, rotating key... (attempt {attempt + 1}/{MAX_RETRIES})")
                continue
            else:
                # Error อื่นๆ ไม่ต้อง retry
                logging.error(f"❌ [LLM Handler] Error calling Groq: {e}")
                break
    
    logging.error(f"❌ [LLM Handler] All retries failed: {last_error}")
    return "ขออภัยค่ะ ระบบ AI ขัดข้องชั่วคราว (LLM Error)"

async def get_llama_response_direct_async(user_query: str) -> str:
    """
    สำหรับ Small Talk / การสนทนาทั่วไป
    สำคัญ: มี system prompt ที่บอกให้ไม่ถามคำถามกลับ + Multi-language support
    """
    system_prompt = """คุณคือน้องน่าน ไกด์ท่องเที่ยวจังหวัดน่าน

🌐 กฎภาษา (สำคัญมาก!):
- ถ้าผู้ใช้ถามเป็นภาษาไทย → ตอบเป็นภาษาไทย ใช้ "ค่ะ"
- If user speaks English → Reply in English naturally
- 如果用户用中文 → 用中文回答

กฎอื่นๆ:
1. ตอบสั้นๆ กระชับ (2-3 ประโยค)
2. ส่งคำถามกลับเสมอเพื่อกระตุ้นให้ผู้ใช้ตอบกลับ
3. ถ้ามีคนบอกว่ามาจากที่ไหน ให้ต้อนรับอย่างอบอุ่น

ตัวอย่าง:
- "มาจากจีนครับ" → "ยินดีต้อนรับค่ะ! น่านมีวัฒนธรรมไทลื้อที่น่าสนใจนะคะ 🎉"
- "I'm from Japan" → "Welcome! Nan has beautiful temples and culture you'll love 🎉"
- "我来自中国" → "欢迎光临！南府有很多精彩的景点等您探索 🎉"
"""
    
    return await get_llm_response(
        [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_query}
        ],
        model_name=settings.GROQ_SMALL_TALK_MODEL,
        temperature=0.7 
    )

async def get_groq_rag_response_async(user_query: str, context: str, insights: str = "", turn_count: int = 1, ai_mode: str = "fast") -> Dict[str, Any]:
    """
    ai_mode: 'fast' = Groq/Llama, 'detailed' = Gemini
    พร้อม Multi-language support
    """
    # 🌐 Multi-language instruction
    language_rule = """🌐 กฎภาษา: ตอบในภาษาเดียวกับที่ผู้ใช้ถาม
- ภาษาไทย → ตอบภาษาไทย ใช้ "ค่ะ"
- English → Reply in English
- 中文 → 用中文回答"""
    
    system_msg = f"""คุณคือน้องน่าน ไกด์นำเที่ยวจังหวัดน่าน

{language_rule}

ข้อมูลอ้างอิง:
{context}"""
    
    if insights:
        system_msg += f"\n\nข้อมูลเพิ่มเติมจากสถิติ:\n{insights}"

    if ai_mode == "detailed":
        # Use Gemini for detailed responses
        response_text = await get_gemini_response_async(
            user_query=user_query,
            system_prompt=system_msg,
            max_tokens=2048  # Allow longer responses
        )
    else:
        # Use Groq/Llama for fast responses
        response_text = await get_llm_response([
            {"role": "system", "content": system_msg},
            {"role": "user", "content": user_query}
        ])
    
    return {"answer": response_text, "sources_used": []}

# ========================================
# Gemini Support for Detailed Mode
# ========================================
import google.generativeai as genai

# 🔑 ไม่ configure ล่วงหน้า - configure ใหม่ทุกครั้งพร้อม key ที่หมุน

async def get_gemini_response_async(
    user_query: str,
    system_prompt: str = "",
    model_name: str = "gemini-2.5-flash",
    max_tokens: int = 2048
) -> str:
    """
    ใช้ Gemini สำหรับ detailed mode - คำตอบยาวและละเอียดกว่า
    พร้อม Key Rotation เมื่อเจอ Rate Limit
    """
    import asyncio
    last_error = None
    
    for attempt in range(MAX_RETRIES):
        try:
            # 🔑 หมุน key ทุกครั้งที่เรียก
            api_key = gemini_key_manager.get_key()
            if not api_key:
                raise RuntimeError("No Gemini API keys available")
            
            masked = api_key[:8] + "..." + api_key[-4:]
            logging.info(f"🔑 [Gemini] Using key: {masked}")
            
            genai.configure(api_key=api_key)
            model = genai.GenerativeModel(model_name)
            
            full_prompt = f"{system_prompt}\n\nคำถามผู้ใช้: {user_query}\n\nกรุณาตอบอย่างละเอียดและครบถ้วน:"
            
            # Run in thread pool since google-generativeai is synchronous
            response = await asyncio.to_thread(
                model.generate_content,
                full_prompt,
                generation_config=genai.types.GenerationConfig(
                    max_output_tokens=max_tokens,
                    temperature=0.7
                )
            )
            
            return response.text
            
        except Exception as e:
            last_error = e
            error_str = str(e).lower()
            
            # ถ้าเป็น rate limit/quota error ให้ลอง key ถัดไป
            # เพิ่มคำสำคัญหลายๆ แบบที่ Google API อาจส่งมา
            rate_limit_keywords = ["rate", "429", "quota", "resource", "exceeded", "limit", "exhausted"]
            is_rate_limit = any(keyword in error_str for keyword in rate_limit_keywords)
            
            if is_rate_limit:
                logging.warning(f"⚠️ [Gemini] Rate limit hit, rotating key... (attempt {attempt + 1}/{MAX_RETRIES})")
                continue
            else:
                # Error อื่นๆ ไม่ต้อง retry
                logging.error(f"❌ [Gemini] Error: {e}")
                break
    
    logging.error(f"❌ [Gemini] All retries failed: {last_error}")
    return f"ขออภัยค่ะ ระบบ AI ขัดข้องชั่วคราว (Gemini Error: {str(last_error)[:100]})"