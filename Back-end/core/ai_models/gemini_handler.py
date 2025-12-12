# Back-end/core/ai_models/gemini_handler.py
"""
Gemini AI Handler - สำหรับ Detailed Mode
พร้อม Key Rotation และ Retry Logic
"""

import logging
import asyncio
import google.generativeai as genai
from core.config import settings
from core.ai_models.key_manager import gemini_key_manager

MAX_RETRIES = 4  # ลองใหม่เท่ากับจำนวน keys

async def get_gemini_response(
    user_query: str,
    system_prompt: str = "",
    model_name: str = "gemini-2.5-flash",
    max_tokens: int = 8192
) -> str:
    """
    ใช้ Gemini สำหรับ detailed mode
    - คำตอบยาวและละเอียด
    - Key Rotation เมื่อเจอ Rate Limit
    - Multi-language support
    """
    last_error = None
    
    for attempt in range(MAX_RETRIES):
        try:
            # 🔑 หมุน key ทุกครั้งที่เรียก
            api_key = gemini_key_manager.get_key()
            if not api_key:
                raise RuntimeError("No Gemini API keys available")
            
            masked = api_key[:8] + "..." + api_key[-4:]
            logging.info(f"🔑 [Gemini Handler] Using key: {masked}")
            
            genai.configure(api_key=api_key)
            model = genai.GenerativeModel(model_name)
            
            full_prompt = f"{system_prompt}\n\nคำถามผู้ใช้: {user_query}"
            
            # Run in thread pool since google-generativeai is synchronous
            response = await asyncio.to_thread(
                model.generate_content,
                full_prompt,
                generation_config=genai.types.GenerationConfig(
                    max_output_tokens=max_tokens,
                    temperature=0.7
                )
            )
            
            logging.info(f"✅ [Gemini Handler] Response generated successfully")
            return response.text
            
        except Exception as e:
            last_error = e
            error_str = str(e).lower()
            
            # ตรวจจับ rate limit/quota error
            rate_limit_keywords = ["rate", "429", "quota", "exceeded", "limit", "exhausted", "resource"]
            is_rate_limit = any(keyword in error_str for keyword in rate_limit_keywords)
            
            if is_rate_limit:
                logging.warning(f"⚠️ [Gemini Handler] Rate limit hit, rotating key... (attempt {attempt + 1}/{MAX_RETRIES})")
                continue
            else:
                logging.error(f"❌ [Gemini Handler] Error: {e}")
                break
    
    logging.error(f"❌ [Gemini Handler] All retries failed: {last_error}")
    return f"ขออภัยค่ะ ระบบ Gemini ขัดข้องชั่วคราว กรุณาลองใหม่อีกครั้งค่ะ"
