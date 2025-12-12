# Back-end/core/ai_models/gemini_handler.py
"""
Gemini AI Handler - สำหรับ Detailed Mode
แยกออกมาจาก llm_handler.py เพื่อให้จัดการง่าย
"""

import logging
import asyncio
import google.generativeai as genai
from core.config import settings

# Configure Gemini
if settings.GEMINI_API_KEYS:
    genai.configure(api_key=settings.GEMINI_API_KEYS[0])


async def get_gemini_response(
    user_query: str,
    system_prompt: str = "",
    model_name: str = "gemini-2.5-flash",
    max_tokens: int = 8192
) -> str:
    """
    ใช้ Gemini สำหรับ detailed mode
    - คำตอบยาวและละเอียด
    - ไม่จำกัด tokens มาก
    """
    try:
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
        
        logging.info(f"✅ [Gemini] Response generated successfully")
        return response.text
        
    except Exception as e:
        logging.error(f"❌ [Gemini] Error: {e}")
        return f"ขออภัยค่ะ ระบบ Gemini ขัดข้องชั่วคราว กรุณาลองใหม่อีกครั้งค่ะ"
