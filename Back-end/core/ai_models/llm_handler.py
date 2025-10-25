# /core/ai_models/llm_handler.py (V2 - The Ultimate Async Version)

import google.generativeai as genai
from groq import AsyncGroq 
import logging 
import asyncio 

from core.config import settings
from .key_manager import gemini_key_manager, groq_key_manager

async def get_llama_response_direct_async(user_query: str) -> str:
    """
    Handles simple small talk using a fast Llama model on Groq (Async).
    """
    api_key = groq_key_manager.get_key()
    if not api_key: 
        logging.error("[Small Talk] No Groq API key available.")
        return "ขออภัยค่ะ มีปัญหาในการเชื่อมต่อค่ะ"
        
    try:
        client = AsyncGroq(api_key=api_key)
        
        system_prompt_small_talk = """You are 'Nong Nan', a cheerful AI tour guide for Nan province, Thailand. Your task is to engage in simple, positive small talk.
- Keep responses very short (1-2 sentences).
- Always be friendly and polite.
- If appropriate, end with a gentle question to keep the conversation going.

Example:
User: สวัสดี
Your response: สวัสดีค่ะ! ยินดีที่ได้คุยกันนะคะ มีอะไรให้น้องน่านช่วยไหมคะ?
"""
        
        logging.info(f"💬 [Small Talk] Handling query with Llama-8B (Async): '{user_query}'")
        
        chat_completion = await client.chat.completions.create(
            messages=[
                {"role": "system", "content": system_prompt_small_talk},
                {"role": "user", "content": user_query}
            ],
            model="llama-3.1-8b-instant", 
            temperature=0.7, 
            max_tokens=100
        )
        response_text = chat_completion.choices[0].message.content
        logging.info("✅ [Small Talk] Received direct response from Groq (Async).")
        return response_text
        
    except Exception as e:
        logging.error(f"❌ [Small Talk] Error calling Groq API (Async): {e}", exc_info=True)
        return "ขออภัยค่ะ มีปัญหานิดหน่อยค่ะ"

# (!!) ฟังก์ชันนี้ยังเป็น Sync เหมือนเดิม เพราะมันทำงานกับ DB (MongoDB)
# RAGOrchestrator จะยังคงเรียกใช้ด้วย 'asyncio.to_thread' ซึ่งถูกต้องแล้ว
def get_insights_from_logs(log_collection) -> dict:
    if log_collection is None:
        return {}
    
    try:
        top_topics_cursor = log_collection.aggregate([
            {"$group": {"_id": "$primary_topic", "count": {"$sum": 1}}},
            {"$sort": {"count": -1}},
            {"$limit": 3}
        ])
        top_topics = [item["_id"] for item in top_topics_cursor if item.get("_id")]
        
        if top_topics:
            logging.info(f"📈 [Analytics] Top topics found: {top_topics}")
            return {"top_topics": top_topics}
            
        return {}
    except Exception as e:
        logging.error(f"❌ [Analytics] Failed to get insights: {e}", exc_info=True)
        return {}

def _generate_gemini_prompt(user_query: str, context: str, insights: dict) -> str:
    insights_text = "ตอนนี้ยังไม่มีข้อมูลเชิงลึกค่ะ"
    if top_topics := insights.get("top_topics"):
        top_topics_str = ", ".join(top_topics)
        insights_text = f"ข้อมูลล่าสุด: สถานที่ที่นักท่องเที่ยวคนอื่นๆ ถามถึงบ่อยที่สุดคือ {top_topics_str}"
    
    prompt = f"""
# ภารกิจ
คุณคือ "น้องน่าน" ไกด์ท้องถิ่นผู้เชี่ยวชาญประจำจังหวัดน่านที่มีนิสัยร่าเริง เป็นมิตร และให้ข้อมูลได้อย่างยอดเยี่ยม เป้าหมายของคุณคือการสร้างคำตอบที่ครบถ้วน มีโครงสร้างสวยงาม และน่าสนใจให้กับนักท่องเที่ยว

# ข้อมูลประกอบ
นี่คือข้อมูลที่ตรวจสอบแล้วเกี่ยวกับจังหวัดน่าน คุณ **ต้อง** ใช้ข้อมูลนี้เป็นหลักในการสร้างคำตอบเท่านั้น
---
{context}
---

# ข้อมูลเชิงลึกจากนักท่องเที่ยวคนอื่นๆ
{insights_text}

# คำสั่ง
จาก "ข้อมูลประกอบ" ที่ให้มา จงตอบ "คำถามของนักท่องเที่ยว" อย่างละเอียดและเป็นประโยชน์ที่สุด

# กฎการจัดรูปแบบคำตอบ
1.  **บุคลิกและน้ำเสียง:** รักษาน้ำเสียงที่อบอุ่น เป็นกันเอง ใช้สรรพนามแทนตัวเองว่า "น้องน่าน" และลงท้ายด้วย "ค่ะ" อย่างเป็นธรรมชาติ
2.  **โครงสร้าง:** หากนักท่องเที่ยวถามคำถามกว้างๆ ให้จัดกลุ่มคำตอบตามหมวดหมู่ แต่ **จงสร้างหัวข้อหมวดหมู่ (`### ...`) ก็ต่อเมื่อมีข้อมูลที่เกี่ยวข้องใน "ข้อมูลประกอบ" เท่านั้น** หากไม่มีข้อมูลสำหรับหมวดหมู่ใดเลย **ห้าม** สร้างหัวข้อของหมวดหมู่นั้นขึ้นมาเด็ดขาด หมวดหมู่ที่มีให้ใช้คือ `### 🏞️ สายธรรมชาติและวิวทิวทัศน์`, `### 🙏 สายวัดและประวัติศาสตร์`, `### 🍜 สายกินและวัฒนธรรม`
3.  **รายละเอียดสถานที่:** สำหรับแต่ละสถานที่ ให้ใช้รูปแบบ: รายการตัวเลข, ชื่อ **ตัวหนา**, และคำอธิบายแบบจุด (`*`)
4.  **การใช้ Markdown:** ใช้ **ตัวหนา** (`**ข้อความ**`) กับชื่อและจุดเด่นที่สำคัญ
5.  **กรณีไม่ทราบข้อมูล:** หาก **"ข้อมูลประกอบ" ทั้งหมด** ไม่มีเนื้อหาที่ตอบคำถามได้เลย ให้ตอบว่า: "ขออภัยค่ะ น้องน่านยังไม่มีข้อมูลเกี่ยวกับเรื่องนี้ในระบบเลยค่ะ"
6.  **การปิดท้าย:** จบด้วยการเชิญชวนให้ถามคำถามเพิ่มเติมเสมอ
7.  **การใช้ข้อมูลเชิงลึก:** สามารถนำ "ข้อมูลเชิงลึก" มาใช้สร้างบทสนทนาที่เป็นธรรมชาติได้ แต่ไม่ต้องพูดถึงทุกครั้ง ใช้เมื่อเข้ากับบริบทเท่านั้น เช่น "โอ้! วัดภูมินทร์เหรอคะ เป็นที่ที่นักท่องเที่ยวคนอื่นๆ ถามถึงบ่อยที่สุดเลยค่ะ!"

# คำถามของนักท่องเที่ยว
{user_query}

# คำตอบของคุณ (ในฐานะ 'น้องน่าน'):
"""
    return prompt.strip()

async def get_gemini_response_async(user_query: str, context: str, insights: dict) -> str:
    api_key = gemini_key_manager.get_key()
    if not api_key:
        logging.error("[LLM] No Gemini API key available.")
        return "เกิดข้อผิดพลาด: ไม่ได้ตั้งค่า Gemini API Key"
        
    try:
        genai.configure(api_key=api_key)
        model = genai.GenerativeModel(settings.GEMINI_MODEL)
        
        full_prompt = _generate_gemini_prompt(user_query, context, insights)
        
        logging.info(f"🤖 [LLM] Calling Gemini API (Async) using key '...{api_key[-4:]}'")
        
        response = await model.generate_content_async(full_prompt)
        
        logging.info("✅ [LLM] Received response from Gemini (Async).")
        
        if not response.parts:
            logging.warning("[LLM] Gemini response was blocked or empty.")
            return "ขออภัยค่ะ มีปัญหาในการสร้างคำตอบ ลองถามใหม่อีกครั้งนะคะ"

        return response.text
    except Exception as e:
        logging.error(f"❌ [LLM] Error calling Gemini API (Async): {e}", exc_info=True)
        return "ขออภัยค่ะ เกิดข้อผิดพลาดในการเชื่อมต่อกับระบบ AI หลัก"