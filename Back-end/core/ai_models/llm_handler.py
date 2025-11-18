# /core/ai_models/llm_handler.py (V4 - Smart Sources)

import google.generativeai as genai
from groq import AsyncGroq 
import logging 
import asyncio 
import json 
from typing import Dict, Any, Optional

from core.config import settings
from .key_manager import gemini_key_manager, groq_key_manager 

async def get_llama_response_direct_async(user_query: str) -> str:
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
        chat_completion = await client.chat.completions.create(
            messages=[
                {"role": "system", "content": system_prompt_small_talk},
                {"role": "user", "content": user_query}
            ],
            model="llama-3.1-8b-instant", # (ใช้โมเดลเล็กสำหรับ Small Talk)
            temperature=0.7, 
            max_tokens=100
        )
        response_text = chat_completion.choices[0].message.content
        logging.info("✅ [Small Talk] Received direct response from Groq (Async).")
        return response_text
        
    except Exception as e:
        logging.error(f"❌ [Small Talk] Error calling Groq API (Async): {e}", exc_info=True)
        return "ขออภัยค่ะ มีปัญหานิดหน่อยค่ะ"

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


def _generate_llama_rag_prompts(user_query: str, context: str, insights: dict) -> Dict[str, str]:
    insights_text = "ตอนนี้ยังไม่มีข้อมูลเชิงลึกค่ะ"
    if top_topics := insights.get("top_topics"):
        top_topics_str = ", ".join(top_topics)
        insights_text = f"ข้อมูลล่าสุด: สถานที่ที่นักท่องเที่ยวคนอื่นๆ ถามถึงบ่อยที่สุดคือ {top_topics_str}"
    
    system_prompt = f"""# ภารกิจ
คุณคือ "น้องน่าน" ไกด์ท้องถิ่นผู้เชี่ยวชาญประจำจังหวัดน่านที่มีนิสัยร่าเริง เป็นมิตร และให้ข้อมูลได้อย่างยอดเยี่ยม
เป้าหมายของคุณคือการสร้างคำตอบให้กับ "คำถามของนักท่องเที่ยว" โดย **ต้อง** อ้างอิงจาก "ข้อมูลประกอบ" (Context) ที่ให้มาเท่านั้น

# กฎการตอบ (สำคัญมาก)
1.  **ต้องตอบเป็น JSON เท่านั้น:** คำตอบของคุณ **ต้อง** เป็น JSON object ที่มี 2 key เท่านั้น: `answer` และ `sources_used`.
2.  **Key "answer" (String):**
    * สร้างคำตอบที่ครบถ้วน มีโครงสร้างสวยงาม (ใช้ Markdown) และน่าสนใจ
    * **ห้าม** แต่งข้อมูลหรือใช้ความรู้ภายนอก "ข้อมูลประกอบ" เด็ดขาด
    * บุคลิก: รักษาน้ำเสียงที่อบอุ่น เป็นกันเอง ใช้สรรพนาม "น้องน่าน" และลงท้าย "ค่ะ"
    * กรณีไม่ทราบ: หาก "ข้อมูลประกอบ" **ทั้งหมด** ไม่มีเนื้อหาที่ตอบคำถามได้เลย ให้ตอบว่า: "ขออภัยค่ะ น้องน่านยังไม่มีข้อมูลเกี่ยวกับเรื่องนี้ในระบบเลยค่ะ"
3.  **Key "sources_used" (List of Strings):**
    * นี่คือ List ของ "ชื่อสถานที่" (`title`) จาก "ข้อมูลประกอบ" ที่คุณ "เลือกใช้" ในการสร้างคำตอบ
    * **ต้อง** ใส่เฉพาะ `title` ของเอกสารที่คุณ "อ้างอิงถึง" ใน `answer` เท่านั้น
    * หาก `answer` ของคุณคือ "ขออภัยค่ะ..." (ไม่ทราบข้อมูล) ให้ key นี้เป็น List ว่าง: `[]`
    * หาก LLM (คือตัวคุณ) ตัดสินใจ "ไม่ใช้" บางเอกสาร (เช่น เสาดินนาน้อย) เพราะมัน "ไม่เกี่ยว" กับคำถาม... **ห้าม** ใส่ `title` นั้นลงใน List นี้เด็ดขาด

# ข้อมูลเชิงลึก (สำหรับช่วยในการสนทนา)
{insights_text} (คุณสามารถใช้ข้อมูลนี้สร้างบทสนทนาได้ เช่น "โอ้! วัดภูมินทร์เหรอคะ เป็นที่ที่นักท่องเที่ยวคนอื่นๆ ถามถึงบ่อยที่สุดเลยค่ะ!")

# รูปแบบ JSON ที่ต้องตอบกลับ (ห้ามมีข้อความอื่นนอก JSON นี้)
{{
"answer": "...",
"sources_used": ["(title ของเอกสารที่ 1 ที่ใช้)", "(title ของเอกสารที่ 2 ที่ใช้)"]
}}

จงตอบ "คำถามของนักท่องเที่ยว" โดยใช้ "ข้อมูลประกอบ" ที่จะให้มาใน user message
"""

    user_prompt = f"""# ข้อมูลประกอบ (Context)
---
{context}
---

# คำถามของนักท่องเที่ยว
{user_query}

# คำตอบของคุณ (ในฐานะ 'น้องน่าน' และต้องเป็น JSON เท่านั้น):
"""
    
    return {"system": system_prompt.strip(), "user": user_prompt.strip()}


async def get_groq_rag_response_async(user_query: str, context: str, insights: dict) -> dict:
    """
    (แก้ไข) ฟังก์ชันนี้จะคืนค่าเป็น Dictionary {"answer": ..., "sources_used": ...}
    """
    api_key = groq_key_manager.get_key()
    if not api_key:
        logging.error("[LLM-RAG] No Groq API key available.")
        return {"answer": "เกิดข้อผิดพลาด: ไม่ได้ตั้งค่า Groq API Key", "sources_used": []} # 👈 คืน dict
        
    try:
        prompts = _generate_llama_rag_prompts(user_query, context, insights)
        
        client = AsyncGroq(api_key=api_key)
        
        logging.info(f"🤖 [LLM-RAG] Calling Groq RAG API (Async) using Llama 70B (Smart Source Mode)...")
        
        chat_completion = await client.chat.completions.create(
            messages=[
                {"role": "system", "content": prompts["system"]},
                {"role": "user", "content": prompts["user"]}
            ],
            model=settings.GROQ_LLAMA_MODEL, 
            temperature=0.3, 
            max_tokens=4096,
            response_format={"type": "json_object"}, # 👈 บังคับ JSON
        )
        
        response_text = chat_completion.choices[0].message.content
        
        try:
            llm_response_dict = json.loads(response_text)
            if "answer" not in llm_response_dict or "sources_used" not in llm_response_dict:
                raise ValueError("Missing required keys")
            
            logging.info(f"✅ [LLM-RAG] Received valid JSON response. Sources used: {llm_response_dict.get('sources_used')}")
            return llm_response_dict

        except Exception as e:
            logging.error(f"❌ [LLM-RAG] Failed to parse JSON from LLM: {e}. Raw text: '{response_text[:100]}...'")
            return {"answer": response_text, "sources_used": None} 
    except Exception as e:
        logging.error(f"❌ [LLM-RAG] Error calling Groq 70B API (Async): {e}", exc_info=True)
        return {"answer": "ขออภัยค่ะ เกิดข้อผิดพลาดในการเชื่อมต่อกับระบบ AI หลัก", "sources_used": []}