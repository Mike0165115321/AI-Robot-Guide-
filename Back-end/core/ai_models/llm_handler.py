# /core/ai_models/llm_handler.py (V5 - Hybrid Fallback System)

import google.generativeai as genai
from groq import AsyncGroq 
import logging 
import asyncio 
import json 
from typing import Dict, Any, Optional

from core.config import settings
from .key_manager import gemini_key_manager, groq_key_manager 

async def get_llama_response_direct_async(user_query: str) -> str:
    # (ส่วน Small Talk ใช้ Groq เหมือนเดิม ถ้าพังให้ตอบกลางๆ)
    api_key = groq_key_manager.get_key()
    if not api_key: 
        return "ช่วงนี้ระบบอินเทอร์เน็ตขัดข้องนิดหน่อยค่ะ รบกวนถามใหม่อีกครั้งนะคะ"
        
    try:
        client = AsyncGroq(api_key=api_key)
        system_prompt_small_talk = "You are 'Nong Nan', a cheerful AI tour guide. Keep responses short and friendly in Thai."
        chat_completion = await client.chat.completions.create(
            messages=[
                {"role": "system", "content": system_prompt_small_talk},
                {"role": "user", "content": user_query}
            ],
            model="llama-3.1-8b-instant",
            temperature=0.7, 
            max_tokens=100
        )
        return chat_completion.choices[0].message.content
    except Exception as e:
        logging.error(f"❌ [Small Talk] Groq Error: {e}")
        return "ตอนนี้น้องน่านมึนหัวนิดหน่อย ถามเรื่องเที่ยวเลยได้ไหมคะ?"

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
        return {"top_topics": top_topics} if top_topics else {}
    except Exception:
        return {}

def _generate_rag_prompts(user_query: str, context: str, insights: dict) -> Dict[str, str]:
    insights_text = "ตอนนี้ยังไม่มีข้อมูลเชิงลึกค่ะ"
    if top_topics := insights.get("top_topics"):
        top_topics_str = ", ".join(top_topics)
        insights_text = f"ข้อมูลล่าสุด: สถานที่ยอดฮิตคือ {top_topics_str}"
    
    system_prompt = f"""# ภารกิจ
คุณคือ "น้องน่าน" ไกด์ท้องถิ่นจังหวัดน่าน (AI) นิสัยร่าเริง เป็นกันเอง
เป้าหมาย: ตอบคำถามนักท่องเที่ยวโดย **ต้อง** อ้างอิงจาก "ข้อมูลประกอบ" (Context) เท่านั้น

# กฎเหล็ก
1.  **ตอบเป็น JSON เท่านั้น:** {{ "answer": "...", "sources_used": [...] }}
2.  **เนื้อหา:** ห้ามแต่งเรื่องเอง ใช้ข้อมูลจาก Context เท่านั้น ถ้าไม่มีข้อมูลให้ตอบว่าไม่ทราบ
3.  **Sources:** ใส่ชื่อสถานที่ (title) ที่นำมาใช้ตอบลงใน List "sources_used"

# ข้อมูลช่วยเสริม
{insights_text}

# รูปแบบ JSON Output
{{
"answer": "คำตอบของคุณ (Markdown)",
"sources_used": ["ชื่อสถานที่ 1", "ชื่อสถานที่ 2"]
}}
"""
    user_prompt = f"""# ข้อมูลประกอบ (Context)
---
{context}
---

# คำถาม: {user_query}
# คำตอบ JSON:
"""
    return {"system": system_prompt.strip(), "user": user_prompt.strip()}

async def _get_gemini_fallback(prompts: Dict[str, str]) -> dict:
    """ระบบสำรอง: เรียกใช้ Gemini เมื่อ Groq ล่ม"""
    api_key = gemini_key_manager.get_key()
    if not api_key:
        logging.error("❌ [Fallback] No Gemini API Key available.")
        return {"answer": "ขออภัยค่ะ ระบบ AI หลักขัดข้องและไม่มีกุญแจสำรอง", "sources_used": []}

    try:
        logging.info(f"🛡️ [Fallback] Switching to Gemini ({settings.GEMINI_MODEL})...")
        genai.configure(api_key=api_key)
        model = genai.GenerativeModel(settings.GEMINI_MODEL)
        
        # Gemini ชอบ Prompt รวมกัน
        full_prompt = f"{prompts['system']}\n\n{prompts['user']}"
        
        response = await asyncio.to_thread(
            model.generate_content,
            full_prompt,
            generation_config={"response_mime_type": "application/json"}
        )
        
        return json.loads(response.text)
    except Exception as e:
        logging.error(f"❌ [Fallback] Gemini Error: {e}")
        return {"answer": "ขออภัยค่ะ ระบบ AI ทั้งหลักและสำรองขัดข้องชั่วคราว", "sources_used": []}

async def get_groq_rag_response_async(user_query: str, context: str, insights: dict) -> dict:
    prompts = _generate_rag_prompts(user_query, context, insights)
    api_key = groq_key_manager.get_key()

    # 1. พยายามใช้ Groq ก่อน (Model หลัก)
    if api_key:
        try:
            client = AsyncGroq(api_key=api_key)
            logging.info(f"🤖 [LLM-RAG] Calling Groq ({settings.GROQ_LLAMA_MODEL})...")
            
            chat_completion = await client.chat.completions.create(
                messages=[
                    {"role": "system", "content": prompts["system"]},
                    {"role": "user", "content": prompts["user"]}
                ],
                model=settings.GROQ_LLAMA_MODEL, 
                temperature=0.3, 
                max_tokens=4096,
                response_format={"type": "json_object"},
            )
            return json.loads(chat_completion.choices[0].message.content)
            
        except Exception as e:
            logging.warning(f"⚠️ [LLM-RAG] Groq Failed: {e}")
            # ถ้าพัง ให้ลงไปทำ Fallback ด้านล่าง

    # 2. ถ้า Groq พัง หรือไม่มี Key -> ใช้ Gemini แทน
    return await _get_gemini_fallback(prompts)