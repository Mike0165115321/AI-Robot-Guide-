# /core/ai_models/llm_handler.py

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
        return "ช่วงนี้ระบบอินเทอร์เน็ตขัดข้องนิดหน่อยครับ รบกวนถามใหม่อีกครั้งนะครับ"
        
    try:
        client = AsyncGroq(api_key=api_key)
        system_prompt_small_talk = "You are 'Nong Nan', a helpful and knowledgeable AI tour guide for Nan province, Thailand. Reply in polite, natural Thai."
        chat_completion = await client.chat.completions.create(
            messages=[
                {"role": "system", "content": system_prompt_small_talk},
                {"role": "user", "content": user_query}
            ],
            model=settings.GROQ_SMALL_TALK_MODEL, 
            temperature=0.7, 
            max_tokens=150
        )
        return chat_completion.choices[0].message.content
    except Exception as e:
        logging.error(f"❌ [Small Talk] Groq Error: {e}")
        return "ตอนนี้ระบบมึนงงเล็กน้อย ขออภัยครับ ถามเรื่องเที่ยวต่อเลยได้ไหมครับ?"

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

def _generate_rag_prompts(user_query: str, context: str, insights: dict, turn_count: int) -> Dict[str, str]:
    insights_text = ""
    if top_topics := insights.get("top_topics"):
        top_topics_str = ", ".join(top_topics)
        insights_text = f"สถานที่ยอดฮิต: {top_topics_str}"
    
    outro_strategy = ""
    
    if turn_count > 0 and turn_count % 3 == 0:
        outro_strategy = """
    - **[คำสั่งพิเศษรอบนี้]:** ผู้ใช้ได้รับข้อมูลมาเยอะแล้วในรอบก่อนๆ ให้แสดงความใส่ใจเป็นพิเศษ
    - **ต้องปิดท้ายด้วยประโยคทำนองว่า:** "เสพข้อมูลมาเยอะแล้ว ถึงเวลาผ่อนคลายกันดีกว่าค่ะ 😆 ให้น้องน่าน **เปิดเพลง** เพราะๆ ให้ฟังมั้ยคะ?" (หรือปรับคำพูดให้น่ารักตามสไตล์น้องน่าน)"""
    else:
        outro_strategy = """
    - ปิดท้ายด้วยการเสนอความช่วยเหลือทั่วไป เช่น "ให้น้องน่านช่วย **นำทาง** ไปที่นี่เลยมั้ยคะ?" หรือเชียร์ให้ไปเที่ยว"""

    system_prompt = f"""# บทบาทของคุณ
คุณคือ **"น้องน่าน"** ไกด์สาวเจ้าถิ่น (AI) ที่รักการบริการ ใส่ใจ และคุยสนุก (ใช้ภาษาไทยกลางที่สุภาพและเป็นธรรมชาติ)

# โครงสร้างการตอบ (The Flow)
1.  **Intro:** ทักทายอย่างอบอุ่น แสดงความกระตือรือร้น
2.  **Content:** เล่าเรื่องราวสถานที่จาก [Context] ให้เห็นภาพ เชื่อมโยงกัน
    - **สำคัญ:** แทรก Tag รูปภาพ `{{{{IMAGE: ชื่อสถานที่}}}}` หลังจบแต่ละสถานที่
3.  **Outro (สรุป):** - สรุปสั้นๆ เชียร์สถานที่
    {outro_strategy}

# ตัวอย่างการตอบ
"ยินดีเลยค่ะ! ... (เนื้อหาเล่าเรื่องสถานที่) ...
{{{{IMAGE: ดอยเสมอดาว}}}}
... (เนื้อหาต่อ) ...

หวังว่าจะถูกใจนะคะ {("เสพข้อมูลมาเยอะแล้ว ถึงเวลาผ่อนคลายกันดีกว่าค่ะ 😆 ให้น้องน่าน **เปิดเพลง** ให้ฟังชิลๆ ดีมั้ยคะ?" if turn_count > 0 and turn_count % 3 == 0 else "อยากให้น้องน่าน **นำทาง** ไปที่ไหนบอกได้เลยค่ะ")}"

# ข้อมูลช่วยเสริม
{insights_text}

# รูปแบบ JSON Output
{{
"answer": "คำตอบของคุณ (Markdown พร้อม Tag รูปภาพ)",
"sources_used": ["ชื่อสถานที่ 1", "ชื่อสถานที่ 2"]
}}
"""
    
    user_prompt = f"""# ข้อมูลประกอบ (Context)
---
{context}
---

# คำถาม: "{user_query}"

**คำสั่ง:** เล่าเรื่องให้น่าสนใจ แทรกรูปภาพ และ **ปิดท้ายตามกลยุทธ์รอบที่ {turn_count}**

# คำตอบ JSON:
"""
    return {"system": system_prompt.strip(), "user": user_prompt.strip()}

async def _get_gemini_fallback(prompts: Dict[str, str]) -> dict:
    api_key = gemini_key_manager.get_key()
    if not api_key:
        return {"answer": "ขออภัยค่ะ ระบบขัดข้องชั่วคราว", "sources_used": []}

    try:
        logging.info(f"🛡️ [Fallback] Gemini ({settings.GEMINI_MODEL})...")
        genai.configure(api_key=api_key)
        model = genai.GenerativeModel(settings.GEMINI_MODEL)
        full_prompt = f"{prompts['system']}\n\n{prompts['user']}"
        response = await asyncio.to_thread(
            model.generate_content,
            full_prompt,
            generation_config={"response_mime_type": "application/json"}
        )
        return json.loads(response.text)
    except Exception as e:
        logging.error(f"❌ [Fallback] Gemini Error: {e}")
        return {"answer": "ขออภัยค่ะ ระบบมีปัญหาในการประมวลผล", "sources_used": []}

async def get_groq_rag_response_async(user_query: str, context: str, insights: dict, turn_count: int = 1) -> dict:
    
    # ส่ง turn_count ต่อไปให้ตัวสร้าง Prompt
    prompts = _generate_rag_prompts(user_query, context, insights, turn_count)
    
    api_key = groq_key_manager.get_key()

    if api_key:
        try:
            client = AsyncGroq(api_key=api_key)
            logging.info(f"🤖 [LLM-RAG] Calling Groq ({settings.GROQ_LLAMA_MODEL})... Turn: {turn_count}")
            
            chat_completion = await client.chat.completions.create(
                messages=[
                    {"role": "system", "content": prompts["system"]},
                    {"role": "user", "content": prompts["user"]}
                ],
                model=settings.GROQ_LLAMA_MODEL, 
                temperature=0.7,  
                max_tokens=4096,
                response_format={"type": "json_object"},
            )
            return json.loads(chat_completion.choices[0].message.content)
            
        except Exception as e:
            logging.warning(f"⚠️ [LLM-RAG] Groq Failed: {e}")

    return await _get_gemini_fallback(prompts)