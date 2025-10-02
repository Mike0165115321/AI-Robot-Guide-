# /core/ai_models/llm_handler.py

import google.generativeai as genai
from groq import Groq
from core.config import settings
from .key_manager import gemini_key_manager, groq_key_manager

# --- PROMPT สำหรับโหมดข้อความ (Text) - เน้นความละเอียด ---
def _generate_gemini_prompt(user_query: str, context: str) -> str:
    """
    สร้าง Prompt สำหรับ Gemini ที่ต้องการคำตอบละเอียดและครบถ้วน
    """
    prompt = f"""
คุณคือ "ไกด์นำเที่ยวอัจฉริยะ" ประจำจังหวัดน่าน ผู้มีความรู้ลึกซึ้งและเล่าเรื่องได้อย่างน่าฟัง

**ภารกิจ:**
จงใช้ "ข้อมูลประกอบ" ที่ได้รับ เพื่อสร้างคำตอบที่สมบูรณ์และเป็นประโยชน์ให้กับ "คำถาม" ของนักท่องเที่ยว

**กฎในการสร้างคำตอบ:**
1.  **ยึดตามข้อมูลจริง:** ตอบโดยอ้างอิงจาก "ข้อมูลประกอบ" ที่ให้มาเป็นหลัก
2.  **เป็นธรรมชาติ:** เขียนในรูปแบบบทสนทนาที่ลื่นไหล เป็นกันเอง และให้ข้อมูลที่เป็นประโยชน์
3.  **ครบถ้วน:** ตอบคำถามให้ตรงประเด็นและครอบคลุมที่สุดเท่าที่ข้อมูลจะเอื้ออำนวย
4.  **ถ้าไม่รู้ ให้บอกว่าไม่รู้:** หากข้อมูลประกอบไม่มีเนื้อหาที่ตอบคำถามได้ ให้ตอบอย่างสุภาพว่า "ขออภัยค่ะ ข้อมูลเกี่ยวกับเรื่องนี้ยังไม่มีในระบบ"

---
**[ข้อมูลประกอบ]**
{context}
---

**[คำถาม]**
{user_query}

**[คำตอบจากไกด์นำเที่ยว]**
"""
    return prompt.strip()


# --- PROMPT สำหรับโหมดเสียง (Voice) - เน้นความเร็วและกระชับ ---
SYSTEM_PROMPT_FOR_VOICE = """
คุณคือ 'AI คู่หูสุดเฟรนด์ลี่' ที่พร้อมคุยทุกเรื่อง เป็นมิตรและมีพลังงานบวก (Positive Energy) ในการสนทนา

**กฎของคุณ:**
1. ใช้ภาษาพูดที่ฟังดูเป็นธรรมชาติ 
2. เน้นการใช้คำว่า **"คะ" หรือ "ค่ะ"** ที่เป็นกันเอง ไม่ใช่คำสุภาพที่เนิบนาบ
3. ตอบให้ **สั้น, กระชับ, และมีชีวิตชีวา** ความยาวไม่ควรเกิน 2 ประโยค เพื่อให้การสนทนาไม่สะดุด
4. พยายามตอบจาก 'ข้อมูล' ที่มีให้ก่อนเสมอ
5. **ทุกครั้งที่ตอบ ให้จบด้วยการแสดงความเห็นสั้นๆ หรือคำถามง่ายๆ ที่เกี่ยวข้อง** เพื่อโยนให้ผู้ใช้คุยต่อ (Ball-throwing technique)
6. หากข้อมูลไม่พอ ให้พูดเชิงว่า ' เรื่องนี้ยังไม่ได้อัปเดตข้อมูลมาเลยค่ะ' หรือ 'อืม... เรื่องนี้ฉันยังไม่รู้เลยน้า' เพื่อให้ดูเป็นธรรมชาติ
""".strip()

def _generate_user_prompt_for_voice(user_query: str, context: str) -> str:
    """
    สร้าง User Prompt ที่มีเฉพาะข้อมูลและคำถามสำหรับโหมดเสียง
    """
    return f"""
**ข้อมูล:**
{context}

**คำถาม:**
{user_query}
""".strip()



def get_gemini_response(user_query: str, context: str) -> str:
    api_key = gemini_key_manager.get_key()
    if not api_key:
        return "เกิดข้อผิดพลาด: ไม่ได้ตั้งค่า Gemini API Key"
        
    try:
        genai.configure(api_key=api_key)
        model = genai.GenerativeModel(settings.GEMINI_MODEL)
        
        full_prompt = _generate_gemini_prompt(user_query, context)
        
        print(f"🤖 [LLM] Calling Gemini API using key ending with '...{api_key[-4:]}'")
        response = model.generate_content(full_prompt)
        print("✅ [LLM] Received response from Gemini.")
        return response.text
    except Exception as e:
        print(f"❌ [LLM] Error calling Gemini API: {e}")
        return "ขออภัยค่ะ เกิดข้อผิดพลาดในการเชื่อมต่อกับระบบ AI หลัก"


def get_llama_response(user_query: str, context: str) -> str:
    api_key = groq_key_manager.get_key()
    if not api_key:
        return "เกิดข้อผิดพลาด: ไม่ได้ตั้งค่า Groq API Key"
        
    try:
        client = Groq(api_key=api_key)
        model_to_use = settings.GROQ_LLAMA_MODEL
        user_content = _generate_user_prompt_for_voice(user_query, context)
        
        print(f"🦙 [LLM] Calling Groq API (Model: {model_to_use}) for VOICE MODE using key '...{api_key[-4:]}'")
        
        chat_completion = client.chat.completions.create(
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT_FOR_VOICE},
                {"role": "user", "content": user_content}
            ],
            model=model_to_use,
        )
        
        response_text = chat_completion.choices[0].message.content
        print("✅ [LLM] Received response from Groq.")
        return response_text
        
    except Exception as e:
        print(f"❌ [LLM] Error calling Groq API: {e}")
        return "ขออภัยค่ะ เกิดข้อผิดพลาดในการเชื่อมต่อกับระบบ AI สนทนา"