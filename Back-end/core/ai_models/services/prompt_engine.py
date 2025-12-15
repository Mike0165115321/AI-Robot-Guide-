from typing import Dict, List
from core.services.language_detector import language_detector

class PromptEngine:
    def __init__(self):
        pass

    def build_rag_prompt(self, user_query: str, context: str, history: List[dict], ai_mode: str = "fast", is_low_confidence: bool = False) -> Dict[str, str]:
        """
        สร้าง Prompt สำหรับการตอบคำถามโดยใช้ข้อมูลอ้างอิง
        ai_mode: 'fast' = กระชับสำหรับ Llama, 'detailed' = ละเอียดสำหรับ Gemini
        is_low_confidence: ถ้า True แสดงว่าระบบค้นหาไม่เจอข้อมูลที่ตรงเป๊ะ ให้ AI ตอบอย่างระมัดระวัง
        
        ทุกอย่างโหลดจากไฟล์ .txt ตามภาษาที่ตรวจจับ
        """
        
        # 🌐 ตรวจจับภาษาจาก user query
        detected_lang = language_detector.detect(user_query)
        lang_info = language_detector.get_language_info(detected_lang)
        
        # โหลด persona prompt ตามภาษาและโมเดล
        if ai_mode == "detailed":
            prompt_file = "persona_gemini"
            model_name = "Gemini"
        else:
            prompt_file = "persona_groq"
            model_name = "Groq/Llama"
        
        persona = language_detector.get_prompt(prompt_file, detected_lang)
        
        # 🌐 โหลด instruction จากไฟล์ตามภาษา
        instruction = language_detector.get_prompt("instruction_rag", detected_lang)
        
        low_conf_warning = ""
        if is_low_confidence:
             # Warning text (Ideally should come from a file, but hardcoded for reliability first)
             warnings = {
                 "th": "\n**คำเตือนสำคัญ:** ข้อมูลใน Context อาจจะไม่ตรงกับคำถาม 100% (คะแนนความเหมือนต่ำ) กรุณาตอบอย่างระมัดระวัง แจ้งผู้ใช้ว่าไม่พบข้อมูลที่เจาะจง แต่ให้ข้อมูลที่ใกล้เคียงแทน หรือถามกลับเพื่อขอความชัดเจน",
                 "en": "\n**IMPORTANT WARNING:** The provided Context might not exactly match the question (low similarity score). Please answer cautiously. State that specific information was not found, but provide related info or ask for clarification.",
                 "zh": "\n**重要警告：** 提供的上下文可能与问题不完全匹配（相似度低）。请谨慎回答。说明未找到具体信息，但提供相关信息或要求澄清。",
                 "ja": "\n**重要な警告:** 提供されたコンテキストは、質問と完全に一致しない可能性があります（類似度が低い）。慎重に回答してください。具体的な情報が見つからなかったことを伝え、関連情報を提供するか、明確化を求めてください。",
             }
             low_conf_warning = warnings.get(detected_lang, warnings["en"])

        print(f"📝 ═══════════════════════════════════════════")
        print(f"📝 [PROMPT ENGINE] Language: {detected_lang} ({lang_info['name']})")
        print(f"📝 [PROMPT ENGINE] Persona: {prompt_file}.txt")
        print(f"📝 [PROMPT ENGINE] Instruction: instruction_rag.txt")
        print(f"📝 [PROMPT ENGINE] Low Confidence: {is_low_confidence}")
        print(f"📝 [PROMPT ENGINE] Model: {model_name}")
        print(f"📝 ═══════════════════════════════════════════")
        
        # จัดการ History (ใช้ภาษาตาม detected_lang)
        history_text = ""
        if history:
            recent_history = history[-3:]
            formatted_history = "\n".join([f"- {h['role'].upper()}: {h['content']}" for h in recent_history])
            # History label ตามภาษา
            history_labels = {
                "th": "ประวัติการสนทนาก่อนหน้า",
                "en": "Previous conversation",
                "zh": "之前的对话",
                "ja": "以前の会話",
                "hi": "पिछली बातचीत",
                "ru": "Предыдущий разговор",
                "ms": "Perbualan sebelumnya",
            }
            label = history_labels.get(detected_lang, "Previous conversation")
            history_text = f"{label}:\n{formatted_history}"

        # Build system prompt - persona + instruction + context + warning
        system_prompt = f"""
{persona}

{instruction}

# Context
{context}

{low_conf_warning}
"""

        # User prompt labels ตามภาษา
        question_labels = {
            "th": "คำถาม",
            "en": "Question",
            "zh": "问题",
            "ja": "質問",
            "hi": "प्रश्न",
            "ru": "Вопрос",
            "ms": "Soalan",
        }
        respond_labels = {
            "th": "ตอบในฐานะน้องน่าน",
            "en": "Respond as Nong Nan",
            "zh": "作为小南回答",
            "ja": "ノーンナーンとして答える",
            "hi": "नोंग नान के रूप में उत्तर दें",
            "ru": "Ответьте как Нонг Нан",
            "ms": "Jawab sebagai Nong Nan",
        }
        
        q_label = question_labels.get(detected_lang, "Question")
        r_label = respond_labels.get(detected_lang, "Respond as Nong Nan")
        
        user_prompt = f"""{history_text}\n\n{q_label}: "{user_query}"\n\n{r_label}:"""
        
        return {"system": system_prompt.strip(), "user": user_prompt.strip()}

    def build_navigation_prompt(self, location_name: str) -> str:
        # Navigation prompt (Thai default for now)
        return f"รับทราบค่ะ! น้องน่านจัดเตรียมพิกัดของ **{location_name}** ให้เรียบร้อยแล้ว \n\n🚗 กดปุ่มด้านล่างเพื่อเปิดแผนที่นำทางได้เลยนะคะ เดินทางปลอดภัยนะคะ!"