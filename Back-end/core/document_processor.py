# /core/document_processor.py (ฉบับอัปเดตตาม config.py ของคุณ)

import io
import json
from PIL import Image
import pytesseract
import google.generativeai as genai
from PyPDF2 import PdfReader
from core.config import settings

class DocumentProcessor:
    def __init__(self):
        try:
            if not settings.GEMINI_API_KEYS:
                raise ValueError("GEMINI_API_KEYS list is empty in the configuration.")

            api_key = settings.GEMINI_API_KEYS[0]
            genai.configure(api_key=api_key)
            
            self.model = genai.GenerativeModel(settings.GEMINI_MODEL)
            
            print(f"✅ Successfully configured Gemini API with model '{settings.GEMINI_MODEL}'.")
        except Exception as e:
            print(f"❌ Failed to configure Gemini API: {e}")
            self.model = None

    def _read_pdf(self, file_content: bytes) -> str:
        """ดึงข้อความจากไฟล์ PDF"""
        text = ""
        try:
            pdf_reader = PdfReader(io.BytesIO(file_content))
            for page in pdf_reader.pages:
                text += page.extract_text() or ""
        except Exception as e:
            print(f"Error reading PDF: {e}")
        return text

    def _read_image(self, file_content: bytes) -> str:
        text = ""
        try:
            image = Image.open(io.BytesIO(file_content))
            text = pytesseract.image_to_string(image, lang='tha+eng')
        except Exception as e:
            print(f"Error reading image with OCR: {e}")
        return text

    def _extract_data_with_gemini(self, content: str) -> dict:
        if not self.model or not content.strip():
            return None

        prompt = f"""
[CONTEXT]
คุณคือระบบ AI สกัดข้อมูลอัตโนมัติ (Automated Data Extraction System) ที่ถูกออกแบบมาเพื่อแปลงข้อความดิบ (raw text) เกี่ยวกับจังหวัดน่านให้เป็น Structured JSON ที่แม่นยำ 100% ภารกิจของคุณคือการทำตามกฎที่กำหนดอย่างเคร่งครัดโดยไม่มีการตีความนอกเหนือคำสั่ง

[TASK]
วิเคราะห์เนื้อหาใน [DOCUMENT CONTENT] และสร้างผลลัพธ์เป็น JSON object ที่สอดคล้องกับ [OUTPUT SPECIFICATION] ทุกประการ

[OUTPUT SPECIFICATION]

1. JSON Structure:
{{
  "slug": "string", 
  "category": "string",
  "topic": "string",
  "title": "string",
  "summary": "string",
  "keywords": ["string", "string", ...],
  "details": [
      {{ "heading": "string", "content": "string" }}
  ]
}}

2. Field Definitions:
-   `slug`: (String) ต้องสร้าง Human-readable slug (key) ที่ไม่ซ้ำกันสำหรับเอกสารนี้ Slug ต้องเป็น `kebab-case` (ตัวพิมพ์เล็กทั้งหมด, ใช้ขีดกลาง `-` แทนช่องว่าง) และอิงจาก `title` หรือ `topic` (เช่น `wat-phumin-nan-legend` หรือ `krua-khun-yai-pua`)
-   `category`: (String) ต้องเลือกหมวดหมู่ที่เหมาะสมที่สุด **เพียงหนึ่งเดียวเท่านั้น** จากรายการต่อไปนี้: 'ประวัติศาสตร์', 'วัฒนธรรมและประเพณี', 'สถานที่สำคัญ', 'กลุ่มชาติพันธุ์และวิถีชีวิต', 'บุคคลสำคัญ', 'ภูมิศาสตร์และธรรมชาติ', 'เศรษฐกิจและสินค้าท้องถิ่น' การตัดสินใจต้องอิงจากแก่นของเนื้อหาเป็นหลัก
-   `topic`: (String) ระบุหัวข้อที่เฉพาะเจาะจงที่สุดของเอกสาร (เช่น 'วัดภูมินทร์', 'ประเพณีแข่งเรือ', 'ผ้าทอลายน้ำไหล')
-   `title`: (String) สกัดชื่อเรื่องหลักที่ครอบคลุมที่สุดจากเอกสาร หากไม่มี ให้สร้างชื่อที่เหมาะสมจากเนื้อหา
-   `summary`: (String) **สรุปใจความสำคัญทั้งหมดของเอกสารให้ครอบคลุมที่สุด โดยเขียนเป็น 1-2 ย่อหน้าที่กระชับ (ประมาณ 5-7 ประโยค) และต้องเก็บรักษาเนื้อหาที่เป็นข้อเท็จจริง (Facts) และใจความหลัก (Core Message) ไว้ให้มากที่สุด**
-   `keywords`: (Array of Strings) ระบุคำสำคัญ 3-5 คำ ที่เป็นตัวแทนของเนื้อหาและมีประโยชน์ต่อการค้นหา
-   `details`: (Array of Objects) สกัดเนื้อหาโดยละเอียดตามโครงสร้างหัวข้อในเอกสารต้นฉบับ
    -   `heading`: (String) ชื่อหัวข้อย่อยที่ปรากฏในเอกสาร
    -   `content`: (String) เนื้อหาทั้งหมดภายใต้หัวข้อย่อยนั้นๆ

[RULES]
1.  ต้องตอบกลับเป็น Valid JSON object เท่านั้น ห้ามมีข้อความอธิบายใดๆ นอกเหนือจากตัว JSON
2.  ค่าของ `category` ต้องมาจากลิสต์ที่กำหนดให้ใน Field Definitions เท่านั้น และเลือกได้เพียง 1 ค่า
3.  เนื้อหาใน `details.content` **ต้องเป็นการ "คัดลอกแบบคำต่อคำ" (Verbatim Copy) มาทั้งหมด ห้ามสรุปย่อหรือเปลี่ยนแปลงเนื้อหาเด็ดขาด**
4.  หากเอกสารไม่มีหัวข้อย่อยที่ชัดเจน ให้ใช้ `heading` ใน `details` เพียงอันเดียว โดยตั้งชื่อว่า "ภาพรวม" และใส่เนื้อหาทั้งหมดลงใน `content` **(โดยห้ามสรุปย่อตามกฎข้อ 3)**
5.  (กฎใหม่) ค่าของ `slug` ต้องเป็น `kebab-case` (lowercase, hyphens) ตามที่กำหนดไว้ใน Field Definitions

[DOCUMENT CONTENT]
---
{content}
---

โปรดดำเนินการสร้าง JSON ตามข้อมูลและกฎข้างต้น
"""

        try:
            response = self.model.generate_content(prompt)
            # (ส่วนนี้ครูขอเพิ่มการ .replace() ที่อาจจะหลุดมาด้วยครับ)
            cleaned_response = response.text.strip().replace("```json", "").replace("```", "").strip()
            extracted_data = json.loads(cleaned_response)
            return extracted_data
        except Exception as e:
            print(f"❌ Error communicating with Gemini or parsing JSON: {e}")
            print(f"Raw response from Gemini: {response.text if 'response' in locals() else 'N/A'}")
            return None

    def analyze_document(self, file_content: bytes, content_type: str) -> dict:
        extracted_text = ""
        if 'pdf' in content_type:
            extracted_text = self._read_pdf(file_content)
        elif 'image' in content_type:
            extracted_text = self._read_image(file_content)
        else:
            print(f"Unsupported content type: {content_type}")
            return None
        
        if not extracted_text:
            print("Could not extract any text from the document.")
            return None

        return self._extract_data_with_gemini(extracted_text)