# /core/services/ai_mapper_service.py
"""
AI Mapper Service: ใช้ Gemini AI แปลงข้อมูลดิบลง Target Fields
สำหรับ AI-Powered Smart ETL System
- รองรับ API Key Rotation เมื่อเจอ Quota Error (429)
- มี Retry Logic พร้อม Exponential Backoff
"""

import json
import asyncio
from typing import List, Dict, Any, Optional
import google.generativeai as genai
from core.config import settings
from core.ai_models.key_manager import gemini_key_manager


class AIMapperService:
    """
    Service สำหรับ AI-powered data transformation
    ใช้ Gemini วิเคราะห์และ extract ข้อมูลจากข้อความดิบ
    พร้อม API Key Rotation เมื่อเจอ Quota Error
    """
    
    MAX_RETRIES = 4  # ลองใหม่สูงสุด 4 ครั้ง (หมุนครบ 4 keys)
    
    def __init__(self):
        self.current_key = None
        self.model = None
        self._configure_with_next_key()
    
    def _configure_with_next_key(self) -> bool:
        """สลับไปใช้ API Key ตัวถัดไป"""
        try:
            new_key = gemini_key_manager.get_key()
            if not new_key:
                print("❌ [AIMapperService] ไม่มี API keys เหลืออยู่!")
                return False
            
            self.current_key = new_key
            genai.configure(api_key=new_key)
            self.model = genai.GenerativeModel(settings.GEMINI_MODEL)
            
            # แสดง key ที่ใช้ (ซ่อนส่วนท้าย)
            masked_key = new_key[:8] + "..." + new_key[-4:]
            print(f"🔑 [AIMapperService] สลับไปใช้ key: {masked_key}")
            return True
            
        except Exception as e:
            print(f"❌ [AIMapperService] การตั้งค่าล้มเหลว: {e}")
            return False
    
    def _build_prompt(self, raw_data: Dict[str, Any], target_fields: List[str]) -> str:
        """สร้าง Prompt สำหรับ AI ในการ extract ข้อมูล"""
        # รวมข้อมูลดิบทั้งหมดเป็น text เดียว
        raw_text_parts = []
        for key, value in raw_data.items():
            if value and str(value).strip():
                raw_text_parts.append(f"{key}: {value}")
        raw_text = "\n".join(raw_text_parts)
        
        # สร้าง field descriptions - รองรับทั้ง Core และ Detail fields
        field_descriptions = {
            # Core Fields (ตรงกับ Schema)
            "title": "ชื่อหลักของสถานที่/ร้านค้า",
            "category": "หมวดหมู่หลัก เช่น ที่พัก, ร้านอาหาร, แหล่งท่องเที่ยว, วัด",
            "topic": "ประเภทเฉพาะ เช่น คาเฟ่, อาหารเหนือ, วัดประวัติศาสตร์",
            "summary": "สรุปข้อมูลสำคัญทั้งหมดใน 2-3 ประโยค",
            "keywords": "คำสำคัญสำหรับค้นหา คั่นด้วย comma",
            # Detail Fields
            "detail_overview": "ข้อมูลทั่วไปและประวัติความเป็นมา",
            "detail_location": "ที่อยู่ พิกัด GPS และวิธีเดินทาง",
            "detail_hours_contact": "เวลาทำการ เบอร์โทร Line Facebook",
            "detail_highlights": "จุดเด่น สิ่งที่น่าสนใจ สิ่งที่ห้ามพลาด",
            "detail_price": "ช่วงราคา ค่าเข้าชม ค่าใช้จ่าย",
            "detail_atmosphere": "บรรยากาศ ความรู้สึก สไตล์ของสถานที่",
            "detail_facilities": "สิ่งอำนวยความสะดวก ที่จอดรถ WiFi ห้องน้ำ",
            "detail_tips": "เคล็ดลับ คำแนะนำ ช่วงเวลาที่ดีที่สุด",
        }
        
        # สร้างรายการ fields ที่ต้องการ
        fields_list = []
        for field in target_fields:
            desc = field_descriptions.get(field, field)
            fields_list.append(f'  "{field}": "{desc}"')
        fields_json_hint = "{\n" + ",\n".join(fields_list) + "\n}"
        
        prompt = f"""[CONTEXT]
คุณคือ AI Data Extraction Expert ที่เชี่ยวชาญการแปลงข้อมูลดิบให้เป็น Structured JSON
ภารกิจของคุณคือวิเคราะห์ข้อความและ extract ข้อมูลออกมาให้ตรงกับ fields ที่กำหนด

[INPUT DATA - ข้อมูลดิบ]
---
{raw_text}
---

[TARGET FIELDS - ช่องที่ต้องการ extract]
{fields_json_hint}

[INSTRUCTIONS]
1. วิเคราะห์ข้อมูลดิบด้านบนอย่างละเอียด
2. Extract ข้อมูลที่เกี่ยวข้องลงในแต่ละ field ที่กำหนด
3. ข้อมูลรวมกันอยู่ให้แยกออกมา เช่น "เปิด 8-5 โมง" → detail_hours_contact: "08:00-17:00"
4. ถ้าหาข้อมูลไม่เจอสำหรับ field ใด ให้ใส่ค่า null
5. **ห้ามสมมติข้อมูลที่ไม่มีอยู่จริงในต้นฉบับ**

[OUTPUT FORMAT]
ตอบกลับเป็น JSON object เท่านั้น ห้ามมีข้อความอื่นนอกเหนือจาก JSON
"""
        return prompt
    
    async def transform_row(self, raw_data: Dict[str, Any], target_fields: List[str]) -> Dict[str, Any]:
        """
        แปลงข้อมูลดิบ 1 แถวให้เป็น structured data ตาม target fields
        พร้อม retry logic เมื่อเจอ Quota Error (429)
        """
        if not self.model:
            if not self._configure_with_next_key():
                return {field: "[Error: No API key available]" for field in target_fields}
        
        prompt = self._build_prompt(raw_data, target_fields)
        
        for attempt in range(self.MAX_RETRIES):
            try:
                # Call Gemini API
                response = await asyncio.to_thread(
                    self.model.generate_content,
                    prompt
                )
                
                # Clean and parse response
                response_text = response.text.strip()
                
                # Remove markdown code block if present
                if response_text.startswith("```json"):
                    response_text = response_text[7:]
                if response_text.startswith("```"):
                    response_text = response_text[3:]
                if response_text.endswith("```"):
                    response_text = response_text[:-3]
                response_text = response_text.strip()
                
                # Parse JSON
                extracted = json.loads(response_text)
                
                # Ensure all target fields are present
                result = {}
                for field in target_fields:
                    result[field] = extracted.get(field)
                
                return result
                
            except json.JSONDecodeError as je:
                print(f"❌ [AIMapperService] ข้อผิดพลาดในการแปลง JSON: {je}")
                return {field: "[Error: Invalid JSON response]" for field in target_fields}
                
            except Exception as e:
                error_str = str(e)
                
                # Check if it's a Quota Error (429)
                if "429" in error_str or "quota" in error_str.lower() or "exceeded" in error_str.lower():
                    print(f"⚠️ [AIMapperService] Quota เต็ม (ความพยายามครั้งที่ {attempt + 1}/{self.MAX_RETRIES}), กำลังหมุนเวียน key...")
                    
                    # Rotate to next API key
                    if self._configure_with_next_key():
                        # Exponential backoff: 1s, 2s, 4s, 8s
                        wait_time = min(2 ** attempt, 8)
                        print(f"⏳ รอ {wait_time} วินาทีก่อนลองใหม่...")
                        await asyncio.sleep(wait_time)
                        continue
                    else:
                        return {field: "[Error: All API keys exhausted]" for field in target_fields}
                else:
                    # Other errors - don't retry
                    print(f"❌ [AIMapperService] ข้อผิดพลาด: {e}")
                    return {field: f"[Error: {str(e)[:50]}]" for field in target_fields}
        
        # All retries exhausted
        return {field: "[Error: Max retries exceeded]" for field in target_fields}
    
    async def transform_batch(
        self, 
        rows: List[Dict[str, Any]], 
        target_fields: List[str],
        concurrency: int = 5  # เพิ่มเป็น 5 เพื่อความเร็ว
    ) -> List[Dict[str, Any]]:
        """
        แปลงข้อมูลหลายแถว (batch processing)
        ใช้ concurrency ต่ำเพื่อหลีกเลี่ยง rate limit
        """
        results = []
        
        # Process in batches to avoid rate limiting
        for i in range(0, len(rows), concurrency):
            batch = rows[i:i + concurrency]
            
            # Create tasks for concurrent execution
            tasks = [
                self.transform_row(row, target_fields)
                for row in batch
            ]
            
            # Execute batch
            batch_results = await asyncio.gather(*tasks)
            
            # Add original data reference
            for j, result in enumerate(batch_results):
                original_row = batch[j]
                # Combine original values for reference
                original_combined = " | ".join(
                    str(v) for v in original_row.values() if v and str(v).strip()
                )
                result["_original_combined"] = original_combined
                result["_original_row"] = original_row
                results.append(result)
            
            print(f"✅ [AIMapperService] ประมวลผล batch {i//concurrency + 1}, ทั้งหมด: {len(results)}/{len(rows)}")
            
            # Increased delay between batches to avoid rate limiting
            if i + concurrency < len(rows):
                await asyncio.sleep(1.5)
        
        return results


    async def extract_from_document(
        self, 
        document_text: str, 
        target_fields: List[str]
    ) -> Dict[str, Any]:
        """
        Extract ข้อมูลจากเอกสารยาว (เช่น PDF)
        รองรับ text หลายหน้า/หลายบรรทัด
        """
        if not self.model:
            if not self._configure_with_next_key():
                return {field: None for field in target_fields}
        
        # สร้าง field descriptions
        field_descriptions = {
            "title": "ชื่อหลักของสถานที่/ร้านค้า",
            "category": "หมวดหมู่หลัก เช่น ที่พัก, ร้านอาหาร, แหล่งท่องเที่ยว, วัด",
            "topic": "ประเภทเฉพาะ เช่น คาเฟ่, อาหารเหนือ, วัดประวัติศาสตร์",
            "summary": "สรุปข้อมูลสำคัญทั้งหมดใน 2-3 ประโยค",
            "keywords": "คำสำคัญสำหรับค้นหา คั่นด้วย comma",
            "detail_overview": "ข้อมูลทั่วไปและประวัติความเป็นมา",
            "detail_location": "ที่อยู่ พิกัด GPS และวิธีเดินทาง",
            "detail_hours_contact": "เวลาทำการ เบอร์โทร Line Facebook",
            "detail_highlights": "จุดเด่น สิ่งที่น่าสนใจ สิ่งที่ห้ามพลาด",
            "detail_price": "ช่วงราคา ค่าเข้าชม ค่าใช้จ่าย",
            "detail_atmosphere": "บรรยากาศ ความรู้สึก สไตล์ของสถานที่",
            "detail_facilities": "สิ่งอำนวยความสะดวก ที่จอดรถ WiFi ห้องน้ำ",
            "detail_tips": "เคล็ดลับ คำแนะนำ ช่วงเวลาที่ดีที่สุด",
        }
        
        fields_list = []
        for field in target_fields:
            desc = field_descriptions.get(field, field)
            fields_list.append(f'  "{field}": "{desc}"')
        fields_json_hint = "{\n" + ",\n".join(fields_list) + "\n}"
        
        # Prompt สำหรับ document extraction
        prompt = f"""[CONTEXT]
คุณคือ AI Document Extraction Expert ที่เชี่ยวชาญการแปลงเอกสารให้เป็น Structured JSON
ภารกิจของคุณคือวิเคราะห์เอกสาร PDF และ extract ข้อมูลออกมาให้ตรงกับ fields ที่กำหนด

[DOCUMENT CONTENT]
---
{document_text[:15000]}
---

[TARGET FIELDS - ช่องที่ต้องการ extract]
{fields_json_hint}

[INSTRUCTIONS]
1. วิเคราะห์เนื้อหาเอกสารอย่างละเอียด ครอบคลุมทุกหน้า
2. Extract ข้อมูลที่เกี่ยวข้องลงในแต่ละ field ที่กำหนด
3. สำหรับ summary ให้สรุปใจความสำคัญทั้งหมดใน 2-3 ประโยค
4. สำหรับ detail_* fields ให้ใส่รายละเอียดครบถ้วน
5. ถ้าหาข้อมูลไม่เจอสำหรับ field ใด ให้ใส่ค่า null
6. **ห้ามสมมติข้อมูลที่ไม่มีอยู่จริงในต้นฉบับ**

[OUTPUT FORMAT]
ตอบกลับเป็น JSON object เท่านั้น ห้ามมีข้อความอื่นนอกเหนือจาก JSON
"""
        
        for attempt in range(self.MAX_RETRIES):
            try:
                response = await asyncio.to_thread(
                    self.model.generate_content,
                    prompt
                )
                
                response_text = response.text.strip()
                print(f"🔍 [AIMapper] ผลลัพธ์ดิบจาก AI (500 ตัวอักษรแรก): {response_text[:500]}")
                
                # Clean markdown
                if response_text.startswith("```json"):
                    response_text = response_text[7:]
                if response_text.startswith("```"):
                    response_text = response_text[3:]
                if response_text.endswith("```"):
                    response_text = response_text[:-3]
                response_text = response_text.strip()
                
                print(f"🔍 [AIMapper] ผลลัพธ์ที่ทำความสะอาดแล้ว (500 ตัวอักษรแรก): {response_text[:500]}")
                
                extracted = json.loads(response_text)
                
                # Handle nested structure: {"locations": [...]} or list [...]
                if isinstance(extracted, dict) and "locations" in extracted:
                    locations = extracted["locations"]
                    extracted = locations[0] if locations else {}
                    print(f"⚠️ [AIMapper] ดึงข้อมูลจากอาร์เรย์ 'locations', {len(locations)} รายการ")
                elif isinstance(extracted, list):
                    print(f"⚠️ [AIMapper] AI ส่งกลับรายการที่มี {len(extracted)} รายการ, เลือกรายการแรก")
                    extracted = extracted[0] if extracted else {}
                
                result = {}
                for field in target_fields:
                    result[field] = extracted.get(field) if isinstance(extracted, dict) else None
                
                print(f"✅ [AIMapper] การดึงข้อมูลจากเอกสารเสร็จสมบูรณ์ ได้ข้อมูล {len([v for v in result.values() if v])} ฟิลด์")
                return result
                
            except json.JSONDecodeError as je:
                print(f"❌ [AIMapper] ข้อผิดพลาดในการแปลง JSON: {je}")
                print(f"❌ [AIMapper] แปลงไม่สำเร็จ: {response_text[:300] if 'response_text' in dir() else 'N/A'}")
                return {field: None for field in target_fields}
                
            except Exception as e:
                error_str = str(e)
                if "429" in error_str or "quota" in error_str.lower():
                    print(f"⚠️ [AIMapper] Quota เต็ม, กำลังหมุนเวียน key...")
                    if self._configure_with_next_key():
                        await asyncio.sleep(min(2 ** attempt, 8))
                        continue
                    else:
                        return {field: None for field in target_fields}
                else:
                    print(f"❌ [AIMapper] ข้อผิดพลาด: {e}")
                    return {field: None for field in target_fields}
        
        return {field: None for field in target_fields}

    async def extract_with_web_search(
        self, 
        search_query: str, 
        target_fields: List[str]
    ) -> Dict[str, Any]:
        """
        🌐 Extract ข้อมูลโดยใช้ Google Search grounding
        AI จะค้นหาข้อมูลจากเว็บและ extract ลง fields ที่กำหนด
        """
        if not self.model:
            if not self._configure_with_next_key():
                return {field: None for field in target_fields}
        
        # สร้าง field descriptions
        field_descriptions = {
            "title": "ชื่อหลักของสถานที่/ร้านค้า",
            "category": "หมวดหมู่หลัก เช่น ที่พัก, ร้านอาหาร, แหล่งท่องเที่ยว, วัด",
            "topic": "ประเภทเฉพาะ เช่น คาเฟ่, อาหารเหนือ, วัดประวัติศาสตร์",
            "summary": "สรุปข้อมูลสำคัญทั้งหมดใน 2-3 ประโยค",
            "keywords": "คำสำคัญสำหรับค้นหา คั่นด้วย comma",
            "detail_overview": "ข้อมูลทั่วไป ประวัติความเป็นมา ความสำคัญ",
            "detail_location": "ที่อยู่ พิกัด GPS และวิธีเดินทาง",
            "detail_hours_contact": "เวลาทำการ เบอร์โทร Line Facebook",
            "detail_highlights": "จุดเด่น สิ่งที่น่าสนใจ สิ่งที่ห้ามพลาด",
            "detail_price": "ช่วงราคา ค่าเข้าชม ค่าใช้จ่าย",
            "detail_atmosphere": "บรรยากาศ ความรู้สึก สไตล์ของสถานที่",
            "detail_facilities": "สิ่งอำนวยความสะดวก ที่จอดรถ WiFi ห้องน้ำ",
            "detail_tips": "เคล็ดลับ คำแนะนำ ช่วงเวลาที่ดีที่สุด",
        }
        
        fields_list = []
        for field in target_fields:
            desc = field_descriptions.get(field, field)
            fields_list.append(f'  "{field}": "{desc}"')
        fields_json_hint = "{\n" + ",\n".join(fields_list) + "\n}"
        
        # Prompt สำหรับ web search extraction
        prompt = f"""คุณเป็น AI ผู้เชี่ยวชาญด้านข้อมูลท่องเที่ยวจังหวัดน่าน ประเทศไทย

ค้นหาข้อมูลเกี่ยวกับ: "{search_query}"

โปรดค้นหาจาก Google และสรุปข้อมูลที่พบลงใน JSON format ดังนี้:
{fields_json_hint}

กฎ:
1. ใส่ข้อมูลเฉพาะที่พบจากการค้นหาจริงเท่านั้น
2. ถ้าไม่พบข้อมูลสำหรับ field ใด ให้ใส่ null
3. สำหรับ summary ให้สรุปใจความสำคัญใน 2-3 ประโยค
4. ตอบกลับเป็น JSON object เท่านั้น ห้ามมีข้อความอื่น"""

        for attempt in range(self.MAX_RETRIES):
            try:
                # ใช้ Google Search grounding
                from google.generativeai import types
                
                response = await asyncio.to_thread(
                    self.model.generate_content,
                    prompt,
                    tools=[types.Tool(google_search=types.GoogleSearch())]
                )
                
                response_text = response.text.strip()
                
                # Clean markdown
                if response_text.startswith("```json"):
                    response_text = response_text[7:]
                if response_text.startswith("```"):
                    response_text = response_text[3:]
                if response_text.endswith("```"):
                    response_text = response_text[:-3]
                response_text = response_text.strip()
                
                # หา JSON object ใน response
                import re
                json_match = re.search(r'\{[\s\S]*\}', response_text)
                if json_match:
                    response_text = json_match.group()
                
                extracted = json.loads(response_text)
                
                result = {}
                for field in target_fields:
                    result[field] = extracted.get(field)
                
                print(f"✅ [AIMapper] การดึงข้อมูลจากการค้นหาเว็บเสร็จสมบูรณ์สำหรับ: {search_query}")
                return result
                
            except json.JSONDecodeError as je:
                print(f"❌ [AIMapper] ข้อผิดพลาดในการแปลง JSON: {je}")
                print(f"Response was: {response_text[:500] if 'response_text' in dir() else 'N/A'}")
                return {field: None for field in target_fields}
                
            except Exception as e:
                error_str = str(e)
                if "429" in error_str or "quota" in error_str.lower():
                    print(f"⚠️ [AIMapper] Quota เต็ม, กำลังหมุนเวียน key...")
                    if self._configure_with_next_key():
                        await asyncio.sleep(min(2 ** attempt, 8))
                        continue
                    else:
                        return {field: None for field in target_fields}
                else:
                    print(f"❌ [AIMapper] ข้อผิดพลาดการค้นหาเว็บ: {e}")
                    return {field: None for field in target_fields}
        
        return {field: None for field in target_fields}

    async def detect_entries(self, document_text: str, target_count: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        🔍 Detect entries/topics in a large document
        AI จะสแกนเอกสารและระบุหัวข้อ/entries ที่พบ
        
        Args:
            document_text: ข้อความจากเอกสาร
            target_count: จำนวนหัวข้อที่ต้องการ (None = AI แนะนำ)
            
        Returns:
            List of {title, description} for each detected entry
        """
        if not self.model:
            if not self._configure_with_next_key():
                return []
        
        # Truncate text if too long
        max_chars = 15000
        if len(document_text) > max_chars:
            document_text = document_text[:max_chars] + "\n...[ตัดทอน]..."
        
        # Build count instruction
        if target_count and target_count > 0:
            count_instruction = f"4. สร้างให้ได้ {target_count} หัวข้อเท่านั้น (รวมหัวข้อที่คล้ายกัน หรือแยกย่อยตามความเหมาะสม)"
        else:
            count_instruction = "4. ระบุทุกหัวข้อที่พบ ไม่จำกัดจำนวน"
        
        prompt = f"""คุณเป็น AI ผู้เชี่ยวชาญในการวิเคราะห์เอกสารท่องเที่ยว

จากเอกสารต่อไปนี้ ให้ระบุรายการสถานที่/หัวข้อทั้งหมดที่พบ

[เอกสาร]
{document_text}

[คำสั่ง]
1. สแกนเอกสารหาสถานที่ท่องเที่ยว ร้านอาหาร ที่พัก หรือหัวข้อสำคัญ
2. สำหรับแต่ละรายการ ให้ระบุ title และ description สั้นๆ
3. ตอบเป็น JSON array เท่านั้น
{count_instruction}

[ตัวอย่าง output]
[
  {{"title": "วัดภูมินทร์", "description": "วัดที่มีภาพจิตรกรรมฝาผนังกระซิบรักบันลือโลก"}},
  {{"title": "ถนนคนเดินน่าน", "description": "ตลาดนัดยามเย็นกลางเมืองน่าน"}}
]

ตอบเป็น JSON array เท่านั้น:"""

        for attempt in range(self.MAX_RETRIES):
            try:
                response = await asyncio.to_thread(
                    self.model.generate_content,
                    prompt
                )
                
                response_text = response.text.strip()
                
                # Clean markdown
                if response_text.startswith("```json"):
                    response_text = response_text[7:]
                if response_text.startswith("```"):
                    response_text = response_text[3:]
                if response_text.endswith("```"):
                    response_text = response_text[:-3]
                response_text = response_text.strip()
                
                entries = json.loads(response_text)
                
                if not isinstance(entries, list):
                    entries = [entries]
                
                print(f"✅ [AIMapper] ตรวจพบ {len(entries)} รายการในเอกสาร")
                return entries
                
            except json.JSONDecodeError as je:
                print(f"❌ [AIMapper] ข้อผิดพลาดในการแปลง JSON: {je}")
                return []
                
            except Exception as e:
                error_str = str(e)
                if "429" in error_str or "quota" in error_str.lower():
                    print(f"⚠️ [AIMapper] Quota เต็ม, กำลังหมุนเวียน key...")
                    if self._configure_with_next_key():
                        await asyncio.sleep(min(2 ** attempt, 8))
                        continue
                    else:
                        return []
                else:
                    print(f"❌ [AIMapper] ข้อผิดพลาดในการตรวจจับรายการ: {e}")
                    return []
        
        return []

    async def extract_multiple_entries(
        self,
        document_text: str,
        entries: List[Dict[str, str]],
        target_fields: List[str]
    ) -> List[Dict[str, Any]]:
        """
        📊 Extract data for multiple entries from document
        AI จะ extract ข้อมูลของแต่ละ entry ตาม fields ที่กำหนด
        """
        if not self.model:
            if not self._configure_with_next_key():
                return []
        
        results = []
        
        for entry in entries:
            entry_title = entry.get("title", "")
            if not entry_title:
                continue
            
            # Create focused search text
            search_prompt = f"หัวข้อ: {entry_title}\nคำอธิบาย: {entry.get('description', '')}"
            combined = f"{search_prompt}\n\n{document_text}"
            
            # Use existing extraction method
            extracted = await self.extract_from_document(
                document_text=combined,
                target_fields=target_fields
            )
            
            # Ensure title is set
            if not extracted.get("title"):
                extracted["title"] = entry_title
            
            results.append(extracted)
        
        print(f"✅ [AIMapper] ดึงข้อมูลสำหรับ {len(results)} รายการ")
        return results


# Singleton instance
ai_mapper_service = AIMapperService()

