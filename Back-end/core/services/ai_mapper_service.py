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
                print("❌ [AIMapperService] No API keys available!")
                return False
            
            self.current_key = new_key
            genai.configure(api_key=new_key)
            self.model = genai.GenerativeModel(settings.GEMINI_MODEL)
            
            # แสดง key ที่ใช้ (ซ่อนส่วนท้าย)
            masked_key = new_key[:8] + "..." + new_key[-4:]
            print(f"🔑 [AIMapperService] Switched to key: {masked_key}")
            return True
            
        except Exception as e:
            print(f"❌ [AIMapperService] Failed to configure: {e}")
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
                print(f"❌ [AIMapperService] JSON parse error: {je}")
                return {field: "[Error: Invalid JSON response]" for field in target_fields}
                
            except Exception as e:
                error_str = str(e)
                
                # Check if it's a Quota Error (429)
                if "429" in error_str or "quota" in error_str.lower() or "exceeded" in error_str.lower():
                    print(f"⚠️ [AIMapperService] Quota exceeded (attempt {attempt + 1}/{self.MAX_RETRIES}), rotating key...")
                    
                    # Rotate to next API key
                    if self._configure_with_next_key():
                        # Exponential backoff: 1s, 2s, 4s, 8s
                        wait_time = min(2 ** attempt, 8)
                        print(f"⏳ Waiting {wait_time}s before retry...")
                        await asyncio.sleep(wait_time)
                        continue
                    else:
                        return {field: "[Error: All API keys exhausted]" for field in target_fields}
                else:
                    # Other errors - don't retry
                    print(f"❌ [AIMapperService] Error: {e}")
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
            
            print(f"✅ [AIMapperService] Processed batch {i//concurrency + 1}, total: {len(results)}/{len(rows)}")
            
            # Increased delay between batches to avoid rate limiting
            if i + concurrency < len(rows):
                await asyncio.sleep(1.5)
        
        return results


# Singleton instance
ai_mapper_service = AIMapperService()

