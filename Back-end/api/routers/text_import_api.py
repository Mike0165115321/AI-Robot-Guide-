"""
Text Import API - AI-Powered Text Extraction
สำหรับ extract ข้อมูลสถานที่ท่องเที่ยวจาก raw text โดยใช้ Gemini AI
"""

import asyncio
import json
import logging
import re
from typing import List, Dict, Any, Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

import google.generativeai as genai
from core.config import settings
from core.ai_models.key_manager import gemini_key_manager

router = APIRouter(prefix="/api/ai", tags=["AI :: Text Import"])

# Constants
MAX_RETRIES = 3
EXTRACTION_MODEL = "gemini-2.0-flash"


# =============================================================================
# Pydantic Models
# =============================================================================

class TextExtractRequest(BaseModel):
    """Request body for text extraction"""
    raw_text: str = Field(..., min_length=10, description="Raw text to extract locations from")


class ExtractedLocation(BaseModel):
    """Single extracted location"""
    title: str
    category: str
    description: str
    keywords: List[str] = []
    location_guess: Optional[str] = None


class TextExtractResponse(BaseModel):
    """Response containing extracted locations"""
    success: bool
    locations: List[ExtractedLocation] = []
    message: str = ""
    raw_count: int = 0


# =============================================================================
# System Prompt for Gemini
# =============================================================================

EXTRACTION_SYSTEM_PROMPT = """คุณคือ "Data Extraction Specialist" สำหรับการท่องเที่ยวจังหวัดน่าน ประเทศไทย

📌 หน้าที่ของคุณ:
อ่านข้อความที่ผู้ใช้ส่งมา แล้ว extract ข้อมูลสถานที่ท่องเที่ยว ร้านอาหาร หรือที่พักทั้งหมดที่กล่าวถึง

📋 กฎการทำงาน:
1. ตอบเป็น JSON Array เท่านั้น ห้ามมี markdown formatting (เช่น ```json)
2. ถ้าไม่พบสถานที่ใดๆ ให้ตอบ []
3. ห้ามคิดเองหรือเพิ่มข้อมูลที่ไม่มีในต้นฉบับ
4. category ต้องเป็นหนึ่งใน: "แหล่งท่องเที่ยว", "ร้านอาหาร", "ที่พัก", "วัด", "คาเฟ่", "กิจกรรม", "อื่นๆ"

📐 JSON Schema ที่ต้องใช้:
[
  {
    "title": "ชื่อสถานที่",
    "category": "หมวดหมู่",
    "description": "สรุปสั้นๆ 2-3 ประโยค",
    "keywords": ["คำสำคัญ1", "คำสำคัญ2"],
    "location_guess": "อำเภอหรือพื้นที่ (ถ้าระบุได้)"
  }
]

ตอบเฉพาะ JSON Array เท่านั้น ไม่ต้องมีคำอธิบายเพิ่มเติม"""


# =============================================================================
# Helper Functions
# =============================================================================

def _clean_json_response(response_text: str) -> str:
    """Remove markdown code blocks and clean JSON response"""
    # Remove ```json ... ``` blocks
    cleaned = re.sub(r'^```json\s*', '', response_text.strip())
    cleaned = re.sub(r'\s*```$', '', cleaned)
    # Also handle ``` without json
    cleaned = re.sub(r'^```\s*', '', cleaned)
    cleaned = re.sub(r'\s*```$', '', cleaned)
    return cleaned.strip()


def _parse_extraction_response(response_text: str) -> List[Dict[str, Any]]:
    """Parse and validate the extraction response"""
    try:
        cleaned = _clean_json_response(response_text)
        data = json.loads(cleaned)
        
        if not isinstance(data, list):
            logging.warning(f"Expected list, got {type(data)}")
            return []
        
        # Validate each item
        valid_items = []
        for item in data:
            if isinstance(item, dict) and "title" in item:
                valid_items.append({
                    "title": str(item.get("title", "")),
                    "category": str(item.get("category", "อื่นๆ")),
                    "description": str(item.get("description", "")),
                    "keywords": item.get("keywords", []) if isinstance(item.get("keywords"), list) else [],
                    "location_guess": item.get("location_guess") or None
                })
        
        return valid_items
        
    except json.JSONDecodeError as e:
        logging.error(f"❌ JSON parse error: {e}\nResponse: {response_text[:500]}")
        return []


async def _call_gemini_extraction(raw_text: str) -> List[Dict[str, Any]]:
    """Call Gemini API with key rotation for text extraction"""
    last_error = None
    
    for attempt in range(MAX_RETRIES):
        try:
            # Get rotated API key
            api_key = gemini_key_manager.get_key()
            if not api_key:
                logging.error("❌ No Gemini API keys available")
                return []
            
            genai.configure(api_key=api_key)
            model = genai.GenerativeModel(EXTRACTION_MODEL)
            
            # Build the full prompt
            full_prompt = f"{EXTRACTION_SYSTEM_PROMPT}\n\n--- ข้อความที่ต้องวิเคราะห์ ---\n{raw_text}"
            
            # Generate response
            response = await asyncio.to_thread(
                model.generate_content,
                full_prompt,
                generation_config=genai.GenerationConfig(
                    temperature=0.1,  # Low temperature for consistent extraction
                    max_output_tokens=4096
                )
            )
            
            if response and response.text:
                logging.info(f"✅ Gemini extraction successful (attempt {attempt + 1})")
                return _parse_extraction_response(response.text)
            else:
                logging.warning("⚠️ Empty response from Gemini")
                return []
                
        except Exception as e:
            last_error = e
            error_str = str(e).lower()
            is_rate_limit = "429" in str(e) or "quota" in error_str or "rate" in error_str
            
            if is_rate_limit:
                logging.warning(f"⚠️ Rate limit hit, rotating key... (attempt {attempt + 1}/{MAX_RETRIES})")
                await asyncio.sleep(1)  # Brief pause before retry
                continue
            else:
                logging.error(f"❌ Gemini extraction error: {e}")
                break
    
    logging.error(f"❌ All extraction retries failed: {last_error}")
    return []


# =============================================================================
# API Endpoints
# =============================================================================

@router.post("/extract-text", response_model=TextExtractResponse)
async def extract_text_locations(request: TextExtractRequest):
    """
    🪄 Extract tourist locations from raw text using AI
    
    ใช้ Gemini AI วิเคราะห์ข้อความและ extract สถานที่ท่องเที่ยว/ร้านอาหาร/ที่พักออกมา
    
    - **Input**: Raw text (e.g., copied from PDF, article, website)
    - **Output**: List of extracted locations with title, category, description, keywords
    """
    if not request.raw_text or len(request.raw_text.strip()) < 10:
        raise HTTPException(status_code=400, detail="ข้อความสั้นเกินไป กรุณาวางข้อความที่มีข้อมูลมากกว่านี้")
    
    try:
        logging.info(f"🔍 [TextImport] Starting extraction from {len(request.raw_text)} characters")
        
        # Call Gemini for extraction
        extracted = await _call_gemini_extraction(request.raw_text)
        
        if not extracted:
            return TextExtractResponse(
                success=True,
                locations=[],
                message="ไม่พบสถานที่ท่องเที่ยวในข้อความนี้",
                raw_count=0
            )
        
        logging.info(f"✅ [TextImport] Extracted {len(extracted)} locations")
        
        return TextExtractResponse(
            success=True,
            locations=[ExtractedLocation(**loc) for loc in extracted],
            message=f"พบ {len(extracted)} สถานที่",
            raw_count=len(extracted)
        )
        
    except Exception as e:
        logging.error(f"❌ [TextImport] Extraction error: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=f"AI extraction ล้มเหลว: {str(e)}"
        )
