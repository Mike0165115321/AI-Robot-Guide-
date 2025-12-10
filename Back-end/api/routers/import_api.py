# /api/routers/import_api.py
"""
API Router สำหรับ AI-Powered Smart ETL System
รองรับการ Import ข้อมูลจาก Excel/CSV และ AI Transformation
"""

import asyncio
import logging
from typing import List, Dict, Any, Optional
from fastapi import APIRouter, File, UploadFile, HTTPException, Body, Depends
from pydantic import BaseModel, Field

from core.services.import_service import import_service
from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from ..dependencies import get_mongo_manager, get_qdrant_manager

router = APIRouter(tags=["Admin :: Bulk Import"])


# =============================================================================
# Pydantic Models
# =============================================================================

class RawPreviewResponse(BaseModel):
    """Response model สำหรับ raw file preview"""
    columns: List[str] = Field(..., description="รายชื่อ columns ที่พบในไฟล์")
    preview_rows: List[Dict[str, Any]] = Field(..., description="ตัวอย่างข้อมูล (max 10 rows)")
    total_rows: int = Field(..., description="จำนวน rows ทั้งหมด")
    showing_rows: int = Field(..., description="จำนวน rows ที่แสดง")
    filename: str = Field(..., description="ชื่อไฟล์ต้นฉบับ")


class AITransformRequest(BaseModel):
    """Request model สำหรับ AI transformation"""
    raw_data: List[Dict[str, Any]] = Field(..., description="ข้อมูลดิบจาก preview")
    target_fields: List[str] = Field(
        ..., 
        description="Fields ที่ต้องการให้ AI map ข้อมูลลงไป",
        min_length=1,
        max_length=15
    )


class AITransformResponse(BaseModel):
    """Response model สำหรับ AI transformation result"""
    original_rows: List[Dict[str, Any]] = Field(..., description="ข้อมูลดิบ (สำหรับเปรียบเทียบ)")
    transformed_rows: List[Dict[str, Any]] = Field(..., description="ข้อมูลที่ AI แปลงแล้ว")
    target_fields: List[str] = Field(..., description="Fields ที่เลือก")
    total_processed: int = Field(..., description="จำนวน rows ที่ประมวลผล")


class ConfirmSaveRequest(BaseModel):
    """Request model สำหรับ confirm และ save ข้อมูล"""
    transformed_rows: List[Dict[str, Any]] = Field(..., description="ข้อมูลที่ AI แปลงแล้ว (พร้อม save)")


class ConfirmSaveResponse(BaseModel):
    """Response model สำหรับ save result"""
    success: bool
    saved_count: int = Field(..., description="จำนวน records ที่ save สำเร็จ")
    failed_count: int = Field(0, description="จำนวน records ที่ save ไม่สำเร็จ")
    message: str

# =============================================================================
# Target Fields Configuration - ตรงกับ MongoDB Schema จริง
# =============================================================================

# Core Fields - ฟิลด์หลักที่ต้องมีใน Database
CORE_FIELDS = [
    {
        "key": "title", 
        "label": "Title (ชื่อ)", 
        "description": "ชื่อหลักของสถานที่/ร้านค้า",
        "type": "text",
        "required": True,
        "group": "core"
    },
    {
        "key": "category", 
        "label": "Category (หมวดหมู่)", 
        "description": "หมวดหมู่หลัก เช่น ที่พัก, ร้านอาหาร, แหล่งท่องเที่ยว, วัด",
        "type": "text",
        "required": True,
        "group": "core"
    },
    {
        "key": "topic", 
        "label": "Topic (ประเภทย่อย)", 
        "description": "ประเภทเฉพาะ เช่น คาเฟ่, อาหารเหนือ, วัดประวัติศาสตร์",
        "type": "text",
        "required": True,
        "group": "core"
    },
    {
        "key": "summary", 
        "label": "Summary (สรุป)", 
        "description": "สรุปข้อมูลสำคัญทั้งหมดในย่อหน้าเดียว",
        "type": "textarea",
        "required": False,
        "group": "core"
    },
    {
        "key": "keywords", 
        "label": "Keywords (คำค้นหา)", 
        "description": "คำสำคัญสำหรับค้นหา คั่นด้วย comma เช่น วัด,น่าน,จิตรกรรม",
        "type": "tags",
        "required": False,
        "group": "core"
    },
]

# Detail Fields - ฟิลด์รายละเอียด (แต่ละอันจะสร้างเป็น details[] item)
DETAIL_FIELDS = [
    {
        "key": "detail_overview",
        "label": "ภาพรวม",
        "description": "ข้อมูลทั่วไปและประวัติความเป็นมา",
        "type": "detail",
        "heading": "ภาพรวม",
        "group": "details"
    },
    {
        "key": "detail_location",
        "label": "ที่ตั้งและการเดินทาง",
        "description": "ที่อยู่ พิกัด GPS และวิธีเดินทาง",
        "type": "detail",
        "heading": "ที่ตั้งและการเดินทาง",
        "group": "details"
    },
    {
        "key": "detail_hours_contact",
        "label": "เวลาเปิด-ปิด และติดต่อ",
        "description": "เวลาทำการ เบอร์โทร Line Facebook",
        "type": "detail",
        "heading": "เวลาเปิด-ปิด และติดต่อ",
        "group": "details"
    },
    {
        "key": "detail_highlights",
        "label": "จุดเด่นและสิ่งห้ามพลาด",
        "description": "สิ่งที่น่าสนใจ สิ่งที่ต้องทำ/ดู",
        "type": "detail",
        "heading": "จุดเด่นและสิ่งห้ามพลาด",
        "group": "details"
    },
    {
        "key": "detail_price",
        "label": "ราคาและค่าใช้จ่าย",
        "description": "ช่วงราคา ค่าเข้าชม ค่าอาหาร",
        "type": "detail",
        "heading": "ราคาและค่าใช้จ่าย",
        "group": "details"
    },
    {
        "key": "detail_atmosphere",
        "label": "บรรยากาศและสไตล์",
        "description": "บรรยากาศ ความรู้สึก สไตล์ของสถานที่",
        "type": "detail",
        "heading": "บรรยากาศและสไตล์",
        "group": "details"
    },
    {
        "key": "detail_facilities",
        "label": "สิ่งอำนวยความสะดวก",
        "description": "ที่จอดรถ WiFi ห้องน้ำ สิ่งอำนวยความสะดวก",
        "type": "detail",
        "heading": "สิ่งอำนวยความสะดวก",
        "group": "details"
    },
    {
        "key": "detail_tips",
        "label": "เคล็ดลับและข้อแนะนำ",
        "description": "คำแนะนำสำหรับผู้มาเยือน ช่วงเวลาที่ดีที่สุด",
        "type": "detail",
        "heading": "เคล็ดลับและข้อแนะนำ",
        "group": "details"
    },
]

# รวม Fields ทั้งหมด
ALL_CONFIGURABLE_FIELDS = CORE_FIELDS + DETAIL_FIELDS


# =============================================================================
# API Endpoints
# =============================================================================

@router.get("/target-fields", tags=["Admin :: Bulk Import"])
async def get_target_fields():
    """
    ดึงรายการ Target Fields แบ่งตามกลุ่ม
    - core: ฟิลด์หลัก (title, category, topic, summary, keywords)
    - details: ฟิลด์รายละเอียด (แต่ละอันสร้างเป็น details[] item)
    """
    return {
        "core_fields": CORE_FIELDS,
        "detail_fields": DETAIL_FIELDS,
        "all_fields": ALL_CONFIGURABLE_FIELDS,
        "total": len(ALL_CONFIGURABLE_FIELDS)
    }


@router.post("/preview-raw", response_model=RawPreviewResponse, tags=["Admin :: Bulk Import"])
async def preview_raw_file(file: UploadFile = File(...)):
    """
    📤 Upload และ Preview ไฟล์ Excel/CSV
    
    อ่านไฟล์และแสดง preview ข้อมูลดิบก่อน AI transform
    """
    # Validate file type
    if not file.filename:
        raise HTTPException(status_code=400, detail="ไม่พบชื่อไฟล์")
    
    filename_lower = file.filename.lower()
    valid_extensions = ['.xlsx', '.xls', '.csv']
    
    if not any(filename_lower.endswith(ext) for ext in valid_extensions):
        raise HTTPException(
            status_code=400, 
            detail=f"ไม่รองรับไฟล์ประเภทนี้: {file.filename}. รองรับเฉพาะ Excel (.xlsx, .xls) และ CSV (.csv)"
        )
    
    try:
        # Read file content
        file_content = await file.read()
        
        if not file_content:
            raise HTTPException(status_code=400, detail="ไฟล์ว่างเปล่า")
        
        # Parse file in thread pool (blocking operation)
        raw_data = await asyncio.to_thread(
            import_service.parse_file,
            file_content,
            file.filename
        )
        
        if not raw_data:
            raise HTTPException(status_code=400, detail="ไม่พบข้อมูลในไฟล์")
        
        # Generate preview
        preview = import_service.get_preview(raw_data, max_rows=10)
        
        return RawPreviewResponse(
            columns=preview["columns"],
            preview_rows=preview["preview_rows"],
            total_rows=preview["total_rows"],
            showing_rows=preview["showing_rows"],
            filename=file.filename
        )
        
    except ValueError as ve:
        raise HTTPException(status_code=400, detail=str(ve))
    except Exception as e:
        logging.error(f"❌ [ImportAPI] Error previewing file: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"เกิดข้อผิดพลาดในการอ่านไฟล์: {str(e)}")


@router.post("/ai-transform", response_model=AITransformResponse, tags=["Admin :: Bulk Import"])
async def ai_transform_data(request: AITransformRequest):
    """
    🤖 AI Transform: แปลงข้อมูลดิบลง Target Fields ที่เลือก
    
    ใช้ Gemini AI วิเคราะห์และ extract ข้อมูลจากแต่ละ row
    """
    if not request.raw_data:
        raise HTTPException(status_code=400, detail="ไม่มีข้อมูลให้ประมวลผล")
    
    if not request.target_fields:
        raise HTTPException(status_code=400, detail="กรุณาเลือก Target Fields อย่างน้อย 1 field")
    
    # Validate target fields - รองรับทั้ง core และ detail fields
    valid_field_keys = [f["key"] for f in ALL_CONFIGURABLE_FIELDS]
    for field in request.target_fields:
        # Custom fields (เริ่มด้วย custom_) ไม่ต้อง validate
        if field.startswith("custom_"):
            continue
        if field not in valid_field_keys:
            raise HTTPException(
                status_code=400, 
                detail=f"Invalid field: {field}. Valid fields: {valid_field_keys}"
            )
    
    try:
        # Import AI Mapper Service
        from core.services.ai_mapper_service import ai_mapper_service
        
        # Process with AI
        logging.info(f"🤖 [ImportAPI] Starting AI transform for {len(request.raw_data)} rows, fields: {request.target_fields}")
        
        transformed = await ai_mapper_service.transform_batch(
            rows=request.raw_data,
            target_fields=request.target_fields,
            concurrency=3  # Process 3 rows at a time
        )
        
        logging.info(f"✅ [ImportAPI] AI transform completed: {len(transformed)} rows processed")
        
        return AITransformResponse(
            original_rows=request.raw_data,
            transformed_rows=transformed,
            target_fields=request.target_fields,
            total_processed=len(transformed)
        )
        
    except Exception as e:
        logging.error(f"❌ [ImportAPI] AI transform error: {e}", exc_info=True)
        raise HTTPException(
            status_code=500, 
            detail=f"AI Transform ล้มเหลว: {str(e)}"
        )


@router.post("/confirm-save", response_model=ConfirmSaveResponse, tags=["Admin :: Bulk Import"])
async def confirm_save_data(
    request: ConfirmSaveRequest,
    db: MongoDBManager = Depends(get_mongo_manager),
    vector_db: QdrantManager = Depends(get_qdrant_manager)
):
    """
    💾 Confirm และ Save ข้อมูลที่ AI แปลงแล้วลง Database
    
    บันทึกลง MongoDB และสร้าง Vector ใน Qdrant
    """
    if not request.transformed_rows:
        raise HTTPException(status_code=400, detail="ไม่มีข้อมูลให้บันทึก")
    
    saved_count = 0
    failed_count = 0
    errors = []
    
    for idx, row in enumerate(request.transformed_rows):
        try:
            # Remove internal fields
            clean_row = {k: v for k, v in row.items() if not k.startswith("_")}
            
            # Generate slug from title (new schema) or name (old)
            title = clean_row.get("title") or clean_row.get("name", "")
            if not title:
                title = f"imported-item-{idx + 1}"
            
            # Create slug (kebab-case)
            import re
            slug = re.sub(r'[^a-zA-Z0-9\u0E00-\u0E7F\s-]', '', title.lower())
            slug = re.sub(r'[\s]+', '-', slug.strip())
            slug = slug[:50]  # Limit length
            if not slug:
                slug = f"item-{idx + 1}"
            
            # Check for duplicate slug, add suffix if needed
            existing = await asyncio.to_thread(db.get_location_by_slug, slug)
            if existing:
                import time
                slug = f"{slug}-{int(time.time()) % 10000}"
            
            # Build location document matching new schema
            location_doc = {
                "slug": slug,
                "title": title,
                "category": clean_row.get("category", "อื่นๆ"),
                "topic": clean_row.get("topic") or clean_row.get("sub_topic", "ทั่วไป"),
                "summary": clean_row.get("summary") or _build_summary(clean_row),
                "keywords": _extract_keywords(clean_row),
                "details": _build_details(clean_row),
                "metadata": {
                    "image_prefix": slug,
                    "imported_via": "bulk_import",
                    "source_fields": list(clean_row.keys())
                }
            }
            
            # Save to MongoDB
            mongo_id = await asyncio.to_thread(
                db.add_location,
                location_doc
            )
            
            if not mongo_id:
                raise Exception("MongoDB insert returned None")
            
            # Create vector in Qdrant
            try:
                desc_for_vector = f"หัวข้อ: {location_doc['title']}\nประเภท: {location_doc['topic']}\nสรุป: {location_doc['summary']}"
                await vector_db.upsert_location(mongo_id=mongo_id, description=desc_for_vector)
            except Exception as ve:
                logging.warning(f"⚠️ Vector creation failed for {mongo_id}: {ve}")
            
            saved_count += 1
            logging.info(f"✅ Saved: {location_doc['title']} (slug: {slug})")
            
        except Exception as e:
            failed_count += 1
            error_msg = f"Row {idx + 1}: {str(e)}"
            errors.append(error_msg)
            logging.error(f"❌ Failed to save row {idx + 1}: {e}")
    
    # Build result message
    if failed_count == 0:
        message = f"บันทึกสำเร็จทั้งหมด {saved_count} รายการ!"
    else:
        message = f"บันทึก {saved_count} รายการ, ล้มเหลว {failed_count} รายการ"
        if errors:
            message += f". Errors: {'; '.join(errors[:3])}"
    
    return ConfirmSaveResponse(
        success=failed_count == 0,
        saved_count=saved_count,
        failed_count=failed_count,
        message=message
    )


def _build_summary(row: dict) -> str:
    """สร้าง summary จากข้อมูลที่ extract ได้"""
    parts = []
    
    if row.get("highlights"):
        parts.append(f"จุดเด่น: {row['highlights']}")
    if row.get("atmosphere"):
        parts.append(f"บรรยากาศ: {row['atmosphere']}")
    if row.get("location_text"):
        parts.append(f"ที่ตั้ง: {row['location_text']}")
    if row.get("opening_hours"):
        parts.append(f"เปิด: {row['opening_hours']}")
    if row.get("price_range"):
        parts.append(f"ราคา: {row['price_range']}")
    if row.get("contact"):
        parts.append(f"ติดต่อ: {row['contact']}")
    
    return " | ".join(parts) if parts else "ข้อมูลที่นำเข้าจาก Bulk Import"


def _extract_keywords(row: dict) -> list:
    """ดึง keywords จากข้อมูล - รองรับทั้ง schema เก่าและใหม่"""
    keywords = []
    
    # Title/Name
    title = row.get("title") or row.get("name")
    if title:
        keywords.append(title)
    
    # Category
    if row.get("category"):
        keywords.append(row["category"])
    
    # Topic/SubTopic    
    topic = row.get("topic") or row.get("sub_topic")
    if topic:
        keywords.append(topic)
    
    # Keywords field (comma separated string or already a list)
    if row.get("keywords"):
        if isinstance(row["keywords"], list):
            keywords.extend(row["keywords"][:5])
        else:
            kw_list = str(row["keywords"]).split(",")
            keywords.extend([k.strip() for k in kw_list[:5]])
    
    # Highlights
    highlights = row.get("detail_highlights") or row.get("highlights")
    if highlights:
        hl_list = str(highlights).split(",")
        keywords.extend([h.strip() for h in hl_list[:3]])
    
    return list(set([k for k in keywords if k]))[:10]  # Max 10 unique keywords


def _build_details(row: dict) -> list:
    """สร้าง details array จากข้อมูล - รองรับทั้ง detail_ prefix และ field ตรง"""
    details = []
    
    # Detail field mappings: (key_prefix, heading)
    detail_mappings = [
        ("detail_overview", "ภาพรวม"),
        ("detail_location", "ที่ตั้งและการเดินทาง"),
        ("detail_hours_contact", "เวลาทำการและติดต่อ"),
        ("detail_highlights", "จุดเด่น"),
        ("detail_price", "ราคา"),
        ("detail_atmosphere", "บรรยากาศ"),
        ("detail_facilities", "สิ่งอำนวยความสะดวก"),
        ("detail_tips", "เคล็ดลับ"),
    ]
    
    # Process detail_ prefixed fields
    for key, heading in detail_mappings:
        if row.get(key):
            details.append({
                "heading": heading,
                "content": str(row[key])
            })
    
    # Fallback to old field names if no detail_ fields found
    if not details:
        info_parts = []
        if row.get("location_text"):
            info_parts.append(f"ที่ตั้ง: {row['location_text']}")
        if row.get("coordinates"):
            info_parts.append(f"พิกัด: {row['coordinates']}")
        if row.get("opening_hours"):
            info_parts.append(f"เวลาเปิด: {row['opening_hours']}")
        if row.get("contact"):
            info_parts.append(f"ติดต่อ: {row['contact']}")
        if row.get("price_range"):
            info_parts.append(f"ราคา: {row['price_range']}")
        
        if info_parts:
            details.append({"heading": "ข้อมูลทั่วไป", "content": "\n".join(info_parts)})
        
        if row.get("highlights"):
            details.append({"heading": "จุดเด่น", "content": row["highlights"]})
        if row.get("atmosphere"):
            details.append({"heading": "บรรยากาศ", "content": row["atmosphere"]})
        if row.get("facilities"):
            details.append({"heading": "สิ่งอำนวยความสะดวก", "content": row["facilities"]})
    
    # Handle custom fields (custom_ prefix)
    for key, value in row.items():
        if key.startswith("custom_") and value:
            heading = key.replace("custom_", "").replace("_", " ").title()
            details.append({"heading": heading, "content": str(value)})
    
    return details if details else [{"heading": "ภาพรวม", "content": "ข้อมูลที่นำเข้าจาก Bulk Import"}]
