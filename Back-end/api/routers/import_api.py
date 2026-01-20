# /api/routers/import_api.py
"""
API Router สำหรับ AI-Powered Smart ETL System
รองรับการ Import ข้อมูลจาก Excel/CSV และ AI Transformation
"""

import asyncio
import logging
from typing import List, Dict, Any, Optional
from fastapi import APIRouter, File, UploadFile, HTTPException, Body, Depends, Form
from fastapi.responses import StreamingResponse
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


class PDFExtractResponse(BaseModel):
    """Response model สำหรับ PDF text extraction และ AI fill"""
    success: bool
    extracted_text: Optional[str] = Field(None, description="Text ที่ extract ได้จาก PDF")
    page_count: int = Field(0, description="จำนวนหน้าใน PDF")
    ai_data: Optional[Dict[str, Any]] = Field(None, description="ข้อมูลที่ AI extract ได้")
    message: str


class AIFillFormRequest(BaseModel):
    """Request model สำหรับ AI ช่วยเติมข้อมูล"""
    partial_data: Dict[str, Any] = Field(..., description="ข้อมูลบางส่วนที่กรอกมา")
    target_fields: List[str] = Field(
        default=["title", "category", "topic", "summary", "keywords"],
        description="Fields ที่ต้องการให้ AI ช่วยเติม"
    )
    use_web_search: bool = Field(
        default=False,
        description="ถ้า True จะใช้ Google Search หาข้อมูลเพิ่มเติม"
    )


class AIFillFormResponse(BaseModel):
    """Response model สำหรับ AI fill form"""
    success: bool
    filled_data: Dict[str, Any] = Field(..., description="ข้อมูลที่ AI ช่วยเติมให้")
    message: str


class DocumentScanResponse(BaseModel):
    """Response model สำหรับ document scan"""
    success: bool
    page_count: int = Field(0, description="จำนวนหน้าในเอกสาร")
    text_preview: str = Field("", description="ตัวอย่างข้อความจากเอกสาร")
    entries: List[Dict[str, str]] = Field(default=[], description="รายการ entries ที่ AI พบ")
    message: str
    ai_suggested_count: int = Field(0, description="จำนวนหัวข้อที่ AI แนะนำ")


class DocumentExtractRequest(BaseModel):
    """Request model สำหรับ document extract"""
    document_text: str = Field(..., description="ข้อความจากเอกสาร")
    entries: List[Dict[str, str]] = Field(..., description="รายการ entries ที่ต้องการ extract")
    target_fields: List[str] = Field(..., description="Fields ที่ต้องการให้ AI extract")


class DocumentExtractResponse(BaseModel):
    """Response model สำหรับ document extract"""
    success: bool
    data: List[Dict[str, Any]] = Field(default=[], description="ข้อมูลที่ extract ได้")
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
    # Validate file type using File Validator (Magic Number Check)
    from utils.file_validator import verify_file_signature
    import io
    
    if not await verify_file_signature(file):
        raise HTTPException(
            status_code=400, 
            detail=f"ไฟล์ไม่ถูกต้องหรือเสียหาย (Invalid signature for {file.filename})"
        )
    
    try:
        # Read file with size limit (Anti-DoS)
        MAX_FILE_SIZE = 200 * 1024 * 1024 # 200 MB (Requested by Admin)
        content = io.BytesIO()
        size = 0
        CHUNK_SIZE = 1024 * 1024 # 1MB
        
        while chunk := await file.read(CHUNK_SIZE):
            size += len(chunk)
            if size > MAX_FILE_SIZE:
                raise HTTPException(status_code=413, detail=f"ไฟล์มีขนาดใหญ่เกินไป (Max {MAX_FILE_SIZE/1024/1024} MB)")
            content.write(chunk)
            
        content.seek(0)
        file_content = content.read() # Load verified content for parsing
        
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
        logging.error(f"❌ [ImportAPI] เกิดข้อผิดพลาดในการดูตัวอย่างไฟล์: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"เกิดข้อผิดพลาดในการอ่านไฟล์: {str(e)}")


@router.post("/ai-transform", response_model=AITransformResponse, tags=["Admin :: Bulk Import"])
async def ai_transform_data(request: AITransformRequest):
    """
    🤖 AI Transform: แปลงข้อมูลดิบลง Target Fields ที่เลือก
    
    Update: 
    - ⚡ Smart Bypass: ถ้าข้อมูลดิบมี Fields ครบตาม Target อยู่แล้ว จะข้าม AI ไปเลย (Direct Import)
    - 🤖 AI Fallback: ถ้าข้อมูลไม่ครบ หรือ Field ไม่ตรง จะใช้ AI ช่วย Map ให้
    """
    if not request.raw_data:
        raise HTTPException(status_code=400, detail="ไม่มีข้อมูลให้ประมวลผล")
    
    if not request.target_fields:
        raise HTTPException(status_code=400, detail="กรุณาเลือก Target Fields อย่างน้อย 1 field")
    
    # Validate target fields
    valid_field_keys = [f["key"] for f in ALL_CONFIGURABLE_FIELDS]
    for field in request.target_fields:
        if field.startswith("custom_"): continue
        if field not in valid_field_keys:
            raise HTTPException(status_code=400, detail=f"Invalid field: {field}")
    
    try:
        # 1. ⚡ Smart Check: ตรวจสอบว่าต้องใช้ AI หรือไม่?
        # เช็คจากแถวแรกๆ ว่ามี Key ตรงกับ Target Fields หรือไม่
        sample_size = min(5, len(request.raw_data))
        sample_rows = request.raw_data[:sample_size]
        
        # นับจำนวน Field ที่ตรงกัน (Case-insensitive check)
        direct_map_count = 0
        target_set = set(f.lower() for f in request.target_fields)
        
        for row in sample_rows:
            row_keys = set(k.lower() for k in row.keys())
            # ถ้า row นี้มี keys ที่ครอบคลุม target_fields อย่างน้อย 80% (หรือครบ)
            common = row_keys.intersection(target_set)
            if len(common) >= len(target_set) * 0.8: # Threshold 80%
                direct_map_count += 1
        
        # ถ้า Data ส่วนใหญ่ (เกิน 80% ของ Sample) พร้อมใช้อยู่แล้ว -> Bypass AI
        is_structured_data = direct_map_count >= (sample_size * 0.8)
        
        transformed = []
        
        if is_structured_data:
            logging.info(f"⚡ [ImportAPI] Smart Bypass Active! ข้อมูลมีโครงสร้างครบถ้วน -> ข้ามขั้นตอน AI")
            
            # Direct Map Logic
            for row in request.raw_data:
                # Normalize keys to lowercase for matching
                row_lower = {k.lower(): v for k, v in row.items()}
                
                new_row = {}
                for field in request.target_fields:
                    # พยายามหา value จาก key ที่ตรงกัน (case-insensitive)
                    # เช่น target="title", data มี "Title" -> match
                    field_lower = field.lower()
                    if field_lower in row_lower:
                        new_row[field] = row_lower[field_lower]
                    else:
                        new_row[field] = None # หรือ "" ตามต้องการ
                
                # Keep original data for reference (optional) or merge missing columns
                # for k, v in row.items():
                #     if k not in new_row: new_row[k] = v
                    
                transformed.append(new_row)
                
        else:
            # 2. 🤖 AI Process Logic (เดิม)
            from core.services.ai_mapper_service import ai_mapper_service
            
            logging.info(f"🤖 [ImportAPI] Data is unstructured -> Using Gemini AI for {len(request.raw_data)} rows")
            
            transformed = await ai_mapper_service.transform_batch(
                rows=request.raw_data,
                target_fields=request.target_fields,
                concurrency=8 
            )
        
        logging.info(f"✅ [ImportAPI] Transform เสร็จสิ้น: {len(transformed)} แถว (Mode: {'Direct' if is_structured_data else 'AI'})")
        
        return AITransformResponse(
            original_rows=request.raw_data,
            transformed_rows=transformed,
            target_fields=request.target_fields,
            total_processed=len(transformed)
        )
        
    except Exception as e:
        logging.error(f"❌ [ImportAPI] Transform Error: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Transform Failed: {str(e)}")


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
            
            # Check for duplicate slug and secure uniqueness
            original_slug = slug
            import uuid
            
            # Check if slug exists in DB (loop to ensure uniqueness)
            retry_count = 0
            while retry_count < 5:
                existing = await asyncio.to_thread(db.get_location_by_slug, slug)
                if not existing:
                    break
                
                # If exists, append random suffix
                suffix = uuid.uuid4().hex[:6]
                slug = f"{original_slug}-{suffix}"
                retry_count += 1
            
            if retry_count >= 5:
                # Fallback if still busy
                slug = f"{original_slug}-{uuid.uuid4().hex[:12]}"
            
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
                logging.warning(f"⚠️ การสร้าง Vector ล้มเหลวสำหรับ {mongo_id}: {ve}")
            
            saved_count += 1
            logging.info(f"✅ Saved: {location_doc['title']} (slug: {slug})")
            
        except Exception as e:
            failed_count += 1
            error_msg = f"Row {idx + 1}: {str(e)}"
            errors.append(error_msg)
            logging.error(f"❌ บันทึกแถวที่ {idx + 1} ล้มเหลว: {e}")
    
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


@router.post("/pdf-extract", response_model=PDFExtractResponse, tags=["Admin :: Bulk Import"])
async def extract_from_pdf(file: UploadFile = File(...)):
    """
    📄 Extract ข้อความจาก PDF และให้ AI วิเคราะห์ข้อมูลสำหรับ Manual Entry

    1. อ่าน PDF ทุกหน้า
    2. ให้ AI วิเคราะห์และ extract ข้อมูลลง fields ที่กำหนด
    3. ส่งกลับข้อมูลสำหรับกรอกลงฟอร์ม
    """
    if not file.filename:
        raise HTTPException(status_code=400, detail="ไม่พบชื่อไฟล์")
    
    if not file.filename.lower().endswith('.pdf'):
        raise HTTPException(status_code=400, detail="รองรับเฉพาะไฟล์ PDF เท่านั้น")
    
    try:
        # Read PDF content
        file_content = await file.read()
        
        if not file_content:
            raise HTTPException(status_code=400, detail="ไฟล์ว่างเปล่า")
        
        # Extract text from PDF
        from core.services.pdf_reader_service import pdf_reader_service
        extracted_text = pdf_reader_service.extract_text(file_content)
        page_count = pdf_reader_service.get_page_count(file_content)
        
        if not extracted_text.strip():
            return PDFExtractResponse(
                success=False,
                extracted_text="",
                page_count=page_count,
                ai_data=None,
                message="ไม่พบข้อความใน PDF (อาจเป็นไฟล์ภาพหรือ scanned document)"
            )
        
        # Use AI to extract data from PDF text
        from core.services.ai_mapper_service import ai_mapper_service
        
        target_fields = ["title", "category", "topic", "summary", "keywords", 
                        "detail_overview", "detail_location", "detail_hours_contact",
                        "detail_highlights", "detail_price"]
        
        ai_data = await ai_mapper_service.extract_from_document(
            document_text=extracted_text,
            target_fields=target_fields
        )
        
        logging.info(f"✅ [ImportAPI] ดึงข้อมูลจาก PDF: {page_count} หน้า, AI เติมข้อมูล {len([v for v in ai_data.values() if v])} ฟิลด์")
        logging.info(f"📄 [ImportAPI] ข้อมูลจาก AI: {ai_data}")
        
        return PDFExtractResponse(
            success=True,
            extracted_text=extracted_text[:2000],  # Limit text preview
            page_count=page_count,
            ai_data=ai_data,
            message=f"อ่าน PDF สำเร็จ {page_count} หน้า"
        )
        
    except ValueError as ve:
        raise HTTPException(status_code=400, detail=str(ve))
    except Exception as e:
        logging.error(f"❌ [ImportAPI] ข้อผิดพลาดในการดึงข้อมูลจาก PDF: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"เกิดข้อผิดพลาดในการอ่าน PDF: {str(e)}")


@router.post("/ai-fill-form", response_model=AIFillFormResponse, tags=["Admin :: Bulk Import"])
async def ai_fill_form(request: AIFillFormRequest):
    """
    🤖 AI ช่วยเติมข้อมูลที่ขาดหายไปในฟอร์ม
    
    รับข้อมูลบางส่วน (เช่น title, details) และให้ AI ช่วยเติม fields อื่นๆ
    ถ้า use_web_search=True จะค้นหาข้อมูลจาก Google ด้วย
    """
    if not request.partial_data:
        raise HTTPException(status_code=400, detail="ไม่มีข้อมูลให้ประมวลผล")
    
    try:
        from core.services.ai_mapper_service import ai_mapper_service
        
        # รวมข้อมูลที่มีเป็น text
        input_text = "\n".join([
            f"{k}: {v}" for k, v in request.partial_data.items() 
            if v and str(v).strip()
        ])
        
        if not input_text.strip():
            return AIFillFormResponse(
                success=False,
                filled_data={},
                message="กรุณากรอกข้อมูลบางส่วนก่อน เช่น ชื่อสถานที่ หรือรายละเอียด"
            )
        
        # เลือกวิธีการ extract
        if request.use_web_search:
            # 🌐 ใช้ Google Custom Search หาข้อมูล
            from core.services.web_search_service import web_search_service
            
            search_query = request.partial_data.get("title", "") or request.partial_data.get("details", "")
            search_query = f"{search_query} จังหวัดน่าน ข้อมูลท่องเที่ยว"  # เพิ่ม context
            
            logging.info(f"🌐 [ImportAPI] กำลังค้นหาข้อมูลจากเว็บสำหรับ: {search_query}")
            
            # ค้นหาจาก Google
            web_results = await web_search_service.search_and_summarize(search_query)
            
            if web_results:
                # รวมข้อมูลจากเว็บกับข้อมูลที่มี แล้วให้ AI วิเคราะห์
                combined_text = f"{input_text}\n\n{web_results}"
                target_fields = request.target_fields + ["detail_overview", "detail_location", 
                                "detail_hours_contact", "detail_highlights", "detail_price"]
            else:
                combined_text = input_text
                target_fields = request.target_fields
                logging.warning("⚠️ [ImportAPI] การค้นหาจากเว็บไม่พบผลลัพธ์ ใช้ข้อมูลท้องถิ่นเท่านั้น")
            
            ai_data = await ai_mapper_service.extract_from_document(
                document_text=combined_text,
                target_fields=target_fields
            )
        else:
            # 📝 ใช้ข้อมูลที่กรอกมาเท่านั้น
            ai_data = await ai_mapper_service.extract_from_document(
                document_text=input_text,
                target_fields=request.target_fields
            )
        
        # รวมกับข้อมูลเดิม (ไม่ทับข้อมูลที่กรอกมาแล้ว)
        filled_data = {**request.partial_data}
        for field, value in ai_data.items():
            if value and (field not in filled_data or not filled_data[field]):
                filled_data[field] = value
        
        filled_count = len([v for v in ai_data.values() if v])
        method = "🌐 Web Search" if request.use_web_search else "📝 Local"
        logging.info(f"✅ [ImportAPI] {method} - AI เติมข้อมูล {filled_count} ฟิลด์")
        
        return AIFillFormResponse(
            success=True,
            filled_data=filled_data,
            message=f"AI ช่วยเติมข้อมูลสำเร็จ {filled_count} fields" + (" (ค้นจากเว็บ)" if request.use_web_search else "")
        )
        
    except Exception as e:
        logging.error(f"❌ [ImportAPI] ข้อผิดพลาดในการเติมข้อมูลด้วย AI แบบฟอร์ม: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"AI ช่วยเติมข้อมูลล้มเหลว: {str(e)}")


# =============================================================================
# Document Import Endpoints - สำหรับอัปโหลดเอกสาร PDF/DOC และให้ AI สร้างหลาย entries
# =============================================================================

@router.post("/document-scan", response_model=DocumentScanResponse)
async def document_scan(
    file: UploadFile = File(..., description="ไฟล์ PDF หรือ DOC/DOCX"),
    target_count: int = Form(default=0, description="จำนวนหัวข้อที่ต้องการ (0 = AI แนะนำ)")
):
    """
    📄 Scan เอกสาร PDF/DOC และให้ AI ระบุ entries ที่พบ
    
    1. อ่านข้อความจากเอกสาร
    2. AI สแกนหาหัวข้อ/สถานที่
    3. ส่งกลับ entries ที่พบ (ผู้ใช้แก้ไขได้)
    """
    # Validate file type
    from core.services.doc_reader_service import doc_reader_service
    
    if not file.filename:
        raise HTTPException(status_code=400, detail="กรุณาระบุชื่อไฟล์")
    
    if not doc_reader_service.is_supported(file.filename):
        raise HTTPException(
            status_code=400, 
            detail=f"รองรับเฉพาะไฟล์ PDF, DOC, DOCX เท่านั้น"
        )
    
    try:
        # Read file
        file_bytes = await file.read()
        
        if len(file_bytes) == 0:
            raise HTTPException(status_code=400, detail="ไฟล์ว่างเปล่า")
        
        # Extract text
        extracted_text, page_count = doc_reader_service.extract_text(file_bytes, file.filename)
        
        if not extracted_text.strip():
            return DocumentScanResponse(
                success=False,
                page_count=page_count,
                text_preview="",
                entries=[],
                message="ไม่พบข้อความในเอกสาร (อาจเป็นไฟล์ภาพหรือ scanned document)"
            )
        
        # AI detect entries with target count
        from core.services.ai_mapper_service import ai_mapper_service
        
        entries = await ai_mapper_service.detect_entries(extracted_text, target_count=target_count if target_count > 0 else None)
        
        ai_suggested_count = len(entries)
        logging.info(f"✅ [ImportAPI] สแกนเอกสาร: {page_count} หน้า, พบ {len(entries)} รายการ (เป้าหมาย: {target_count})")
        
        return DocumentScanResponse(
            success=True,
            page_count=page_count,
            text_preview=extracted_text[:2000],
            entries=entries,
            message=f"พบ {len(entries)} รายการในเอกสาร {page_count} หน้า",
            ai_suggested_count=ai_suggested_count
        )
        
    except ValueError as ve:
        raise HTTPException(status_code=400, detail=str(ve))
    except Exception as e:
        logging.error(f"❌ [ImportAPI] ข้อผิดพลาดในการสแกนเอกสาร: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"เกิดข้อผิดพลาดในการสแกนเอกสาร: {str(e)}")


@router.post("/document-extract", response_model=DocumentExtractResponse)
async def document_extract(request: DocumentExtractRequest):
    """
    📊 Extract ข้อมูลจากเอกสารตาม entries และ fields ที่เลือก
    
    1. รับ entries ที่ผู้ใช้กำหนด (จาก document-scan หรือแก้ไขเอง)
    2. AI extract ข้อมูลแต่ละ entry ตาม fields ที่เลือก
    3. ส่งกลับ table data พร้อมบันทึก
    """
    if not request.entries:
        raise HTTPException(status_code=400, detail="กรุณาระบุ entries ที่ต้องการ extract")
    
    if not request.target_fields:
        raise HTTPException(status_code=400, detail="กรุณาเลือก fields ที่ต้องการ extract")
    
    try:
        from core.services.ai_mapper_service import ai_mapper_service
        
        results = await ai_mapper_service.extract_multiple_entries(
            document_text=request.document_text,
            entries=request.entries,
            target_fields=request.target_fields
        )
        
        logging.info(f"✅ [ImportAPI] ดึงข้อมูลจากเอกสาร: ได้รับ {len(results)} รายการ")
        
        return DocumentExtractResponse(
            success=True,
            data=results,
            message=f"Extract ข้อมูลสำเร็จ {len(results)} รายการ"
        )
        
    except Exception as e:
        logging.error(f"❌ [ImportAPI] ข้อผิดพลาดในการดึงข้อมูลจากเอกสาร: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"เกิดข้อผิดพลาดในการ extract: {str(e)}")


@router.post("/document-stream")
async def document_stream(
    file: UploadFile = File(..., description="ไฟล์ PDF หรือ DOC/DOCX"),
    target_count: int = Form(default=5, description="จำนวนหัวข้อที่ต้องการ"),
    target_fields: str = Form(default="title,category,topic,summary,keywords", description="Fields ที่ต้องการ (comma-separated)")
):
    """
    📊 SSE Streaming: อัปโหลด → สแกน → Extract ทีละ entry → Stream ผลลัพธ์
    
    ส่ง SSE events:
    - {"type": "scan", "data": {"page_count": 5, "total_entries": 5}}
    - {"type": "entry", "data": {"index": 0, "title": "...", ...}}
    - {"type": "done", "data": {"total": 5}}
    - {"type": "error", "data": {"message": "..."}}
    """
    import json
    from core.services.doc_reader_service import doc_reader_service
    from core.services.ai_mapper_service import ai_mapper_service
    
    # Read file BEFORE streaming (to avoid closed file error)
    file_bytes = await file.read()
    filename = file.filename
    
    async def generate():
        try:
            # 1. Validate file
            if len(file_bytes) == 0:
                yield f"data: {json.dumps({'type': 'error', 'data': {'message': 'ไฟล์ว่างเปล่า'}})}\n\n"
                return
            
            # 2. Extract text
            extracted_text, page_count = doc_reader_service.extract_text(file_bytes, filename)
            if not extracted_text.strip():
                yield f"data: {json.dumps({'type': 'error', 'data': {'message': 'ไม่พบข้อความในเอกสาร'}})}\n\n"
                return
            
            # 3. Detect entries
            entries = await ai_mapper_service.detect_entries(extracted_text, target_count=target_count)
            
            # Send scan result
            yield f"data: {json.dumps({'type': 'scan', 'data': {'page_count': page_count, 'total_entries': len(entries)}})}\n\n"
            
            # 4. Extract each entry and stream
            fields_list = [f.strip() for f in target_fields.split(',') if f.strip()]
            
            for idx, entry in enumerate(entries):
                try:
                    # Extract this entry
                    entry_title = entry.get("title", "")
                    search_prompt = f"หัวข้อ: {entry_title}\nคำอธิบาย: {entry.get('description', '')}"
                    combined = f"{search_prompt}\n\n{extracted_text}"
                    
                    extracted = await ai_mapper_service.extract_from_document(
                        document_text=combined,
                        target_fields=fields_list
                    )
                    
                    # Ensure title is set
                    extracted['title'] = extracted.get('title') or entry_title
                    extracted['_index'] = idx
                    
                    # Send entry result
                    yield f"data: {json.dumps({'type': 'entry', 'data': extracted}, ensure_ascii=False)}\n\n"
                    
                except Exception as entry_error:
                    logging.warning(f"Entry {idx} failed: {entry_error}")
                    yield f"data: {json.dumps({'type': 'entry', 'data': {'title': entry.get('title', f'Entry {idx+1}'), '_error': str(entry_error), '_index': idx}})}\n\n"
            
            # 5. Done
            yield f"data: {json.dumps({'type': 'done', 'data': {'total': len(entries)}})}\n\n"
            logging.info(f"✅ [ImportAPI] Document stream completed: {len(entries)} entries")
            
        except Exception as e:
            logging.error(f"❌ [ImportAPI] Document stream error: {e}", exc_info=True)
            yield f"data: {json.dumps({'type': 'error', 'data': {'message': str(e)}})}\n\n"
    
    return StreamingResponse(
        generate(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no"
        }
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


@router.post("/import-jsonl-stream", tags=["Admin :: Bulk Import"])
async def import_jsonl_stream(
    file: UploadFile = File(..., description="ไฟล์ JSONL (.jsonl) ข้อมูลมหาศาล"),
    db: MongoDBManager = Depends(get_mongo_manager),
    vector_db: QdrantManager = Depends(get_qdrant_manager)
):
    """
    🌊 Streaming Import for Massive JSONL Data (Supports 200MB+)
    
    - อ่านไฟล์ทีละบรรทัด (Memory Efficient)
    - บันทึกลง DB ทันที (Real-time Insert)
    - Bypass AI ถ้าข้อมูลครบถ้วน (Fastest)
    - Syncs to Qdrant (Vector DB) for AI
    """
    import json
    import re
    
    if not file.filename.endswith('.jsonl') and not file.filename.endswith('.json'):
         raise HTTPException(status_code=400, detail="รองรับเฉพาะไฟล์ .jsonl หรือ .json เท่านั้น")

    async def processed_stream():
        count = 0
        success = 0
        failed = 0
        
        # Buffer for parsing lines
        buffer = b""
        
        try:
            # Yield initialization
            yield json.dumps({"type": "start", "message": "Starting stream import..."}) + "\n"
            
            while True:
                chunk = await file.read(1024 * 1024) # Read 1MB chunks
                if not chunk:
                    break
                
                buffer += chunk
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    if not line.strip(): continue
                    
                    count += 1
                    try:
                        # 1. Parse JSON Line
                        record = json.loads(line.decode('utf-8'))
                        
                        # 2. Add minimal defaults if needed
                        if "title" not in record:
                            record["title"] = f"Imported Item {count}"
                        
                        # 3. Save to MongoDB (Directly)
                        # Generate slug
                        slug = record.get("slug", "")
                        if not slug:
                            slug = re.sub(r'[^a-zA-Z0-9\u0E00-\u0E7F\s-]', '', record["title"].lower())
                            slug = re.sub(r'[\s]+', '-', slug.strip())
                        
                        # Ensure uniqueness (simple append)
                        doc = {
                            "slug": slug,
                            **record,
                            "imported_via": "stream_jsonl"
                        }
                        
                        # Save to MongoDB
                        db.add_location(doc)
                        
                        # 4. Sync to Qdrant (Vector DB) -> AI BRAIN 🧠
                        try:
                           # Construct text representation for embedding
                           desc_parts = [
                               doc.get('title', ''),
                               doc.get('category', ''),
                               doc.get('topic', ''),
                               doc.get('summary', '')
                           ]
                           if 'details' in doc and isinstance(doc['details'], list):
                               for d in doc['details']:
                                   desc_parts.append(f"{d.get('heading','')}: {d.get('content','')}")
                           
                           text_for_ai = " ".join([str(p) for p in desc_parts if p])
                           
                           # Upsert to Qdrant
                           vector_db.upsert_location(doc_id=str(doc.get('_id', slug)), description=text_for_ai, payload=doc)
                           
                        except Exception as v_err:
                           # Log vector error but don't fail the import
                           print(f"⚠️ Vector DB Sync Error for {slug}: {v_err}")

                        success += 1
                        
                        # Yield progress every 100 items
                        if success % 100 == 0:
                             yield json.dumps({"type": "progress", "count": count, "success": success}) + "\n"

                    except Exception as e:
                        failed += 1
                        yield json.dumps({"type": "error", "row": count, "message": str(e)}) + "\n"
        
            # Final stats
            yield json.dumps({
                "type": "complete", 
                "total": count, 
                "success": success, 
                "failed": failed,
                "message": f"Import Completed! Success: {success}, Failed: {failed}"
            }) + "\n"
            
        except Exception as e:
             yield json.dumps({"type": "fatal_error", "message": str(e)}) + "\n"

    return StreamingResponse(processed_stream(), media_type="application/x-ndjson")
