import asyncio
import shutil
import logging
from pathlib import Path
from typing import List, Dict, Any
from bson import ObjectId
from fastapi import (APIRouter, Body, Depends, File, HTTPException, UploadFile,
                     status, Query)
from pydantic import ValidationError

from ..schemas import (LocationAdminSummaryWithImage, LocationBase, LocationInDB,
                       LocationAdminSummary)
from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from core.document_processor import DocumentProcessor
from ..dependencies import get_mongo_manager, get_qdrant_manager, get_analytics_service
from core.services.analytics_service import AnalyticsService
from core.services.image_sync_service import ImageSyncService

router = APIRouter(tags=["Admin"])

STATIC_IMAGE_DIR = Path(__file__).resolve().parent.parent.parent / "static" / "images"
STATIC_IMAGE_DIR.mkdir(parents=True, exist_ok=True)


def _find_first_image_for_prefix(prefix: str) -> str | None:
    if not prefix:
        return None
    try:
        sorted_files = sorted(STATIC_IMAGE_DIR.glob(f"{prefix}*"))
        for f in sorted_files:
            if f.is_file() and f.suffix.lower() in ('.jpg', '.jpeg', '.png', '.webp'):
                return f"/static/images/{f.name}"
        return None
    except Exception as e:
        logging.error(f"เกิดข้อผิดพลาดในการหารูปภาพแรกสำหรับ prefix '{prefix}': {e}", exc_info=False)
        return None

@router.post("/locations/upload-image/", tags=["Admin :: Image Upload"])
async def upload_location_image(
    image_prefix: str = Query(..., description="Prefix ที่ตรงกับ 'slug' ของสถานที่"),
    file: UploadFile = File(...)
):
    if not image_prefix.strip():
        raise HTTPException(status_code=400, detail="Image Prefix (slug) is required.")
    file_extension = Path(file.filename).suffix.lower()
    if file_extension not in [".jpg", ".jpeg", ".png", ".webp"]:
        raise HTTPException(status_code=400, detail="Invalid image type. Only JPG, PNG, WEBP.")
    try:
        file_content = await file.read()
        if not file_content:
            raise HTTPException(status_code=400, detail="File is empty.")
        def save_file_in_thread(prefix: str, content: bytes, extension: str) -> str | None:
            try:
                import uuid
                unique_id = uuid.uuid4().hex[:8]
                new_filename = f"{prefix}-{unique_id}{extension}"
                file_path = STATIC_IMAGE_DIR / new_filename
                with file_path.open("wb") as buffer:
                    buffer.write(content)
                logging.info(f"🖼️  อัปโหลดและบันทึกรูปภาพเรียบร้อยแล้ว: {new_filename}")
                return new_filename
            except Exception as e:
                logging.error(f"❌ เกิดข้อผิดพลาดขณะบันทึกไฟล์ (sync thread) สำหรับ prefix '{prefix}': {e}", exc_info=True)
                return None
        saved_filename = await asyncio.to_thread(
            save_file_in_thread,
            image_prefix,
            file_content,
            file_extension
        )
        if not saved_filename:
            raise HTTPException(status_code=500, detail="Could not save image to disk.")
        return {"image_prefix": image_prefix, "saved_as": saved_filename}
    except HTTPException as http_exc:
        raise http_exc
    except Exception as e:
        logging.error(f"❌ เกิดข้อผิดพลาดในการอัปโหลดรูปภาพสำหรับ prefix '{image_prefix}': {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Could not upload image: {e}")


@router.post("/sync-images", tags=["Admin :: Image Sync"])
async def sync_images(
    db: MongoDBManager = Depends(get_mongo_manager)
):
    """
    🔄 สแกนไฟล์รูปภาพจาก /static/images/ และซิงค์ข้อมูลลง MongoDB
    
    ใช้เมื่อ:
    - เพิ่มรูปภาพใหม่ลงโฟลเดอร์
    - ต้องการ refresh cache
    - ตรวจสอบว่ารูปภาพทั้งหมดถูกบันทึกในฐานข้อมูล
    
    Returns:
        สรุปผลการ sync (จำนวนรูป prefix ที่พบ และบันทึก)
    """
    try:
        sync_service = ImageSyncService(db)
        result = await asyncio.to_thread(sync_service.sync_images)
        logging.info(f"✅ [API] Image Sync สำเร็จ: {result}")
        return result
    except Exception as e:
        logging.error(f"❌ เกิดข้อผิดพลาดในการซิงค์รูปภาพ: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Image sync failed: {e}")


@router.get("/analytics/dashboard", tags=["Admin :: Analytics"])
async def get_analytics_dashboard(
    days: int = Query(30, description="จำนวนวันย้อนหลังที่ต้องการดูข้อมูล"),
    analytics: AnalyticsService = Depends(get_analytics_service)
):
    """
    ดึงข้อมูลสรุปสำหรับ Dashboard (กราฟและตัวเลขรวม)
    """
    try:
        stats = await analytics.get_dashboard_summary(days)
        return stats
    except Exception as e:
        logging.error(f"❌ เกิดข้อผิดพลาดในการดึงข้อมูล Analytics Dashboard: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Failed to fetch analytics data.")


@router.get("/schema/fields", tags=["Admin :: Schema"])
async def get_available_fields(
    db: MongoDBManager = Depends(get_mongo_manager),
    sample_size: int = Query(50, description="Number of documents to sample for field detection")
):
    """
    ดึงรายชื่อ fields ทั้งหมดที่มีในฐานข้อมูล - ใช้สำหรับ Field Visibility Settings
    
    Returns:
        List of field names with metadata (type hints, sample values)
    """
    def get_fields_sync() -> Dict[str, Any]:
        try:
            collection = db.get_collection("nan_locations")
            if collection is None:
                return {"fields": [], "error": "Collection not found"}
            
            # Sample documents to detect all unique field names
            all_fields = set()
            field_samples = {}
            
            docs = list(collection.find({}).limit(sample_size))
            
            def extract_fields(obj, prefix=""):
                """Recursively extract field names from nested objects"""
                if isinstance(obj, dict):
                    for key, value in obj.items():
                        full_key = f"{prefix}.{key}" if prefix else key
                        # Skip internal MongoDB fields
                        if key.startswith("_") and key != "_id":
                            continue
                        all_fields.add(full_key)
                        # Store sample value for type hint
                        if full_key not in field_samples and value is not None:
                            if isinstance(value, (str, int, float, bool)):
                                field_samples[full_key] = str(value)[:50]
                            elif isinstance(value, list):
                                field_samples[full_key] = f"[Array: {len(value)} items]"
                            elif isinstance(value, dict):
                                field_samples[full_key] = "{Object}"
                                extract_fields(value, full_key)
            
            for doc in docs:
                extract_fields(doc)
            
            # Build field info with metadata
            field_info = []
            for field in sorted(all_fields):
                # Determine field type hint
                sample = field_samples.get(field, "")
                is_required = field in ["_id", "slug", "title"]
                is_recommended = field in ["category", "topic", "summary", "keywords"]
                
                field_info.append({
                    "name": field,
                    "sample": sample,
                    "required": is_required,
                    "recommended": is_recommended,
                    "nested": "." in field
                })
            
            return {
                "fields": field_info,
                "total_documents": collection.count_documents({}),
                "sampled_documents": len(docs)
            }
        except Exception as e:
            logging.error(f"❌ เกิดข้อผิดพลาดในการดึง Schema Fields: {e}", exc_info=True)
            return {"fields": [], "error": str(e)}
    
    try:
        result = await asyncio.to_thread(get_fields_sync)
        return result
    except Exception as e:
        logging.error(f"❌ เกิดข้อผิดพลาดใน get_available_fields: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Failed to fetch schema fields.")

@router.post(
    "/locations/",
    response_model=LocationInDB,
    status_code=status.HTTP_201_CREATED,
    tags=["Admin :: Locations CRUD"]
)
async def create_location(
    location_data: LocationBase,
    db: MongoDBManager = Depends(get_mongo_manager),
    vector_db: QdrantManager = Depends(get_qdrant_manager)
):
    logging.info(f"กำลังพยายามสร้างสถานที่ใหม่ด้วย Slug: {location_data.slug}")
    try:
        existing = await asyncio.to_thread(db.get_location_by_slug, location_data.slug)
        if existing:
            logging.warning(f"การสร้างล้มเหลว: Slug '{location_data.slug}' มีอยู่แล้ว")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Slug '{location_data.slug}' นี้มีอยู่แล้ว กรุณาใช้ Slug อื่น"
            )
    except HTTPException as http_exc:
        raise http_exc
    except Exception as e:
        logging.error(f"เกิดข้อผิดพลาดในการตรวจสอบ Slug '{location_data.slug}' ที่มีอยู่: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Error checking for existing slug.")
    mongo_id_str = ""
    try:
        mongo_id_str = await asyncio.to_thread(
            db.add_location,
            location_data.model_dump()
        )
        if not mongo_id_str:
             raise Exception("Failed to create document in MongoDB (add_location returned None or empty string).")
        logging.info(f"สร้างข้อมูลใน MongoDB สำเร็จ: slug='{location_data.slug}', mongo_id='{mongo_id_str}'")
    except Exception as e:
        logging.error(f"❌ เกิดข้อผิดพลาดในการสร้างสถานที่ '{location_data.slug}' ใน MongoDB: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to create location in database: {e}"
        )
    # 🔄 [SYNC] MongoDB -> Qdrant (Create)
    # ส่วนนี้คือการนำข้อมูลที่เพิ่งสร้างใน MongoDB ไปสร้าง Vector ลง Qdrant ทันที
    # เพื่อให้สามารถค้นหาแบบ Semantic Search ได้ทันทีโดยไม่ต้องรอ Sync รอบใหญ่
    try:
        desc_title = location_data.title
        desc_topic = location_data.topic
        desc_summary = location_data.summary
        # เตรียมข้อความที่จะนำไปทำ Embedding (Vector)
        description_for_vector = f"หัวข้อ: {desc_title}\nประเภท: {desc_topic}\nสรุป: {desc_summary}"
        
        # สั่งให้ QdrantManager ทำการสร้าง Vector และบันทึกลงฐานข้อมูล Qdrant
        
        # 🆕 เตรียม Metadata สำหรับ Payload (Fix for Filter Bug)
        qdrant_metadata = {
            "title": location_data.title,
            "slug": location_data.slug,
            "category": location_data.category,  # สำคัญ!
            "district": (location_data.related_info or {}).get("district"),
            "sub_district": (location_data.related_info or {}).get("sub_district")
        }
        
        await vector_db.upsert_location(
            mongo_id=mongo_id_str, 
            description=description_for_vector,
            metadata=qdrant_metadata # 👈 ส่ง Metadata ไปด้วย
        )
        logging.info(f"สร้าง Vector สำหรับ mongo_id '{mongo_id_str}' สำเร็จ")
    except Exception as vector_e:
        logging.error(f"⚠️ คำเตือน: สร้างข้อมูลใน MongoDB สำหรับ slug '{location_data.slug}' สำเร็จ แต่ล้มเหลวในการสร้าง Vector สำหรับ {mongo_id_str} ข้อผิดพลาด: {vector_e}", exc_info=True)
    try:
        new_location_doc = await asyncio.to_thread(db.get_location_by_id, mongo_id_str)
        if not new_location_doc:
            raise Exception("Could not retrieve document immediately after creation.")
        return LocationInDB(**new_location_doc)
    except Exception as e:
         logging.error(f"❌ วิกฤต: สร้างใน MongoDB แล้ว (ID: {mongo_id_str}) แต่ไม่สามารถดึงข้อมูลมาตอบกลับได้ ข้อผิดพลาด: {e}", exc_info=True)
         raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Location created but failed to retrieve. Check DB manually for ID {mongo_id_str}."
        )

@router.post("/locations/analyze-document", tags=["Admin :: Document Analysis"])
async def analyze_document_endpoint(file: UploadFile = File(...)):
    try:
        file_content = await file.read()
        if not file_content:
            raise HTTPException(status_code=400, detail="No file content received.")
        processor = DocumentProcessor()
        extracted_data = await asyncio.to_thread(
            processor.analyze_document,
            file_content=file_content,
            content_type=file.content_type
        )
        if not extracted_data:
            raise HTTPException(status_code=500, detail="Failed to process document or extract data.")
        return extracted_data
    except HTTPException as http_exc:
        raise http_exc
    except Exception as e:
        logging.error(f"❌ เกิดข้อผิดพลาดระหว่างการวิเคราะห์เอกสาร: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"An unexpected error occurred during analysis: {str(e)}")


@router.get("/locations/", response_model=Dict[str, Any], tags=["Admin :: Locations CRUD"])
async def get_all_locations_summary(
    skip: int = Query(0, ge=0),
    limit: int = Query(10, ge=1),
    db: MongoDBManager = Depends(get_mongo_manager)
):
    def get_paginated_summaries_sync() -> Dict[str, Any]:
        try:
            locations_from_db, total_count = db.get_locations_paginated(skip=skip, limit=limit)
            enriched_models = []
            for loc_dict in locations_from_db:
                if not isinstance(loc_dict, dict) or '_id' not in loc_dict:
                    logging.warning(f"ข้ามข้อมูลสถานที่ที่ไม่ถูกต้อง: {loc_dict}")
                    continue
                try:
                    prefix = (loc_dict.get("metadata") or {}).get("image_prefix")
                    preview_url = _find_first_image_for_prefix(prefix)
                    
                    summary_model = LocationAdminSummaryWithImage(
                        **loc_dict,
                        preview_image_url=preview_url
                    )
                    enriched_models.append(summary_model)
                except ValidationError as e:
                    logging.warning(f"ข้ามสถานที่เนื่องจากข้อผิดพลาดในการตรวจสอบความถูกต้อง: {loc_dict.get('slug', 'N/A')} รายละเอียด: {e}")
                    continue

            return {
                "items": enriched_models,
                "total_count": total_count,
                "page": (skip // limit) + 1,
                "limit": limit
            }
        except Exception as e:
            logging.error(f"❌ เกิดข้อผิดพลาดในการรวมข้อมูลสรุปใน sync thread: {e}", exc_info=True)
            return {"items": [], "total_count": 0, "page": 1, "limit": limit}

    try:
        result = await asyncio.to_thread(get_paginated_summaries_sync)
        return result
    except Exception as e:
        logging.error(f"❌ เกิดข้อผิดพลาดที่ไม่คาดคิดในการดึงข้อมูลสรุปสถานที่ทั้งหมด: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Internal server error retrieving location summaries.")


@router.get("/locations/{slug}", response_model=LocationInDB, tags=["Admin :: Locations CRUD"])
async def get_location_by_slug(
    slug: str,
    db: MongoDBManager = Depends(get_mongo_manager)
):
    logging.info(f"กำลังพยายามดึงข้อมูลสถานที่ด้วย Slug: {slug}")
    try:
        location_data = await asyncio.to_thread(db.get_location_by_slug, slug)

        if not location_data:
            logging.warning(f"ไม่พบสถานที่สำหรับ Slug: {slug}")
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND,
                                detail=f"Location with slug '{slug}' not found.")

        prefix = (location_data.get("metadata") or {}).get("image_prefix")
        preview_url = _find_first_image_for_prefix(prefix)
        
        location_model = LocationInDB(
            **location_data,
            preview_image_url=preview_url
        )
        
        logging.debug(f"ข้อมูลดิบจาก DB สำหรับ Slug '{slug}': {location_data}")
        return location_model

    except HTTPException as http_exc:
        raise http_exc
    except ValidationError as e:
        logging.error(f"❌ เกิดข้อผิดพลาด Pydantic Validation สำหรับ Slug '{slug}': {e}", exc_info=True)
        logging.error(f"ข้อมูลที่ทำให้เกิดข้อผิดพลาดในการตรวจสอบความถูกต้อง: {location_data}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Data inconsistency error for location '{slug}'. Check server logs."
        )
    except Exception as e:
        logging.error(f"❌ เกิดข้อผิดพลาดที่ไม่คาดคิดในการดึงข้อมูลสถานที่ '{slug}': {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"An internal error occurred while fetching location '{slug}'."
        )


@router.put("/locations/{slug}", response_model=LocationInDB, tags=["Admin :: Locations CRUD"])
async def update_location_by_slug(
    slug: str,
    location_update: LocationBase,
    db: MongoDBManager = Depends(get_mongo_manager),
    vector_db: QdrantManager = Depends(get_qdrant_manager)
):
    logging.info(f"กำลังพยายามอัปเดตสถานที่ด้วย Slug: {slug}")
    if location_update.slug != slug:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST,
                            detail="Slug in URL parameter does not match slug in request body.")
    update_data = location_update.model_dump(exclude_unset=True)
    logging.debug(f"ข้อมูลอัปเดตสำหรับ Slug '{slug}': {update_data}")
    mongo_id = None
    updated_location = None
    try:
        modified_count = await asyncio.to_thread(db.update_location_by_slug, slug, update_data)
        if modified_count == 0:
            exists = await asyncio.to_thread(db.get_location_by_slug, slug)
            if not exists:
                logging.warning(f"การอัปเดตล้มเหลว: ไม่พบสถานที่สำหรับ Slug '{slug}'")
                raise HTTPException(status_code=status.HTTP_404_NOT_FOUND,
                                    detail=f"Location with slug '{slug}' not found.")
            logging.info(f"ได้รับคำขออัปเดตสถานที่ '{slug}' แต่ข้อมูลเหมือนเดิม ไม่มีการเปลี่ยนแปลง")
            updated_location = exists
        else:
             logging.info(f"อัปเดตข้อมูลใน MongoDB สำหรับ Slug '{slug}' สำเร็จ")
             updated_location = await asyncio.to_thread(db.get_location_by_slug, slug)
        if not updated_location:
             logging.error(f"ไม่สามารถดึงข้อมูลสถานที่ '{slug}' หลังจากอัปเดตได้")
             raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                                 detail="Could not retrieve location after update.")
        
        prefix = (updated_location.get("metadata") or {}).get("image_prefix")
        preview_url = _find_first_image_for_prefix(prefix)
        updated_model = LocationInDB(**updated_location, preview_image_url=preview_url)
        
        mongo_id = str(updated_model.mongo_id)
        # 🔄 [SYNC] MongoDB -> Qdrant (Update)
        # ส่วนนี้คือการอัปเดตข้อมูล Vector ใน Qdrant เมื่อมีการแก้ไขข้อมูลใน MongoDB
        # เช่น ถ้ามีการแก้ชื่อ หรือสรุปข้อมูล ก็ต้องอัปเดต Vector ใหม่ด้วย เพื่อให้ผลการค้นหายังคงถูกต้อง
        try:
            desc_title = updated_model.title or ''
            desc_topic = updated_model.topic or ''
            desc_summary = updated_model.summary or ''
            # เตรียมข้อความใหม่สำหรับการทำ Embedding 
            # เตรียมข้อความใหม่สำหรับการทำ Embedding 
            description_for_vector = f"หัวข้อ: {desc_title}\nประเภท: {desc_topic}\nสรุป: {desc_summary}"
            
            # 🆕 เตรียม Metadata สำหรับ Payload (Fix for Filter Bug)
            qdrant_metadata = {
                "title": updated_model.title,
                "slug": updated_model.slug,
                "category": updated_model.category,
                "district": (updated_model.related_info or {}).get("district"),
                "sub_district": (updated_model.related_info or {}).get("sub_district")
            }

            await vector_db.upsert_location(
                mongo_id=mongo_id, 
                description=description_for_vector,
                metadata=qdrant_metadata # 👈 ส่ง Metadata ไปด้วย
            )
            logging.info(f"ซิงค์ Vector สำหรับ mongo_id '{mongo_id}' (slug: '{slug}') สำเร็จ")
        except Exception as vector_e:
            logging.error(f"⚠️ คำเตือน: อัปเดต MongoDB สำหรับ slug '{slug}' แล้ว แต่ล้มเหลวในการซิงค์ Vector สำหรับ {mongo_id} ข้อผิดพลาด: {vector_e}", exc_info=True)
            
        return updated_model 
    except HTTPException as http_exc:
        raise http_exc
    except ValidationError as e:
         logging.error(f"❌ Pydantic Validation Error หลังจากอัปเดต Slug '{slug}': {e}", exc_info=True)
         raise HTTPException(
             status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
             detail=f"Data inconsistency error after update for location '{slug}'. Check server logs."
         )
    except Exception as e:
        logging.error(f"❌ เกิดข้อผิดพลาดที่ไม่คาดคิดในการอัปเดตสถานที่ '{slug}': {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"An internal error occurred while updating location '{slug}'."
        )

@router.delete("/locations/{slug}", status_code=status.HTTP_204_NO_CONTENT, tags=["Admin :: Locations CRUD"])
async def delete_location_by_slug(
    slug: str,
    db: MongoDBManager = Depends(get_mongo_manager),
    vector_db: QdrantManager = Depends(get_qdrant_manager)
):
    logging.info(f"กำลังพยายามลบสถานที่ด้วย Slug: {slug}")
    mongo_id = None
    try:
        location_to_delete = await asyncio.to_thread(db.get_location_by_slug, slug)
        if not location_to_delete:
            logging.warning(f"ลบไม่สำเร็จ: ไม่พบสถานที่สำหรับ Slug '{slug}'")
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND,
                                detail=f"Location with slug '{slug}' not found.")
        mongo_id = str(location_to_delete['_id'])
        logging.debug(f"พบสถานที่ที่จะลบ: slug='{slug}', mongo_id='{mongo_id}'")
        
        # ==========================================
        # 🔄 [SYNC] MongoDB -> Qdrant (Delete)
        # ส่วนนี้คือการลบข้อมูล Vector ใน Qdrant ออกเมื่อข้อมูลใน MongoDB ถูกลบ
        # เพื่อไม่ให้ค้นหาเจอข้อมูลที่ไม่มีอยู่จริงแล้ว (Ghost Data)
        # ==========================================
        try:
            vector_deleted = await vector_db.delete_vector(mongo_id)
            if not vector_deleted:
                logging.warning(f"⚠️ ไม่พบ Vector สำหรับ {mongo_id} (slug: {slug}) หรือลบใน Qdrant ล้มเหลว ดำเนินการลบใน MongoDB ต่อไป")
        except Exception as vector_e:
            logging.error(f"⚠️ คำเตือน: เกิดข้อผิดพลาดในการลบ Vector สำหรับ {mongo_id} ข้อผิดพลาด: {vector_e} ดำเนินการลบใน MongoDB ต่อไป", exc_info=True)
        deleted_count = await asyncio.to_thread(db.delete_location_by_slug, slug)
        if deleted_count == 0:
            logging.error(f"ความไม่สอดคล้องของการลบ: พบสถานที่ '{slug}' แต่ไม่สามารถลบออกจาก MongoDB ได้")
            raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                                detail=f"Location {slug} found but could not be deleted from MongoDB.")
        logging.info(f"✅ ลบสถานที่ {slug} (mongo_id: {mongo_id}) ออกจาก MongoDB สำเร็จ")
    except HTTPException as http_exc:
        raise http_exc
    except Exception as e:
        logging.error(f"❌ เกิดข้อผิดพลาดที่ไม่คาดคิดในการลบสถานที่ '{slug}' (mongo_id: {mongo_id}): {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"An internal error occurred while deleting location '{slug}'."
        )