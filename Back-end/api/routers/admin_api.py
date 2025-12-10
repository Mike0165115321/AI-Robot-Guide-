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
        logging.error(f"Error finding first image for prefix '{prefix}': {e}", exc_info=False)
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
                logging.info(f"🖼️  Image uploaded and saved as: {new_filename}")
                return new_filename
            except Exception as e:
                logging.error(f"❌ Error during file save (sync thread) for prefix '{prefix}': {e}", exc_info=True)
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
        logging.error(f"❌ Error uploading image for prefix '{image_prefix}': {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Could not upload image: {e}")

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
        logging.error(f"❌ Error fetching analytics dashboard: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Failed to fetch analytics data.")

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
    logging.info(f"Attempting to create new location with slug: {location_data.slug}")
    try:
        existing = await asyncio.to_thread(db.get_location_by_slug, location_data.slug)
        if existing:
            logging.warning(f"Create failed: Slug '{location_data.slug}' already exists.")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Slug '{location_data.slug}' นี้มีอยู่แล้ว กรุณาใช้ Slug อื่น"
            )
    except HTTPException as http_exc:
        raise http_exc
    except Exception as e:
        logging.error(f"Error checking existing slug '{location_data.slug}': {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Error checking for existing slug.")
    mongo_id_str = ""
    try:
        mongo_id_str = await asyncio.to_thread(
            db.add_location,
            location_data.model_dump()
        )
        if not mongo_id_str:
             raise Exception("Failed to create document in MongoDB (add_location returned None or empty string).")
        logging.info(f"Successfully created in MongoDB: slug='{location_data.slug}', mongo_id='{mongo_id_str}'")
    except Exception as e:
        logging.error(f"❌ Error creating location '{location_data.slug}' in MongoDB: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to create location in database: {e}"
        )
    try:
        desc_title = location_data.title
        desc_topic = location_data.topic
        desc_summary = location_data.summary
        description_for_vector = f"หัวข้อ: {desc_title}\nประเภท: {desc_topic}\nสรุป: {desc_summary}"
        await vector_db.upsert_location(mongo_id=mongo_id_str, description=description_for_vector)
        logging.info(f"Successfully created vector for mongo_id '{mongo_id_str}'.")
    except Exception as vector_e:
        logging.error(f"⚠️ WARNING: MongoDB created for slug '{location_data.slug}', but FAILED to create vector for {mongo_id_str}. Error: {vector_e}", exc_info=True)
    try:
        new_location_doc = await asyncio.to_thread(db.get_location_by_id, mongo_id_str)
        if not new_location_doc:
            raise Exception("Could not retrieve document immediately after creation.")
        return LocationInDB(**new_location_doc)
    except Exception as e:
         logging.error(f"❌ CRITICAL: MongoDB created (ID: {mongo_id_str}) but failed to retrieve it for response. Error: {e}", exc_info=True)
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
        logging.error(f"❌ Error during document analysis: {e}", exc_info=True)
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
                    logging.warning(f"Skipping invalid location data: {loc_dict}")
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
                    logging.warning(f"Skipping location due to validation error: {loc_dict.get('slug', 'N/A')}. Details: {e}")
                    continue

            return {
                "items": enriched_models,
                "total_count": total_count,
                "page": (skip // limit) + 1,
                "limit": limit
            }
        except Exception as e:
            logging.error(f"❌ Error enriching summaries in sync thread: {e}", exc_info=True)
            return {"items": [], "total_count": 0, "page": 1, "limit": limit}

    try:
        result = await asyncio.to_thread(get_paginated_summaries_sync)
        return result
    except Exception as e:
        logging.error(f"❌ Unexpected error getting all locations summary: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Internal server error retrieving location summaries.")


@router.get("/locations/{slug}", response_model=LocationInDB, tags=["Admin :: Locations CRUD"])
async def get_location_by_slug(
    slug: str,
    db: MongoDBManager = Depends(get_mongo_manager)
):
    logging.info(f"Attempting to fetch location with slug: {slug}")
    try:
        location_data = await asyncio.to_thread(db.get_location_by_slug, slug)

        if not location_data:
            logging.warning(f"Location not found for slug: {slug}")
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND,
                                detail=f"Location with slug '{slug}' not found.")

        prefix = (location_data.get("metadata") or {}).get("image_prefix")
        preview_url = _find_first_image_for_prefix(prefix)
        
        location_model = LocationInDB(
            **location_data,
            preview_image_url=preview_url
        )
        
        logging.debug(f"Raw data from DB for slug '{slug}': {location_data}")
        return location_model

    except HTTPException as http_exc:
        raise http_exc
    except ValidationError as e:
         logging.error(f"❌ Pydantic Validation Error for slug '{slug}': {e}", exc_info=True)
         logging.error(f"Data causing validation error: {location_data}")
         raise HTTPException(
             status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
             detail=f"Data inconsistency error for location '{slug}'. Check server logs."
         )
    except Exception as e:
        logging.error(f"❌ Unexpected error fetching location '{slug}': {e}", exc_info=True)
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
    logging.info(f"Attempting to update location with slug: {slug}")
    if location_update.slug != slug:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST,
                            detail="Slug in URL parameter does not match slug in request body.")
    update_data = location_update.model_dump(exclude_unset=True)
    logging.debug(f"Update data for slug '{slug}': {update_data}")
    mongo_id = None
    updated_location = None
    try:
        modified_count = await asyncio.to_thread(db.update_location_by_slug, slug, update_data)
        if modified_count == 0:
            exists = await asyncio.to_thread(db.get_location_by_slug, slug)
            if not exists:
                logging.warning(f"Update failed: Location not found for slug '{slug}'.")
                raise HTTPException(status_code=status.HTTP_404_NOT_FOUND,
                                    detail=f"Location with slug '{slug}' not found.")
            logging.info(f"Location '{slug}' update received, but data was identical. No changes made.")
            updated_location = exists
        else:
             logging.info(f"Successfully updated MongoDB for slug '{slug}'.")
             updated_location = await asyncio.to_thread(db.get_location_by_slug, slug)
        if not updated_location:
             logging.error(f"Failed to retrieve location '{slug}' after potential update.")
             raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                                 detail="Could not retrieve location after update.")
        
        prefix = (updated_location.get("metadata") or {}).get("image_prefix")
        preview_url = _find_first_image_for_prefix(prefix)
        updated_model = LocationInDB(**updated_location, preview_image_url=preview_url)
        
        mongo_id = str(updated_model.mongo_id)
        try:
            desc_title = updated_model.title or ''
            desc_topic = updated_model.topic or ''
            desc_summary = updated_model.summary or ''
            description_for_vector = f"หัวข้อ: {desc_title}\nประเภท: {desc_topic}\nสรุป: {desc_summary}"
            await vector_db.upsert_location(mongo_id=mongo_id, description=description_for_vector)
            logging.info(f"Successfully synced vector for mongo_id '{mongo_id}' (slug: '{slug}').")
        except Exception as vector_e:
            logging.error(f"⚠️ WARNING: MongoDB updated for slug '{slug}', but failed to sync vector for {mongo_id}. Error: {vector_e}", exc_info=True)
            
        return updated_model 
    except HTTPException as http_exc:
        raise http_exc
    except ValidationError as e:
         logging.error(f"❌ Pydantic Validation Error after updating slug '{slug}': {e}", exc_info=True)
         raise HTTPException(
             status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
             detail=f"Data inconsistency error after update for location '{slug}'. Check server logs."
         )
    except Exception as e:
        logging.error(f"❌ Unexpected error updating location '{slug}': {e}", exc_info=True)
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
    logging.info(f"Attempting to delete location with slug: {slug}")
    mongo_id = None
    try:
        location_to_delete = await asyncio.to_thread(db.get_location_by_slug, slug)
        if not location_to_delete:
            logging.warning(f"Deletion failed: Location not found for slug '{slug}'.")
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND,
                                detail=f"Location with slug '{slug}' not found.")
        mongo_id = str(location_to_delete['_id'])
        logging.debug(f"Found location to delete: slug='{slug}', mongo_id='{mongo_id}'")
        try:
            vector_deleted = await vector_db.delete_vector(mongo_id)
            if not vector_deleted:
                logging.warning(f"⚠️ Vector for {mongo_id} (slug: {slug}) not found or delete failed in Qdrant. Proceeding with MongoDB deletion.")
        except Exception as vector_e:
            logging.error(f"⚠️ WARNING: Error deleting vector for {mongo_id}. Error: {vector_e}. Proceeding with MongoDB deletion.", exc_info=True)
        deleted_count = await asyncio.to_thread(db.delete_location_by_slug, slug)
        if deleted_count == 0:
            logging.error(f"Deletion inconsistency: Location '{slug}' found but could not be deleted from MongoDB.")
            raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                                detail=f"Location {slug} found but could not be deleted from MongoDB.")
        logging.info(f"✅ Successfully deleted location {slug} (mongo_id: {mongo_id}) from MongoDB.")
    except HTTPException as http_exc:
        raise http_exc
    except Exception as e:
        logging.error(f"❌ Unexpected error deleting location '{slug}' (mongo_id: {mongo_id}): {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"An internal error occurred while deleting location '{slug}'."
        )