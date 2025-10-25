import asyncio
import shutil
import logging
from pathlib import Path
from typing import List, Dict, Any

from bson import ObjectId
from fastapi import (APIRouter, Body, Depends, File, HTTPException, UploadFile,
                     status)
from pydantic import ValidationError

from ..schemas import (LocationAdminSummaryWithImage, LocationBase, LocationInDB,
                       LocationAdminSummary)
from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from core.document_processor import DocumentProcessor
from ..dependencies import get_mongo_manager, get_qdrant_manager

router = APIRouter(tags=["Admin"])

STATIC_IMAGE_DIR = Path(__file__).resolve().parent.parent.parent / "static" / "images"
STATIC_IMAGE_DIR.mkdir(parents=True, exist_ok=True)


def _find_first_image_for_prefix(prefix: str) -> str | None:
    if not prefix:
        return None
    try:
        for f in STATIC_IMAGE_DIR.glob(f"{prefix}*"):
            if f.is_file() and f.suffix.lower() in ('.jpg', '.jpeg', '.png', '.webp'):
                return f"/static/images/{f.name}"
        return None
    except Exception as e:
        logging.error(f"Error finding first image for prefix '{prefix}': {e}", exc_info=False) # Log error quietly
        return None


@router.post("/upload-image/", tags=["Admin :: Image Upload"])
async def upload_location_image(
    image_prefix: str = Body(..., description="Prefix ที่ตรงกับ 'slug' ของสถานที่ เช่น 'wat_phumin_'"), # Updated example
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

        def save_file_in_thread(prefix: str, content: bytes, extension: str) -> str | None: # Return None on error
            try:
                existing_files = [f for f in STATIC_IMAGE_DIR.iterdir() if f.name.startswith(f"{prefix}-")]
                next_index = len(existing_files) + 1

                new_filename = f"{prefix}-{next_index:02d}{extension}" # Use hyphen for index separation
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


@router.post("/analyze-document", tags=["Admin :: Document Analysis"])
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


@router.get("/", response_model=List[LocationAdminSummaryWithImage], tags=["Admin :: Locations CRUD"])
async def get_all_locations_summary(
    db: MongoDBManager = Depends(get_mongo_manager)
):
    def get_enriched_summaries_sync() -> List[Dict[str, Any]]:
        try:
            locations = db.get_all_locations() 
            enriched_locations = []

            for loc in locations:
                if not isinstance(loc, dict) or '_id' not in loc:
                    logging.warning(f"Skipping invalid location data: {loc}")
                    continue

                loc['_id'] = str(loc['_id']) 

                prefix = loc.get("metadata", {}).get("image_prefix")
                loc["preview_image_url"] = _find_first_image_for_prefix(prefix)

                enriched_locations.append(loc)
            return enriched_locations
        except Exception as e:
            logging.error(f"❌ Error enriching summaries in sync thread: {e}", exc_info=True)
            return [] 

    try:
        all_locations_enriched = await asyncio.to_thread(get_enriched_summaries_sync)

        return all_locations_enriched
    except ValidationError as e:
         logging.error(f"❌ Pydantic Validation Error during summary enrichment: {e}", exc_info=True)
         raise HTTPException(
             status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
             detail="Data inconsistency error during summary preparation. Check server logs."
         )
    except Exception as e:
        logging.error(f"❌ Unexpected error getting all locations summary: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Internal server error retrieving location summaries.")


@router.get("/{slug}", response_model=LocationInDB, tags=["Admin :: Locations CRUD"])
async def get_location_by_slug(
    slug: str,
    db: MongoDBManager = Depends(get_mongo_manager)
):
    logging.info(f"Attempting to fetch location with slug: {slug}")
    location_data = None
    try:
        location_data = await asyncio.to_thread(db.get_location_by_slug, slug)

        if not location_data:
            logging.warning(f"Location not found for slug: {slug}")
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND,
                                detail=f"Location with slug '{slug}' not found.")

        logging.debug(f"Raw data from DB for slug '{slug}': {location_data}")
        return location_data

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


@router.put("/{slug}", response_model=LocationInDB, tags=["Admin :: Locations CRUD"])
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

        mongo_id = str(updated_location['_id']) 

        try:
            desc_title = updated_location.get('title', '')
            desc_topic = updated_location.get('topic', '')
            desc_summary = updated_location.get('summary', '')
            description_for_vector = f"หัวข้อ: {desc_title}\nประเภท: {desc_topic}\nสรุป: {desc_summary}"

            await vector_db.upsert_location(mongo_id=mongo_id, description=description_for_vector)
            logging.info(f"Successfully synced vector for mongo_id '{mongo_id}' (slug: '{slug}').")

        except Exception as vector_e:
            logging.error(f"⚠️ WARNING: MongoDB updated for slug '{slug}', but failed to sync vector for {mongo_id}. Error: {vector_e}", exc_info=True)

        return updated_location

    except HTTPException as http_exc:
        raise http_exc
    except ValidationError as e:
         # Catch validation errors if fetching updated data fails schema check
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


@router.delete("/{slug}", status_code=status.HTTP_204_NO_CONTENT, tags=["Admin :: Locations CRUD"])
async def delete_location_by_slug(
    slug: str,
    db: MongoDBManager = Depends(get_mongo_manager),
    vector_db: QdrantManager = Depends(get_qdrant_manager)
):
    """
    [V5.1 Refactor] ลบข้อมูลสถานที่ด้วย 'slug' และลบข้อมูลใน Vector DB
    """
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
