# api/routers/admin_api.py (ฉบับแก้ไข)
from fastapi import APIRouter, HTTPException, Body, status, Depends
from typing import List
from api.schemas import LocationCreate, LocationInDB, LocationAdminSummary, LocationBase # <--- เพิ่ม import LocationBase
from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from bson import ObjectId
from fastapi import File, UploadFile
from core.document_processor import DocumentProcessor

mongo_manager = MongoDBManager()
qdrant_manager = QdrantManager()

def get_mongo_manager():
    return mongo_manager

def get_qdrant_manager():
    return qdrant_manager

router = APIRouter(
    tags=["Admin Locations"]
)

@router.post("/", response_model=LocationInDB, status_code=status.HTTP_201_CREATED)
def create_location(
    location: LocationCreate,
    db: MongoDBManager = Depends(get_mongo_manager),
    vector_db: QdrantManager = Depends(get_qdrant_manager)
):
    location_dict = location.dict()
    mongo_id = db.add_location(location_dict)
    if not mongo_id:
        raise HTTPException(status_code=500, detail="Failed to create location in MongoDB.")
    
    try:
        description_for_vector = f"หัวข้อ: {location.title}\nประเภท: {location.topic}\nสรุป: {location.summary}"
        vector_db.upsert_location(mongo_id=mongo_id, description=description_for_vector)
    except Exception as e:
        db.delete_location(mongo_id)
        raise HTTPException(status_code=500, detail=f"Failed to create vector in Qdrant: {e}")

    created_location = db.get_location_by_id(mongo_id)
    if not created_location:
        raise HTTPException(status_code=404, detail="Could not retrieve newly created location.")
        
    created_location['_id'] = str(created_location['_id'])
    return created_location


@router.get("/", response_model=List[LocationAdminSummary]) 
def get_all_locations_summary(db: MongoDBManager = Depends(get_mongo_manager)):
    """
    ดึงข้อมูลสถานที่ทั้งหมด (เฉพาะ field ที่จำเป็นสำหรับหน้า Admin list)
    """
    all_locations = db.get_all_locations()
    for loc in all_locations:
        loc['_id'] = str(loc['_id'])
    return all_locations

@router.get("/{location_id}", response_model=LocationInDB)
def get_location_by_id(location_id: str, db: MongoDBManager = Depends(get_mongo_manager)):
    """
    ดึงข้อมูลสถานที่รายการเดียวแบบเต็ม
    """
    if not ObjectId.is_valid(location_id):
        raise HTTPException(status_code=400, detail="Invalid MongoDB ObjectId format.")
        
    location = db.get_location_by_id(location_id)
    if not location:
        raise HTTPException(status_code=404, detail="Location not found.")
    
    location['_id'] = str(location['_id'])
    return location

@router.put("/{location_id}", response_model=LocationInDB)
def update_location(
    location_id: str, 
    location_update: LocationBase,
    db: MongoDBManager = Depends(get_mongo_manager),
    vector_db: QdrantManager = Depends(get_qdrant_manager)
):
    if not ObjectId.is_valid(location_id):
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Invalid MongoDB ObjectId format.")

    update_data = location_update.model_dump(exclude_unset=True)
    modified_count = db.update_location(location_id, update_data)

    if modified_count == 0:
        if not db.get_location_by_id(location_id):
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Location not found.")
    
    if modified_count > 0:
        try:
            description_for_vector = f"หัวข้อ: {location_update.title}\nประเภท: {location_update.topic}\nสรุป: {location_update.summary}"
            vector_db.upsert_location(mongo_id=location_id, description=description_for_vector)
        except Exception as e:
            print(f"⚠️ WARNING: Failed to update vector for {location_id}, but MongoDB was updated. Error: {e}")

    updated_location = db.get_location_by_id(location_id)
    if not updated_location:
         raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Could not find location after update.")
    updated_location['_id'] = str(updated_location['_id'])
    return updated_location


@router.post("/analyze-document", tags=["Admin Document Analysis"])
async def analyze_document_endpoint(file: UploadFile = File(...)):
    """
    รับไฟล์ (PDF/Image) จาก Frontend, ประมวลผลด้วย AI,
    และส่งข้อมูลที่สกัดได้กลับไป
    """
    file_content = await file.read()
    
    if not file_content:
        raise HTTPException(status_code=400, detail="No file content received.")
    
    try:
        processor = DocumentProcessor()
        extracted_data = processor.analyze_document(
            file_content=file_content,
            content_type=file.content_type
        )
        
        if not extracted_data:
            raise HTTPException(status_code=500, detail="Failed to process document or extract data.")
            
        return extracted_data

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"An unexpected error occurred: {str(e)}")


@router.delete("/{location_id}", status_code=status.HTTP_204_NO_CONTENT)
def delete_location(
    location_id: str,
    db: MongoDBManager = Depends(get_mongo_manager),
    vector_db: QdrantManager = Depends(get_qdrant_manager)
):
    if not ObjectId.is_valid(location_id):
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Invalid MongoDB ObjectId format.")

    vector_deleted = vector_db.delete_vector(location_id)
    if not vector_deleted:
        print(f"⚠️ WARNING: Could not delete vector for {location_id}. Proceeding to delete from MongoDB.")

    deleted_count = db.delete_location(location_id)
    if deleted_count == 0:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Location not found in MongoDB.")

    return