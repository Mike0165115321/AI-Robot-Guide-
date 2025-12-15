"""
Google Sheets Sync API Endpoints
รองรับการเชื่อมต่อ, sync, และ webhook
"""

from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks
from pydantic import BaseModel
from typing import Optional, Dict, Any

from core.services.google_sheets_service import get_sheets_service, GoogleSheetsService
from core.database.mongodb_manager import MongoDBManager
from api.dependencies import get_mongo_manager

router = APIRouter(tags=["Google Sheets Sync"])


class ConnectRequest(BaseModel):
    sheet_url: Optional[str] = None
    sheet_id: Optional[str] = None


class WebhookPayload(BaseModel):
    """Payload จาก Google Apps Script"""
    event: str  # "edit", "insert", "delete"
    row_data: Optional[Dict[str, Any]] = None
    slug: Optional[str] = None


# Store connected sheet config
_sheets_config = {
    "sheet_id": None,
    "sheet_url": None,
    "auto_sync_enabled": False,
    "sync_interval_minutes": 5
}


@router.post("/connect", response_model=Dict[str, Any])
async def connect_google_sheet(
    request: ConnectRequest,
    mongo: MongoDBManager = Depends(get_mongo_manager)
):
    """
    เชื่อมต่อ Google Sheet สำหรับ sync
    
    - **sheet_url**: URL เต็มของ Google Sheet
    - **sheet_id**: หรือใส่แค่ ID ก็ได้
    """
    if not request.sheet_url and not request.sheet_id:
        raise HTTPException(status_code=400, detail="ต้องระบุ sheet_url หรือ sheet_id")
    
    service = get_sheets_service(mongo)
    success = service.connect(
        sheet_id=request.sheet_id,
        sheet_url=request.sheet_url
    )
    
    if not success:
        raise HTTPException(
            status_code=400, 
            detail="เชื่อมต่อไม่สำเร็จ - ตรวจสอบว่า Share sheet ให้ Service Account แล้ว"
        )
    
    # Save config
    _sheets_config["sheet_id"] = service.sheet_id
    _sheets_config["sheet_url"] = request.sheet_url
    
    return {
        "success": True,
        "message": f"เชื่อมต่อสำเร็จ: {service.spreadsheet.title}",
        "status": service.get_status()
    }


@router.get("/status", response_model=Dict[str, Any])
async def get_sync_status(
    mongo: MongoDBManager = Depends(get_mongo_manager)
):
    """ดูสถานะการเชื่อมต่อและ sync"""
    service = get_sheets_service(mongo)
    return {
        "connection": service.get_status(),
        "config": _sheets_config
    }


@router.get("/check-availability", response_model=Dict[str, Any])
async def check_sheets_availability():
    """
    ตรวจสอบว่ามี credentials สำหรับ Google Sheets หรือไม่
    Frontend ใช้เพื่อแสดง UI ที่เหมาะสม
    """
    from core.services.google_sheets_service import CREDENTIALS_PATH
    
    has_credentials = CREDENTIALS_PATH.exists()
    
    return {
        "has_credentials": has_credentials,
        "available_modes": ["public"] if not has_credentials else ["public", "service_account"],
        "recommended_mode": "service_account" if has_credentials else "public",
        "message": "กรุณา Share Google Sheet เป็น 'Anyone with the link' สำหรับ Public Mode" if not has_credentials else None
    }


@router.post("/connect-public", response_model=Dict[str, Any])
async def connect_public_sheet(
    request: ConnectRequest,
    mongo: MongoDBManager = Depends(get_mongo_manager)
):
    """
    เชื่อมต่อ Google Sheet แบบ Public (ไม่ต้อง credentials)
    
    Sheet ต้องถูก Share เป็น "Anyone with the link" ก่อน
    
    - **sheet_url**: URL เต็มของ Google Sheet
    """
    if not request.sheet_url:
        raise HTTPException(status_code=400, detail="ต้องระบุ sheet_url")
    
    service = get_sheets_service(mongo)
    success = service.connect_public(request.sheet_url)
    
    if not success:
        raise HTTPException(
            status_code=400, 
            detail="เชื่อมต่อไม่สำเร็จ - ตรวจสอบว่า Sheet ถูกแชร์เป็น 'ทุกคนที่มีลิงก์' แล้ว และ URL ถูกคัดลอกมาครบถ้วน"
        )
    
    # Save config
    _sheets_config["sheet_id"] = service.sheet_id
    _sheets_config["sheet_url"] = request.sheet_url
    _sheets_config["mode"] = "public"
    
    return {
        "success": True,
        "message": f"เชื่อมต่อสำเร็จ (Public Mode): {service.sheet_title}",
        "status": service.get_status()
    }


@router.post("/sync-now", response_model=Dict[str, Any])
async def sync_now(
    mongo: MongoDBManager = Depends(get_mongo_manager)
):
    """
    บังคับ sync ทันที (Polling mode manual trigger)
    รองรับทั้ง Public Mode และ Service Account Mode
    """
    service = get_sheets_service(mongo)
    
    # [FIX] Check sheet_id instead of spreadsheet - Public Mode uses sheet_id only
    if not service.sheet_id:
        raise HTTPException(status_code=400, detail="ยังไม่ได้เชื่อมต่อ Sheet กรุณาวางลิงก์และกดเชื่อมต่อก่อน")
    
    result = service.full_sync()
    
    return {
        "success": len(result.errors) == 0,
        "result": result.to_dict()
    }


@router.post("/webhook", response_model=Dict[str, Any])
async def receive_webhook(
    payload: WebhookPayload,
    background_tasks: BackgroundTasks,
    mongo: MongoDBManager = Depends(get_mongo_manager)
):
    """
    รับ webhook จาก Google Apps Script (Real-time mode)
    
    Events:
    - **edit**: แก้ไข row
    - **insert**: เพิ่ม row ใหม่
    - **delete**: ลบ row
    """
    service = get_sheets_service(mongo)
    
    if payload.event == "edit" and payload.row_data:
        # Update existing
        slug = payload.row_data.get("slug") or payload.slug
        if slug:
            try:
                normalized = service._normalize_row(payload.row_data)
                mongo.update_location(slug, normalized)
                return {"success": True, "action": "updated", "slug": slug}
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
    
    elif payload.event == "insert" and payload.row_data:
        # Create new
        try:
            normalized = service._normalize_row(payload.row_data)
            mongo.add_location(normalized)
            return {"success": True, "action": "created", "slug": normalized.get("slug")}
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))
    
    elif payload.event == "delete" and payload.slug:
        # Delete
        try:
            mongo.delete_location(payload.slug)
            return {"success": True, "action": "deleted", "slug": payload.slug}
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))
    
    return {"success": False, "message": "Unknown event or missing data"}


@router.delete("/disconnect", response_model=Dict[str, Any])
async def disconnect_sheet():
    """ยกเลิกการเชื่อมต่อ Google Sheet (เก็บข้อมูลไว้)"""
    global _sheets_config
    _sheets_config = {
        "sheet_id": None,
        "sheet_url": None,
        "auto_sync_enabled": False,
        "sync_interval_minutes": 5
    }
    
    # Reset service
    service = get_sheets_service()
    service.spreadsheet = None
    service.worksheet = None
    service.sheet_id = None
    
    return {"success": True, "message": "ยกเลิกการเชื่อมต่อแล้ว (ข้อมูลยังคงอยู่)"}


@router.delete("/disconnect-and-delete", response_model=Dict[str, Any])
async def disconnect_and_delete_sheet_data(
    mongo: MongoDBManager = Depends(get_mongo_manager)
):
    """
    ยกเลิกการเชื่อมต่อ Google Sheet และลบข้อมูลทั้งหมดที่ sync มาจาก Sheet นี้
    
    ⚠️ คำเตือน: ข้อมูลที่ลบจะไม่สามารถกู้คืนได้
    """
    global _sheets_config
    
    service = get_sheets_service(mongo)
    sheet_id = service.sheet_id
    
    if not sheet_id:
        raise HTTPException(status_code=400, detail="ยังไม่ได้เชื่อมต่อ Sheet ใดๆ")
    
    # Delete all data from this sheet
    deleted_count = mongo.delete_locations_by_sheet_id(sheet_id)
    
    # Reset connection
    _sheets_config = {
        "sheet_id": None,
        "sheet_url": None,
        "auto_sync_enabled": False,
        "sync_interval_minutes": 5
    }
    
    # Reset service
    service.spreadsheet = None
    service.worksheet = None
    service.sheet_id = None
    service.sheet_title = None
    service.connection_mode = None
    
    return {
        "success": True,
        "message": f"ยกเลิกการเชื่อมต่อและลบข้อมูลทั้งหมด {deleted_count} รายการแล้ว",
        "deleted_count": deleted_count
    }
