"""
🧠 Knowledge Gap API
API endpoints สำหรับ Admin จัดการคำถามที่ AI ตอบไม่ได้
"""

from fastapi import APIRouter, HTTPException, Depends, Query
from pydantic import BaseModel, Field
from typing import Optional, List
import logging

from core.services.knowledge_gap_service import get_knowledge_gap_service, KnowledgeGapService
from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from api.dependencies import get_mongo_manager, get_qdrant_manager

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/admin/knowledge-gaps", tags=["Knowledge Gaps"])


# ========== Dependencies ==========

def get_gap_service(
    mongo: MongoDBManager = Depends(get_mongo_manager),
    qdrant: QdrantManager = Depends(get_qdrant_manager)
) -> KnowledgeGapService:
    """Dependency injection for KnowledgeGapService"""
    return get_knowledge_gap_service(mongo, qdrant)


# ========== Request/Response Models ==========

class ResolveGapRequest(BaseModel):
    """Request body for resolving a knowledge gap"""
    answer: str = Field(..., min_length=10, description="คำตอบจาก Admin (อย่างน้อย 10 ตัวอักษร)")
    title: Optional[str] = Field(None, description="หัวข้อ (ถ้าไม่ระบุจะใช้คำถาม)")
    category: str = Field("ข้อมูลเพิ่มเติม", description="หมวดหมู่ของข้อมูล")
    admin_id: Optional[str] = Field(None, description="ID ของ Admin")


class DismissGapRequest(BaseModel):
    """Request body for dismissing a knowledge gap"""
    reason: Optional[str] = Field(None, description="เหตุผลที่ยกเลิก")
    admin_id: Optional[str] = Field(None, description="ID ของ Admin")


class GapResponse(BaseModel):
    """Response model for a single gap"""
    _id: str
    query: str
    max_score: float
    count: int
    status: str
    first_asked: str
    last_asked: str


class StatsResponse(BaseModel):
    """Response model for stats"""
    pending_count: int
    resolved_count: int
    dismissed_count: int
    resolved_today: int
    total: int


# ========== Endpoints ==========

@router.get("")
async def get_knowledge_gaps(
    limit: int = Query(50, ge=1, le=200, description="จำนวน max ที่จะดึง"),
    sort_by: str = Query("count", regex="^(count|last_asked)$", description="เรียงตาม count หรือ last_asked"),
    include_resolved: bool = Query(False, description="รวมคำถามที่ตอบแล้ว"),
    service: KnowledgeGapService = Depends(get_gap_service)
):
    """
    📋 ดึงรายการคำถามที่ AI ตอบไม่ได้
    
    - **limit**: จำนวน max ที่จะดึง (default: 50)
    - **sort_by**: เรียงตาม 'count' (ยอดนิยม) หรือ 'last_asked' (ล่าสุด)
    - **include_resolved**: รวมคำถามที่ตอบแล้วด้วยหรือไม่
    """
    gaps = await service.get_pending_gaps(
        limit=limit,
        sort_by=sort_by,
        include_resolved=include_resolved
    )
    return {
        "success": True,
        "count": len(gaps),
        "gaps": gaps
    }


@router.get("/stats")
async def get_knowledge_gap_stats(
    service: KnowledgeGapService = Depends(get_gap_service)
):
    """
    📊 ดึงสถิติภาพรวมของ Knowledge Gaps
    
    Returns:
        - pending_count: จำนวนคำถามที่รอตอบ
        - resolved_count: จำนวนที่ตอบแล้ว
        - resolved_today: จำนวนที่ตอบวันนี้
        - top_recurring: Top 5 คำถามที่ถูกถามบ่อยสุด
    """
    stats = await service.get_stats()
    return {
        "success": True,
        **stats
    }


@router.get("/{gap_id}")
async def get_single_gap(
    gap_id: str,
    service: KnowledgeGapService = Depends(get_gap_service)
):
    """
    🔍 ดึงข้อมูล gap เดียวตาม ID
    """
    gap = await service.get_gap_by_id(gap_id)
    if not gap:
        raise HTTPException(status_code=404, detail="Gap not found")
    return {
        "success": True,
        "gap": gap
    }


@router.post("/{gap_id}/resolve")
async def resolve_knowledge_gap(
    gap_id: str,
    request: ResolveGapRequest,
    service: KnowledgeGapService = Depends(get_gap_service)
):
    """
    ✅ Admin ตอบคำถาม
    
    เมื่อตอบแล้ว:
    1. อัปเดตสถานะเป็น RESOLVED
    2. สร้าง document ใหม่ใน Qdrant Vector DB
    3. AI จะสามารถตอบคำถามนี้ได้ในครั้งต่อไป
    """
    result = await service.resolve_gap(
        gap_id=gap_id,
        answer=request.answer,
        admin_id=request.admin_id,
        title=request.title,
        category=request.category
    )
    
    if not result.get("success"):
        raise HTTPException(status_code=400, detail=result.get("error", "Unknown error"))
    
    return result


@router.post("/{gap_id}/dismiss")
async def dismiss_knowledge_gap(
    gap_id: str,
    request: DismissGapRequest,
    service: KnowledgeGapService = Depends(get_gap_service)
):
    """
    ❌ ยกเลิกคำถาม
    
    ใช้เมื่อคำถามไม่เกี่ยวกับน่าน หรือเป็น spam
    """
    result = await service.dismiss_gap(
        gap_id=gap_id,
        reason=request.reason,
        admin_id=request.admin_id
    )
    
    if not result.get("success"):
        raise HTTPException(status_code=400, detail=result.get("error", "Unknown error"))
    
    return result
