"""
🧠 Knowledge Gap Service
ระบบจัดการช่องว่างความรู้ของ AI (Self-Correcting RAG)

Features:
- บันทึกคำถามที่ AI ตอบไม่ได้ (Low Confidence)
- นับจำนวนครั้งที่คำถามเดียวกันถูกถามซ้ำ
- Admin สามารถเพิ่มคำตอบ → บรรจุเข้า Qdrant
"""

import logging
import asyncio
from datetime import datetime, timezone
from typing import Dict, List, Optional, Any
from bson import ObjectId

logger = logging.getLogger(__name__)


class KnowledgeGapService:
    """บริการจัดการช่องว่างความรู้ของ AI"""

    def __init__(self, mongo_manager, qdrant_manager=None):
        self.mongo_manager = mongo_manager
        self.qdrant_manager = qdrant_manager
        self.collection = self.mongo_manager.get_collection("unanswered_questions")
        logger.info("🧠 Knowledge Gap Service initialized")

    async def log_unanswered(
        self, 
        query: str, 
        score: float, 
        session_id: str = None,
        context: str = None
    ) -> Optional[str]:
        """
        บันทึกคำถามที่ AI ตอบไม่ได้ลง MongoDB
        
        ถ้าคำถามเคยถูกถามแล้ว → เพิ่ม count
        ถ้าใหม่ → สร้าง entry ใหม่
        
        Returns:
            str: ID ของ document (new หรือ existing)
        """
        if self.collection is None:
            logger.warning("⚠️ [KnowledgeGap] Collection not available, skipping log")
            return None

        try:
            # Normalize query (lowercase, strip)
            normalized_query = query.strip().lower()
            
            # Check if similar question already exists
            existing = await asyncio.to_thread(
                self.collection.find_one,
                {"normalized_query": normalized_query, "status": "PENDING"}
            )

            now = datetime.now(timezone.utc)

            if existing:
                # Increment count and update last_asked
                await asyncio.to_thread(
                    self.collection.update_one,
                    {"_id": existing["_id"]},
                    {
                        "$inc": {"count": 1},
                        "$set": {"last_asked": now},
                        "$push": {
                            "sessions": {
                                "$each": [session_id] if session_id else [],
                                "$slice": -10  # Keep last 10 sessions
                            }
                        }
                    }
                )
                logger.info(f"📝 [KnowledgeGap] คำถามซ้ำ (count: {existing['count'] + 1}): '{query[:50]}...'")
                return str(existing["_id"])
            else:
                # Create new entry
                doc = {
                    "query": query,
                    "normalized_query": normalized_query,
                    "max_score": round(score, 4),
                    "count": 1,
                    "first_asked": now,
                    "last_asked": now,
                    "status": "PENDING",  # PENDING | RESOLVED | DISMISSED
                    "sessions": [session_id] if session_id else [],
                    "context": context,  # Optional: context from RAG search
                    "resolved_answer": None,
                    "resolved_by": None,
                    "resolved_at": None,
                    "dismiss_reason": None
                }
                result = await asyncio.to_thread(self.collection.insert_one, doc)
                logger.info(f"🆕 [KnowledgeGap] คำถามใหม่ที่ตอบไม่ได้: '{query[:50]}...' (score: {score:.4f})")
                return str(result.inserted_id)

        except Exception as e:
            logger.error(f"❌ [KnowledgeGap] บันทึกล้มเหลว: {e}", exc_info=True)
            return None

    async def get_pending_gaps(
        self, 
        limit: int = 50, 
        sort_by: str = "count",
        include_resolved: bool = False
    ) -> List[Dict[str, Any]]:
        """
        ดึงรายการคำถามที่รอการตอบ
        
        Args:
            limit: จำนวน max ที่จะดึง
            sort_by: 'count' (ยอดนิยม) หรือ 'last_asked' (ล่าสุด)
            include_resolved: รวมคำถามที่ตอบแล้วด้วยหรือไม่
            
        Returns:
            List of gap documents
        """
        if self.collection is None:
            return []

        try:
            query_filter = {}
            if not include_resolved:
                query_filter["status"] = "PENDING"

            sort_field = "count" if sort_by == "count" else "last_asked"
            
            cursor = self.collection.find(query_filter).sort(sort_field, -1).limit(limit)
            results = await asyncio.to_thread(list, cursor)
            
            # Convert ObjectId to string for JSON serialization
            for doc in results:
                doc["_id"] = str(doc["_id"])
                
            logger.info(f"📋 [KnowledgeGap] ดึงข้อมูล {len(results)} รายการ (filter: {query_filter})")
            return results

        except Exception as e:
            logger.error(f"❌ [KnowledgeGap] ดึงข้อมูลล้มเหลว: {e}", exc_info=True)
            return []

    async def get_stats(self) -> Dict[str, Any]:
        """
        ดึงสถิติภาพรวมของ Knowledge Gaps
        
        Returns:
            Dict with stats: pending_count, resolved_today, top_recurring, etc.
        """
        if self.collection is None:
            return {"error": "Collection not available"}

        try:
            # Count by status
            pending_count = await asyncio.to_thread(
                self.collection.count_documents, {"status": "PENDING"}
            )
            resolved_count = await asyncio.to_thread(
                self.collection.count_documents, {"status": "RESOLVED"}
            )
            dismissed_count = await asyncio.to_thread(
                self.collection.count_documents, {"status": "DISMISSED"}
            )

            # Resolved today
            today_start = datetime.now(timezone.utc).replace(hour=0, minute=0, second=0, microsecond=0)
            resolved_today = await asyncio.to_thread(
                self.collection.count_documents,
                {"status": "RESOLVED", "resolved_at": {"$gte": today_start}}
            )

            # Top recurring questions (most asked)
            top_cursor = self.collection.find({"status": "PENDING"}).sort("count", -1).limit(5)
            top_recurring = await asyncio.to_thread(list, top_cursor)
            for doc in top_recurring:
                doc["_id"] = str(doc["_id"])

            return {
                "pending_count": pending_count,
                "resolved_count": resolved_count,
                "dismissed_count": dismissed_count,
                "resolved_today": resolved_today,
                "total": pending_count + resolved_count + dismissed_count,
                "top_recurring": top_recurring
            }

        except Exception as e:
            logger.error(f"❌ [KnowledgeGap] ดึงสถิติล้มเหลว: {e}", exc_info=True)
            return {"error": str(e)}

    async def resolve_gap(
        self, 
        gap_id: str, 
        answer: str, 
        admin_id: str = None,
        title: str = None,
        category: str = "ข้อมูลเพิ่มเติม"
    ) -> Dict[str, Any]:
        """
        Admin ตอบคำถาม และบรรจุเข้า Qdrant Vector DB
        
        Args:
            gap_id: ID ของ gap document
            answer: คำตอบจาก Admin
            admin_id: ID ของ Admin ที่ตอบ
            title: หัวข้อ (ถ้าไม่ระบุจะใช้คำถาม)
            category: หมวดหมู่ของข้อมูล
            
        Returns:
            Dict with success status and new document info
        """
        if self.collection is None:
            return {"success": False, "error": "Collection not available"}

        try:
            # Get the gap document
            gap_doc = await asyncio.to_thread(
                self.collection.find_one,
                {"_id": ObjectId(gap_id)}
            )

            if not gap_doc:
                return {"success": False, "error": "Gap not found"}

            if gap_doc["status"] != "PENDING":
                return {"success": False, "error": f"Gap already {gap_doc['status']}"}

            now = datetime.now(timezone.utc)
            query = gap_doc["query"]

            # 1. Update gap status to RESOLVED
            await asyncio.to_thread(
                self.collection.update_one,
                {"_id": ObjectId(gap_id)},
                {
                    "$set": {
                        "status": "RESOLVED",
                        "resolved_answer": answer,
                        "resolved_by": admin_id,
                        "resolved_at": now
                    }
                }
            )

            # 2. Create new document in Qdrant (if available)
            qdrant_id = None
            if self.qdrant_manager:
                doc_title = title or query
                new_doc = {
                    "title": doc_title,
                    "summary": answer,
                    "category": category,
                    "doc_type": "KnowledgeGap",
                    "source": "admin_resolved",
                    "original_question": query,
                    "created_at": now.isoformat(),
                    "metadata": {
                        "gap_id": gap_id,
                        "resolved_by": admin_id,
                        "ask_count": gap_doc.get("count", 1)
                    }
                }
                
                # Add to Qdrant
                qdrant_id = await self.qdrant_manager.upsert_document(
                    text=f"{doc_title}. {answer}",
                    metadata=new_doc
                )
                logger.info(f"✅ [KnowledgeGap] บรรจุเข้า Qdrant สำเร็จ: {qdrant_id}")

            logger.info(f"✅ [KnowledgeGap] Resolved: '{query[:50]}...' โดย Admin: {admin_id}")

            return {
                "success": True,
                "gap_id": gap_id,
                "qdrant_id": qdrant_id,
                "message": f"เพิ่มคำตอบสำเร็จ: '{query[:50]}...'"
            }

        except Exception as e:
            logger.error(f"❌ [KnowledgeGap] Resolve ล้มเหลว: {e}", exc_info=True)
            return {"success": False, "error": str(e)}

    async def dismiss_gap(
        self, 
        gap_id: str, 
        reason: str = None,
        admin_id: str = None
    ) -> Dict[str, Any]:
        """
        ยกเลิกคำถาม (เช่น spam, ไม่เกี่ยวกับน่าน)
        
        Args:
            gap_id: ID ของ gap document
            reason: เหตุผลที่ยกเลิก
            admin_id: ID ของ Admin
            
        Returns:
            Dict with success status
        """
        if self.collection is None:
            return {"success": False, "error": "Collection not available"}

        try:
            result = await asyncio.to_thread(
                self.collection.update_one,
                {"_id": ObjectId(gap_id)},
                {
                    "$set": {
                        "status": "DISMISSED",
                        "dismiss_reason": reason,
                        "resolved_by": admin_id,
                        "resolved_at": datetime.now(timezone.utc)
                    }
                }
            )

            if result.modified_count > 0:
                logger.info(f"🚫 [KnowledgeGap] Dismissed: {gap_id} | Reason: {reason}")
                return {"success": True, "message": "ยกเลิกคำถามสำเร็จ"}
            else:
                return {"success": False, "error": "Gap not found or already processed"}

        except Exception as e:
            logger.error(f"❌ [KnowledgeGap] Dismiss ล้มเหลว: {e}", exc_info=True)
            return {"success": False, "error": str(e)}

    async def get_gap_by_id(self, gap_id: str) -> Optional[Dict[str, Any]]:
        """ดึง gap document ตาม ID"""
        if self.collection is None:
            return None

        try:
            doc = await asyncio.to_thread(
                self.collection.find_one,
                {"_id": ObjectId(gap_id)}
            )
            if doc:
                doc["_id"] = str(doc["_id"])
            return doc
        except Exception as e:
            logger.error(f"❌ [KnowledgeGap] ดึงข้อมูลล้มเหลว: {e}")
            return None


# Singleton instance
_knowledge_gap_service: Optional[KnowledgeGapService] = None


def get_knowledge_gap_service(mongo_manager=None, qdrant_manager=None) -> KnowledgeGapService:
    """Get or create singleton instance"""
    global _knowledge_gap_service
    if _knowledge_gap_service is None:
        if mongo_manager is None:
            raise ValueError("mongo_manager required for first initialization")
        _knowledge_gap_service = KnowledgeGapService(mongo_manager, qdrant_manager)
    return _knowledge_gap_service
