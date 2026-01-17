"""
🧪 Unit Tests for Knowledge Gap Service
Tests the Self-Correcting RAG functionality
"""

import pytest
from unittest.mock import MagicMock, AsyncMock, patch
from datetime import datetime, timezone
from bson import ObjectId
import asyncio
import sys
from pathlib import Path

# Add Back-end to path
BACKEND_DIR = Path(__file__).parent.parent.parent / "Back-end"
sys.path.insert(0, str(BACKEND_DIR))


class TestKnowledgeGapService:
    """Test suite for KnowledgeGapService"""

    @pytest.fixture
    def service(self, mock_mongo, mock_qdrant):
        """Create a KnowledgeGapService instance with mocks"""
        from core.services.knowledge_gap_service import KnowledgeGapService
        return KnowledgeGapService(mock_mongo, mock_qdrant)

    # ========== log_unanswered Tests ==========
    
    @pytest.mark.asyncio
    async def test_log_unanswered_creates_new_entry(self, service, mock_mongo):
        """Test that a new unanswered query creates a new entry"""
        # Setup: No existing entry
        mock_collection = mock_mongo.get_collection.return_value
        mock_collection.find_one = MagicMock(return_value=None)
        mock_collection.insert_one = MagicMock(return_value=MagicMock(inserted_id=ObjectId()))
        
        # Execute
        with patch('asyncio.to_thread', side_effect=lambda f, *args, **kwargs: f(*args, **kwargs) if callable(f) else f):
            result = await service.log_unanswered(
                query="ค่าเข้าวัดภูมินทร์เท่าไหร่",
                score=0.32,
                session_id="test_session"
            )
        
        # Verify
        assert result is not None
        mock_collection.insert_one.assert_called_once()
        
        # Check the inserted document structure
        call_args = mock_collection.insert_one.call_args[0][0]
        assert call_args["query"] == "ค่าเข้าวัดภูมินทร์เท่าไหร่"
        assert call_args["max_score"] == 0.32
        assert call_args["count"] == 1
        assert call_args["status"] == "PENDING"

    @pytest.mark.asyncio
    async def test_log_unanswered_increments_existing_count(self, service, mock_mongo):
        """Test that repeated query increments count instead of creating new entry"""
        # Setup: Existing entry with count=3
        existing_doc = {
            "_id": ObjectId(),
            "query": "test query",
            "normalized_query": "test query",
            "count": 3,
            "status": "PENDING"
        }
        mock_collection = mock_mongo.get_collection.return_value
        mock_collection.find_one = MagicMock(return_value=existing_doc)
        mock_collection.update_one = MagicMock(return_value=MagicMock(modified_count=1))
        
        # Execute
        with patch('asyncio.to_thread', side_effect=lambda f, *args, **kwargs: f(*args, **kwargs) if callable(f) else f):
            result = await service.log_unanswered(
                query="test query",
                score=0.30,
                session_id="test_session"
            )
        
        # Verify
        assert result == str(existing_doc["_id"])
        mock_collection.update_one.assert_called_once()
        mock_collection.insert_one.assert_not_called()

    @pytest.mark.asyncio
    async def test_log_unanswered_normalizes_query(self, service, mock_mongo):
        """Test that query is normalized (lowercase, stripped)"""
        mock_collection = mock_mongo.get_collection.return_value
        mock_collection.find_one = MagicMock(return_value=None)
        mock_collection.insert_one = MagicMock(return_value=MagicMock(inserted_id=ObjectId()))
        
        with patch('asyncio.to_thread', side_effect=lambda f, *args, **kwargs: f(*args, **kwargs) if callable(f) else f):
            await service.log_unanswered(
                query="  TEST Query  ",
                score=0.30
            )
        
        # Check normalized query
        call_args = mock_collection.insert_one.call_args[0][0]
        assert call_args["normalized_query"] == "test query"

    # ========== get_pending_gaps Tests ==========
    
    @pytest.mark.asyncio
    async def test_get_pending_gaps_returns_pending_only(self, service, mock_mongo):
        """Test that get_pending_gaps returns only PENDING status by default"""
        mock_collection = mock_mongo.get_collection.return_value
        mock_cursor = MagicMock()
        mock_cursor.sort = MagicMock(return_value=mock_cursor)
        mock_cursor.limit = MagicMock(return_value=[
            {"_id": ObjectId(), "query": "test1", "status": "PENDING"},
            {"_id": ObjectId(), "query": "test2", "status": "PENDING"}
        ])
        mock_collection.find = MagicMock(return_value=mock_cursor)
        
        with patch('asyncio.to_thread', side_effect=lambda f, *args, **kwargs: f(*args, **kwargs) if callable(f) else list(f) if hasattr(f, '__iter__') else f):
            result = await service.get_pending_gaps(limit=10)
        
        # Verify filter was called with PENDING status
        mock_collection.find.assert_called_once()
        call_args = mock_collection.find.call_args[0][0]
        assert call_args.get("status") == "PENDING"

    # ========== resolve_gap Tests ==========
    
    @pytest.mark.asyncio
    async def test_resolve_gap_updates_status(self, service, mock_mongo, mock_qdrant):
        """Test that resolve_gap updates status to RESOLVED"""
        gap_id = str(ObjectId())
        gap_doc = {
            "_id": ObjectId(gap_id),
            "query": "test query",
            "status": "PENDING",
            "count": 5
        }
        
        mock_collection = mock_mongo.get_collection.return_value
        mock_collection.find_one = MagicMock(return_value=gap_doc)
        mock_collection.update_one = MagicMock(return_value=MagicMock(modified_count=1))
        
        with patch('asyncio.to_thread', side_effect=lambda f, *args, **kwargs: f(*args, **kwargs) if callable(f) else f):
            result = await service.resolve_gap(
                gap_id=gap_id,
                answer="คำตอบทดสอบที่มีความยาวเพียงพอ",
                admin_id="admin_test"
            )
        
        # Verify
        assert result["success"] is True
        mock_collection.update_one.assert_called()

    @pytest.mark.asyncio
    async def test_resolve_gap_fails_for_non_pending(self, service, mock_mongo):
        """Test that resolve_gap fails for already resolved gaps"""
        gap_id = str(ObjectId())
        gap_doc = {
            "_id": ObjectId(gap_id),
            "query": "test query",
            "status": "RESOLVED"  # Already resolved
        }
        
        mock_collection = mock_mongo.get_collection.return_value
        mock_collection.find_one = MagicMock(return_value=gap_doc)
        
        with patch('asyncio.to_thread', side_effect=lambda f, *args, **kwargs: f(*args, **kwargs) if callable(f) else f):
            result = await service.resolve_gap(
                gap_id=gap_id,
                answer="คำตอบทดสอบ"
            )
        
        # Verify failure
        assert result["success"] is False
        assert "already" in result["error"].lower()

    # ========== dismiss_gap Tests ==========
    
    @pytest.mark.asyncio
    async def test_dismiss_gap_updates_status(self, service, mock_mongo):
        """Test that dismiss_gap updates status to DISMISSED"""
        gap_id = str(ObjectId())
        
        mock_collection = mock_mongo.get_collection.return_value
        mock_collection.update_one = MagicMock(return_value=MagicMock(modified_count=1))
        
        with patch('asyncio.to_thread', side_effect=lambda f, *args, **kwargs: f(*args, **kwargs) if callable(f) else f):
            result = await service.dismiss_gap(
                gap_id=gap_id,
                reason="ไม่เกี่ยวกับน่าน"
            )
        
        # Verify
        assert result["success"] is True

    # ========== get_stats Tests ==========
    
    @pytest.mark.asyncio
    async def test_get_stats_returns_counts(self, service, mock_mongo):
        """Test that get_stats returns correct counts"""
        mock_collection = mock_mongo.get_collection.return_value
        mock_collection.count_documents = MagicMock(side_effect=[10, 5, 2, 1])  # pending, resolved, dismissed, today
        
        mock_cursor = MagicMock()
        mock_cursor.sort = MagicMock(return_value=mock_cursor)
        mock_cursor.limit = MagicMock(return_value=[])
        mock_collection.find = MagicMock(return_value=mock_cursor)
        
        with patch('asyncio.to_thread', side_effect=lambda f, *args, **kwargs: f(*args, **kwargs) if callable(f) else list(f) if hasattr(f, '__iter__') else f):
            result = await service.get_stats()
        
        # Verify
        assert result["pending_count"] == 10
        assert result["resolved_count"] == 5
        assert result["dismissed_count"] == 2


class TestKnowledgeGapIntegration:
    """Integration tests (require running services)"""
    
    @pytest.mark.integration
    @pytest.mark.skip(reason="Requires running MongoDB")
    async def test_full_workflow(self):
        """Test full workflow: log -> get -> resolve"""
        pass  # Placeholder for integration tests
