"""
🧪 Pytest Fixtures
Shared fixtures for all tests in the project
"""

import pytest
import asyncio
from unittest.mock import MagicMock, AsyncMock
from typing import Generator, AsyncGenerator
import sys
from pathlib import Path

# Add Back-end to path for imports
BACKEND_DIR = Path(__file__).parent.parent / "Back-end"
sys.path.insert(0, str(BACKEND_DIR))


# ========== Event Loop ==========

@pytest.fixture(scope="session")
def event_loop():
    """Create an instance of the default event loop for the test session."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


# ========== Mock Database Managers ==========

@pytest.fixture
def mock_mongo():
    """
    Mock MongoDB Manager
    
    Usage:
        def test_something(mock_mongo):
            mock_mongo.get_collection.return_value.find_one.return_value = {...}
    """
    mock = MagicMock()
    
    # Mock collections
    mock_collection = MagicMock()
    mock_collection.find_one = MagicMock(return_value=None)
    mock_collection.find = MagicMock(return_value=[])
    mock_collection.insert_one = MagicMock(return_value=MagicMock(inserted_id="mock_id"))
    mock_collection.update_one = MagicMock(return_value=MagicMock(modified_count=1))
    mock_collection.count_documents = MagicMock(return_value=0)
    
    mock.get_collection = MagicMock(return_value=mock_collection)
    mock.client = MagicMock()
    
    return mock


@pytest.fixture
def mock_qdrant():
    """
    Mock Qdrant Manager
    
    Usage:
        def test_something(mock_qdrant):
            mock_qdrant.search_similar.return_value = [...]
    """
    mock = MagicMock()
    mock.search_similar = AsyncMock(return_value=[])
    mock.upsert_document = AsyncMock(return_value="mock_qdrant_id")
    mock.initialize = AsyncMock()
    mock.close = AsyncMock()
    mock.client = MagicMock()
    mock.client.get_collections = AsyncMock(return_value=[])
    
    return mock


# ========== Mock Services ==========

@pytest.fixture
def mock_knowledge_gap_service(mock_mongo, mock_qdrant):
    """
    Mock Knowledge Gap Service
    """
    from core.services.knowledge_gap_service import KnowledgeGapService
    
    service = KnowledgeGapService(mock_mongo, mock_qdrant)
    return service


@pytest.fixture
def mock_analytics_service(mock_mongo):
    """
    Mock Analytics Service
    """
    mock = MagicMock()
    mock.log_interaction = AsyncMock()
    mock.get_dashboard_summary = AsyncMock(return_value={})
    mock.log_feedback = AsyncMock()
    
    return mock


# ========== Test Client ==========

@pytest.fixture
def test_client():
    """
    FastAPI TestClient for integration tests
    
    Note: Requires running MongoDB and Qdrant for full integration tests.
    For unit tests, use mock fixtures instead.
    
    Usage:
        def test_api(test_client):
            response = test_client.get("/health")
            assert response.status_code == 200
    """
    try:
        from fastapi.testclient import TestClient
        from api.main import app
        
        with TestClient(app) as client:
            yield client
    except ImportError:
        pytest.skip("FastAPI or test dependencies not installed")


# ========== Sample Data Fixtures ==========

@pytest.fixture
def sample_gap_document():
    """Sample knowledge gap document for testing"""
    from datetime import datetime, timezone
    
    return {
        "_id": "test_gap_id_123",
        "query": "ค่าเข้าวัดภูมินทร์เท่าไหร่",
        "normalized_query": "ค่าเข้าวัดภูมินทร์เท่าไหร่",
        "max_score": 0.32,
        "count": 5,
        "first_asked": datetime.now(timezone.utc),
        "last_asked": datetime.now(timezone.utc),
        "status": "PENDING",
        "sessions": ["session_1", "session_2"],
        "context": None,
        "resolved_answer": None,
        "resolved_by": None,
        "resolved_at": None,
        "dismiss_reason": None
    }


@pytest.fixture
def sample_location_document():
    """Sample location document for testing"""
    return {
        "_id": "test_location_id",
        "title": "วัดภูมินทร์",
        "slug": "wat-phumin",
        "summary": "วัดเก่าแก่ที่มีภาพจิตรกรรมฝาผนังสวยงาม",
        "category": "วัด",
        "topic": "สถานที่ท่องเที่ยว",
        "doc_type": "Location",
        "location_data": {
            "lat": 18.7751,
            "lon": 100.7794
        },
        "metadata": {
            "image_prefix": "wat_phumin_"
        }
    }


# ========== Async Helpers ==========

@pytest.fixture
def run_async():
    """
    Helper fixture to run async functions in sync tests
    
    Usage:
        def test_async_func(run_async):
            result = run_async(some_async_function())
            assert result == expected
    """
    def _run_async(coro):
        loop = asyncio.get_event_loop()
        return loop.run_until_complete(coro)
    
    return _run_async
