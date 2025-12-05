from fastapi import Request, HTTPException, status
from starlette.requests import HTTPConnection
from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from core.ai_models.rag_orchestrator import RAGOrchestrator
from core.ai_models.youtube_handler import YouTubeHandler 

def get_mongo_manager(request: HTTPConnection) -> MongoDBManager:
    manager = getattr(request.app.state, "mongo_manager", None)
    if manager is None:
        raise HTTPException(status_code=503, detail="MongoDB service not available.")
    return manager

def get_qdrant_manager(request: HTTPConnection) -> QdrantManager:
    manager = getattr(request.app.state, "qdrant_manager", None)
    if manager is None:
        raise HTTPException(status_code=503, detail="Qdrant service not available.")
    return manager

def get_rag_orchestrator(request: HTTPConnection) -> RAGOrchestrator:
    orchestrator = getattr(request.app.state, "rag_orchestrator", None)
    if orchestrator is None:
        raise HTTPException(status_code=503, detail="RAG service not available.")
    return orchestrator

def get_youtube_handler(request: HTTPConnection) -> YouTubeHandler:
    handler = getattr(request.app.state, "youtube_handler", None)
    if handler is None:
        raise HTTPException(status_code=503, detail="YouTube service not available.")
    return handler

from core.services.analytics_service import AnalyticsService

def get_analytics_service(request: HTTPConnection) -> AnalyticsService:
    service = getattr(request.app.state, "analytics_service", None)
    if service is None:
        # Fallback: Create on fly if missed in startup (though should be in state)
        mongo = get_mongo_manager(request)
        return AnalyticsService(mongo)
    return service