from fastapi import Request, HTTPException, status
from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from core.ai_models.rag_orchestrator import RAGOrchestrator
from core.ai_models.youtube_handler import YouTubeHandler 

def get_mongo_manager(request: Request) -> MongoDBManager:
    manager = getattr(request.app.state, "mongo_manager", None)
    if manager is None:
        raise HTTPException(status_code=503, detail="MongoDB service not available.")
    return manager

def get_qdrant_manager(request: Request) -> QdrantManager:
    manager = getattr(request.app.state, "qdrant_manager", None)
    if manager is None:
        raise HTTPException(status_code=503, detail="Qdrant service not available.")
    return manager

def get_rag_orchestrator(request: Request) -> RAGOrchestrator:
    orchestrator = getattr(request.app.state, "rag_orchestrator", None)
    if orchestrator is None:
        raise HTTPException(status_code=503, detail="RAG service not available.")
    return orchestrator
def get_youtube_handler(request: Request) -> YouTubeHandler:
    handler = getattr(request.app.state, "youtube_handler", None)
    if handler is None:
        raise HTTPException(status_code=503, detail="YouTube service not available.")
    return handler