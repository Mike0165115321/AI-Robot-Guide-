import uvicorn
import os
import asyncio
import logging
from contextlib import asynccontextmanager
from fastapi import FastAPI, Request, HTTPException, Depends 
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware
from pathlib import Path
from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from core.ai_models.query_interpreter import QueryInterpreter
from core.ai_models.youtube_handler import YouTubeHandler
from core.ai_models.rag_orchestrator import RAGOrchestrator 
from core.ai_models.youtube_handler import youtube_handler_instance
from core.config import settings
from utils.file_cleaner import start_background_cleanup
from api.dependencies import get_rag_orchestrator 
from api.routers import admin_api, chat_api, avatar_api

logging.basicConfig(level=logging.INFO)
logging.getLogger("uvicorn").propagate = False
logging.getLogger("sentence_transformers").setLevel(logging.WARNING)


@asynccontextmanager
async def lifespan(app: FastAPI):
    logging.info("🚀 [Lifespan] Application Starting Up...")
    
    app.state.mongo_manager = MongoDBManager()
    app.state.qdrant_manager = QdrantManager()
    app.state.query_interpreter = QueryInterpreter()
    app.state.youtube_handler = YouTubeHandler()
    app.state.rag_orchestrator = RAGOrchestrator(
        mongo_manager=app.state.mongo_manager,
        qdrant_manager=app.state.qdrant_manager,
        query_interpreter=app.state.query_interpreter
    )
    
    try:
        await app.state.qdrant_manager.initialize()
    except Exception as e:
        logging.critical(f"❌ [Lifespan] CRITICAL: Failed to initialize Qdrant. {e}", exc_info=True)
        raise e

    app.state.cleanup_task = asyncio.create_task(start_background_cleanup())
    logging.info("✅ [Lifespan] Background cleanup task started.")
    
    logging.info("✅ [Lifespan] Startup complete. Ready to serve requests.")
    
    yield 
    logging.info("⏳ [Lifespan] Application Shutting Down...")
    
    await app.state.qdrant_manager.close()
    await app.state.query_interpreter.close()
    
    app.state.cleanup_task.cancel()
    logging.info("✅ [Lifespan] Background cleanup task stopped.")
    
    logging.info("✅ [Lifespan] Shutdown complete.")


app = FastAPI(
    title="AI Robot Guide จังหวัดน่าน API",
    description="API สำหรับจัดการข้อมูลและให้บริการสนทนาสำหรับ AI Guide",
    version="1.0.0",
    lifespan=lifespan  
)

BACKEND_ROOT = Path(__file__).resolve().parent.parent
STATIC_DIR = BACKEND_ROOT / "static"
STATIC_DIR.mkdir(parents=True, exist_ok=True) 

logging.info(f"✅ Serving static files from directory: {STATIC_DIR}")
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

origins = ["*"] 
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(admin_api.router, prefix="/api/admin/locations") 
app.include_router(chat_api.router, prefix="/api/chat")   
app.include_router(avatar_api.router, prefix="/api/avatar")


@app.get("/api/navigation_list", tags=["V-Maps"])
async def get_navigation_list(
    orchestrator: RAGOrchestrator = Depends(get_rag_orchestrator)
):
    try:
        location_list = await orchestrator.get_navigation_list()
        return location_list
    except Exception as e:
        logging.error(f"❌ [API-NavList] Error getting navigation list: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Could not retrieve navigation list.")
    
@app.get("/api/stream")
async def get_stream_url(video_url: str):
    if not video_url:
        raise HTTPException(status_code=400, detail="จำเป็นต้องมี 'video_url' parameter")
    try:
        stream_url = await youtube_handler_instance.get_audio_stream_url(video_url)
        
        if not stream_url:
            raise HTTPException(status_code=404, detail="ไม่พบสตรีมเสียงสำหรับวิดีโอนี้")
        return {"stream_url": stream_url}
    except HTTPException as http_exc:
        raise http_exc
    except Exception as e:
        logging.error(f"❌ [API-Stream] An unexpected error occurred: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="เกิดข้อผิดพลาดภายในเซิร์ฟเวอร์ขณะประมวลผลสตรีมเสียง")

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
FRONTEND_DIR = PROJECT_ROOT / "frontend"

logging.info(f"✅ Serving frontend from directory: {FRONTEND_DIR}")
if not FRONTEND_DIR.is_dir():
    logging.critical(f"❌ CRITICAL ERROR: Frontend directory not found at {FRONTEND_DIR}")

app.mount("/assets", StaticFiles(directory=FRONTEND_DIR / "assets"), name="assets")
templates = Jinja2Templates(directory=str(FRONTEND_DIR))

@app.get("/{full_path:path}", include_in_schema=False)
async def serve_frontend(request: Request, full_path: str):
    path_map = {
        "": "index.html",
        "chat": "chat.html",
        "admin": "admin.html",
        "robot_avatar": "robot_avatar.html",
        "travel_mode": "travel_mode.html"
    }
    file_to_serve = path_map.get(full_path, full_path)
    
    template_path = FRONTEND_DIR / file_to_serve
    if not template_path.is_file():
        return templates.TemplateResponse("index.html", {"request": request})

    return templates.TemplateResponse(file_to_serve, {"request": request})


if __name__ == "__main__":
    uvicorn.run(
        "main:app", 
        host=settings.API_HOST, 
        port=settings.API_PORT, 
        reload=True
    )