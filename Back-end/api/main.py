import uvicorn
import os
import asyncio
import logging
from typing import Optional 
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
from api.routers import admin_api, chat_api, avatar_api, import_api, sheets_api, analytics_api, line_webhook, alert_api

logging.basicConfig(level=logging.INFO)
logging.getLogger("uvicorn").propagate = False
logging.getLogger("sentence_transformers").setLevel(logging.WARNING)


@asynccontextmanager
async def lifespan(app: FastAPI):
    logging.info("🚀 [Lifespan] กำลังเริ่มต้นแอปพลิเคชัน...")
    
    app.state.mongo_manager = MongoDBManager()
    app.state.qdrant_manager = QdrantManager()
    app.state.query_interpreter = QueryInterpreter()
    app.state.youtube_handler = YouTubeHandler()
    app.state.rag_orchestrator = RAGOrchestrator(
        mongo_manager=app.state.mongo_manager,
        qdrant_manager=app.state.qdrant_manager,
        query_interpreter=app.state.query_interpreter
    )
    
    from core.services.analytics_service import AnalyticsService
    app.state.analytics_service = AnalyticsService(app.state.mongo_manager)
    
    try:
        await app.state.qdrant_manager.initialize()
    except Exception as e:
        logging.critical(f"❌ [Lifespan] วิกฤต: ไม่สามารถเริ่มต้น Qdrant ได้ {e}", exc_info=True)
        # raise e  <-- Commented out to allow server to start without Qdrant

    app.state.cleanup_task = asyncio.create_task(start_background_cleanup())
    logging.info("✅ [Lifespan] งานทำความสะอาดเบื้องหลังเริ่มต้นแล้ว")
    
    # เริ่ม News Scheduler สำหรับ Smart News Monitor
    from core.services.news_scheduler import news_scheduler
    from core.services.alert_manager import alert_manager
    news_scheduler.set_alert_callback(alert_manager.broadcast_alert)
    news_scheduler.start()
    logging.info("✅ [Lifespan] News Scheduler เริ่มทำงาน")
    
    logging.info("✅ [Lifespan] เริ่มต้นเสร็จสมบูรณ์ พร้อมให้บริการ")
    
    yield 
    logging.info("⏳ [Lifespan] กำลังปิดแอปพลิเคชัน...")
    
    await app.state.qdrant_manager.close()
    await app.state.query_interpreter.close()
    
    app.state.cleanup_task.cancel()
    logging.info("✅ [Lifespan] งานทำความสะอาดเบื้องหลังหยุดแล้ว")
    
    # หยุด News Scheduler
    from core.services.news_scheduler import news_scheduler
    news_scheduler.stop()
    logging.info("✅ [Lifespan] News Scheduler หยุดทำงาน")
    
    logging.info("✅ [Lifespan] ปิดการทำงานสมบูรณ์")


app = FastAPI(
    title="AI Robot Guide จังหวัดน่าน API",
    description="API สำหรับจัดการข้อมูลและให้บริการสนทนาสำหรับ AI Guide",
    version="1.0.0",
    lifespan=lifespan  
)

BACKEND_ROOT = Path(__file__).resolve().parent.parent
STATIC_DIR = BACKEND_ROOT / "static"
STATIC_DIR.mkdir(parents=True, exist_ok=True) 

logging.info(f"✅ ให้บริการไฟล์ static จาก: {STATIC_DIR}")
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

# CORS Configuration - กำหนด origins ที่อนุญาตอย่างชัดเจนเพื่อความปลอดภัย
origins = [
    "http://localhost:9090",
    "http://127.0.0.1:9090",
    "http://localhost:3000",  # Next.js dev server
    "http://127.0.0.1:3000",  # Next.js dev server
    f"http://{settings.API_HOST}:{settings.API_PORT}",
    # เพิ่ม production domain ที่นี่ เช่น:
    # "https://your-production-domain.com",
] 
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(admin_api.router, prefix="/api/admin") 
app.include_router(chat_api.router, prefix="/api/chat")   
app.include_router(avatar_api.router, prefix="/api/avatar")
app.include_router(import_api.router, prefix="/api/admin/import")  # Smart ETL Import
app.include_router(sheets_api.router, prefix="/api/admin/sheets")  # Google Sheets Sync
app.include_router(analytics_api.router, prefix="/api/analytics")  # Feedback & Stats
app.include_router(line_webhook.router, prefix="/api/v1/line")     # LINE Webhook
app.include_router(alert_api.router, prefix="/api")                 # Smart News Alerts


@app.get("/health", tags=["Health"])
async def health_check(request: Request):
    """Health check endpoint สำหรับ monitoring และ deployment"""
    mongo_status = "unknown"
    qdrant_status = "unknown"
    
    try:
        # Check MongoDB connection
        if hasattr(request.app.state, 'mongo_manager') and request.app.state.mongo_manager:
            request.app.state.mongo_manager.client.admin.command('ping')
            mongo_status = "healthy"
    except Exception:
        mongo_status = "unhealthy"
    
    try:
        # Check Qdrant connection
        if hasattr(request.app.state, 'qdrant_manager') and request.app.state.qdrant_manager:
            await request.app.state.qdrant_manager.client.get_collections()
            qdrant_status = "healthy"
    except Exception:
        qdrant_status = "unhealthy"
    
    overall_status = "healthy" if mongo_status == "healthy" and qdrant_status == "healthy" else "degraded"
    
    return {
        "status": overall_status,
        "version": "1.0.0",
        "services": {
            "mongodb": mongo_status,
            "qdrant": qdrant_status
        }
    }


@app.get("/api/navigation_list", tags=["V-Maps"])
async def get_navigation_list(
    lat: Optional[float] = None, 
    lon: Optional[float] = None, 
    orchestrator: RAGOrchestrator = Depends(get_rag_orchestrator)
):
    try:
        location_list = await orchestrator.get_navigation_list(user_lat=lat, user_lon=lon)
        from fastapi.responses import JSONResponse
        return JSONResponse(content=location_list, media_type="application/json; charset=utf-8")
    except Exception as e:
        logging.error(f"❌ [API-NavList] เกิดข้อผิดพลาดในการดึงรายการนำทาง: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="ไม่สามารถดึงข้อมูลรายการนำทางได้")
    
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
        print(f"เกิดข้อผิดพลาดในการตรวจสอบสถานะ: {e}")
        return {"status": "error", "message": str(e)}









FRONTEND_DIR = settings.FRONTEND_DIR

logging.info(f"✅ ให้บริการ frontend จาก: {FRONTEND_DIR}")
if not FRONTEND_DIR.is_dir():
    logging.critical(f"❌ ข้อผิดพลาดร้ายแรง: ไม่พบโฟลเดอร์ frontend ที่ {FRONTEND_DIR}")

app.mount("/assets", StaticFiles(directory=FRONTEND_DIR / "assets"), name="assets")
templates = Jinja2Templates(directory=str(FRONTEND_DIR))



@app.get("/{full_path:path}", include_in_schema=False)
async def serve_frontend(request: Request, full_path: str):
    path_map = {
        "": "index.html",
        "chat": "chat.html",
        "admin": "admin.html",
        "import": "import.html",
        "robot_avatar": "robot_avatar.html",
        "travel_mode": "travel_mode.html",
        "alerts": "alerts.html"
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

# Trigger reload (Fix applied)