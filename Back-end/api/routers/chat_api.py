import logging
from fastapi import APIRouter, HTTPException, Depends, status, File, UploadFile, WebSocket, WebSocketDisconnect
import json
from ..schemas import ChatQuery, ChatResponse 
from core.ai_models.rag_orchestrator import RAGOrchestrator
from core.config import settings
from ..dependencies import get_rag_orchestrator, get_analytics_service
from core.services.analytics_service import AnalyticsService

from core.ai_models.speech_handler import speech_handler_instance


def construct_full_image_url(image_path: str | None) -> str | None:
    if not image_path: return None
    if image_path.startswith(('http://', 'https://')):
        return image_path
    if image_path.startswith('/'):
        return f"http://{settings.API_HOST}:{settings.API_PORT}{image_path}"
    return image_path

router = APIRouter(tags=["Text Chat"])

@router.post("/transcribe", response_model=ChatResponse)
async def handle_audio_chat(
    orchestrator: RAGOrchestrator = Depends(get_rag_orchestrator),
    file: UploadFile = File(...)
):
    try:
        logging.info(f"💬 [API-Audio] ได้รับไฟล์เสียง: {file.filename}")
        audio_bytes = await file.read()

        transcribed_text = await speech_handler_instance.transcribe_audio_bytes(audio_bytes)
        
        if not transcribed_text:
            logging.warning("[API-Audio] การถอดเสียงล้มเหลวหรือว่างเปล่า")
            return ChatResponse(answer="ขออภัยค่ะ น้องน่านไม่ได้ยินที่คุณพูดเลย ลองพูดอีกครั้งนะคะ")

        logging.info(f"👂 [API-Audio] ได้ยิน (ถอดเสียง): '{transcribed_text}'")
        
        result = await orchestrator.answer_query(transcribed_text, mode='text')
        
        if not result or "answer" not in result:
            raise HTTPException(status_code=500, detail="AI failed to generate a response.")

        result["image_url"] = construct_full_image_url(result.get("image_url"))
        if result.get("image_gallery"):
            raw_gallery = result.get("image_gallery", [])
            result["image_gallery"] = [construct_full_image_url(url) for url in raw_gallery if url]
        if result.get("sources"):
            for source in result["sources"]:
                raw_urls = source.get("image_urls", []) 
                source["image_urls"] = [construct_full_image_url(url) for url in raw_urls if url]
        
        result["transcribed_query"] = transcribed_text
        
        logging.info(f"✅ [API-Audio] กำลังส่งคำตอบกลับไปยังไคลเอนต์")
        return result
    
    except Exception as e:
        logging.error(f"❌ [API-Audio] เกิดข้อผิดพลาดที่ไม่คาดคิด: {e}", exc_info=True)
        return ChatResponse(answer="ขออภัยค่ะ เกิดข้อผิดพลาดร้ายแรงในการประมวลผลเสียงค่ะ")

@router.post("/", response_model=ChatResponse)
async def handle_text_chat(
    query: ChatQuery, 
    orchestrator: RAGOrchestrator = Depends(get_rag_orchestrator),
    analytics: AnalyticsService = Depends(get_analytics_service)
):
    try:
        query_data = query.query 
        session_id = query.session_id 
        
        result = None
        user_intent = None # To track for analytics

        if isinstance(query_data, dict) and (action := query_data.get("action")):
            # 🚀 [แก้ไข] เพิ่มการ log session_id
            logging.info(f"⚡️ [API-Text] ได้รับ EXPLICIT ACTION: '{action}' | Session: '{session_id}'")
            
            if action == "GET_DIRECTIONS":
                entity_slug = query_data.get("entity_slug")
                user_lat = query_data.get("user_lat")
                user_lon = query_data.get("user_lon")
                
                if not entity_slug or user_lat is None or user_lon is None:
                    raise HTTPException(status_code=400, detail="Missing data for GET_DIRECTIONS")
                
                result = await orchestrator.handle_get_directions(entity_slug, user_lat, user_lon)
            
            else:
                logging.warning(f"ได้รับ action ที่ไม่รู้จัก: {action}")
                result = {"answer": "ขออภัยค่ะ ไม่รู้จักคำสั่ง Action นี้ค่ะ", "action": None}

        elif isinstance(query_data, str):
            logging.info(f"💬 [API-Text] ได้รับ IMPLICIT query: '{query_data}' | Session: '{session_id}'")
            result = await orchestrator.answer_query(
                query=query_data, 
                mode='text', 
                session_id=session_id 
            )
        else:
            raise HTTPException(status_code=400, detail="Invalid query format.")
        
        if not result or "answer" not in result:
            raise HTTPException(status_code=500, detail="AI failed to generate a response.")
        
        result["image_url"] = construct_full_image_url(result.get("image_url"))

        if result.get("image_gallery"):
            raw_gallery = result.get("image_gallery", [])
            result["image_gallery"] = [construct_full_image_url(url) for url in raw_gallery if url]

        if result.get("sources"):
            for source in result["sources"]:
                raw_urls = source.get("image_urls", []) 
                source["image_urls"] = [construct_full_image_url(url) for url in raw_urls if url]
        logging.info(f"✅ [API-Text] กำลังส่งคำตอบกลับไปยังไคลเอนต์")
        
        # 📊 Async Log to Analytics
        user_query_str = query_data if isinstance(query_data, str) else str(query_data)
        topic = result.get("category") or result.get("topic")
        location_title = result.get("title") or result.get("location_title")
        
        await analytics.log_interaction(
            session_id=session_id,
            user_query=user_query_str,
            response=result.get("answer", ""),
            topic=topic,
            location_title=location_title
        )

        return result
    
    except Exception as e:
        logging.error(f"❌ [API-Text] เกิดข้อผิดพลาดที่ไม่คาดคิด: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="An internal error occurred.")

# 🆕 Endpoint สำหรับรับข้อมูล province จาก Toast Notification
from pydantic import BaseModel
from typing import Optional
from datetime import datetime, timezone

class WelcomeDataRequest(BaseModel):
    session_id: str
    user_province: Optional[str] = None
    user_origin: Optional[str] = "Thailand"

@router.post("/welcome-data")
async def receive_welcome_data(
    data: WelcomeDataRequest,
    analytics: AnalyticsService = Depends(get_analytics_service)
):
    """
    รับข้อมูลจังหวัด/ประเทศจาก Toast Notification และบันทึกลง analytics
    """
    try:
        logging.info(f"📊 [Welcome] ได้รับข้อมูลจังหวัด: {data.user_province} | {data.user_origin}")
        
        # Log to analytics
        await analytics.log_interaction(
            session_id=data.session_id,
            user_query="[Welcome Form Submission]",
            response="",
            topic=None,
            location_title=None,
            user_origin=data.user_origin,
            user_province=data.user_province
        )
        
        return {"status": "success", "message": "ขอบคุณสำหรับข้อมูลค่ะ!"}
        
    except Exception as e:
        logging.error(f"❌ [Welcome] ข้อผิดพลาดในการบันทึกข้อมูล: {e}")
        return {"status": "error", "message": str(e)}

# 🆕 Music Search Endpoint - สำหรับ in-place search
from core.ai_models.youtube_handler import youtube_handler_instance

class MusicSearchRequest(BaseModel):
    song_name: str

@router.post("/music-search")
async def search_music(request: MusicSearchRequest):
    """
    🎵 Search music on YouTube - returns results for in-place display
    """
    try:
        song_name = request.song_name.strip()
        if not song_name:
            return {"success": False, "error": "กรุณาระบุชื่อเพลง", "results": []}
        
        logging.info(f"🎵 [Music Search] คำค้นหา: '{song_name}'")
        results = await youtube_handler_instance.search_music(query=song_name)
        
        if not results:
            return {"success": False, "error": f"ไม่พบเพลง '{song_name}'", "results": []}
        
        return {"success": True, "query": song_name, "results": results}
        
    except Exception as e:
        logging.error(f"❌ [Music Search] ข้อผิดพลาด: {e}")
        return {"success": False, "error": str(e), "results": []}

class MusicStreamRequest(BaseModel):
    video_url: str

@router.post("/music/stream")
async def get_audio_stream(request: MusicStreamRequest):
    """
    🎧 Get audio stream URL for a YouTube video
    """
    try:
        video_url = request.video_url
        if not video_url:
            raise HTTPException(status_code=400, detail="Missing video_url")
            
        logging.info(f"🎧 [Music Stream] กำลังดึงสตรีมสำหรับ: {video_url}")
        
        # Reuse existing logic from youtube_handler
        stream_url = await youtube_handler_instance.get_audio_stream_url(video_url)
        
        if not stream_url:
            return {"error": "ไม่พบสตรีมเสียงสำหรับวิดีโอนี้", "stream_url": None}
            
        return {"stream_url": stream_url}
        
    except Exception as e:
        logging.error(f"❌ [Music Stream] ข้อผิดพลาด: {e}")
        return {"error": str(e), "stream_url": None}

# 🆕 Navigation Endpoint - สำหรับ in-place display
class NavigationRequest(BaseModel):
    slug: Optional[str] = None
    query: Optional[str] = None
    user_lat: Optional[float] = None
    user_lon: Optional[float] = None

@router.post("/navigation")
async def get_navigation(
    request: NavigationRequest,
    orchestrator: RAGOrchestrator = Depends(get_rag_orchestrator)
):
    """
    🗺️ Direct Navigation via HTTP for in-place updates.
    Passing a 'slug' works best. If not, 'query' acts as a fallback slug/title search.
    """
    try:
        target = request.slug or request.query
        if not target:
             return {"success": False, "error": "Missing slug or query"}

        logging.info(f"🏎️ [HTTP Nav] กำลังขอเส้นทางสำหรับ: '{target}'")
        
        # Directly call orchestrator logic (which calls NavigationService)
        # Note: handle_get_directions expects 'entity_slug' but it handles title fallback too
        result = await orchestrator.handle_get_directions(
            entity_slug=target,
            user_lat=request.user_lat, 
            user_lon=request.user_lon
        )
        
        # Determine success based on result content
        # NavigationService output format: { "answer": ..., "action": "SHOW_MAP_EMBED", "action_payload": ... }
        if result and result.get("action") == "SHOW_MAP_EMBED":
             return {
                 "success": True, 
                 "result": result 
             }
        else:
             return {
                 "success": False, 
                 "error": result.get("answer", "ไม่พบข้อมูลสถานที่ดังกล่าว"),
                 "raw_result": result
             }

    except Exception as e:
        logging.error(f"❌ [HTTP Nav] ข้อผิดพลาด: {e}")
        return {"success": False, "error": str(e)}


@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket, orchestrator: RAGOrchestrator = Depends(get_rag_orchestrator)):
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive()
            
            if "text" in data:
                try:
                    query_data = json.loads(data["text"])
                    query_text = query_data.get("query", "")
                    ai_mode = query_data.get("ai_mode", "fast")  # fast | detailed
                    # 🆕 รับ intent จาก Frontend - ไม่ต้องใช้ LLM วิเคราะห์
                    intent = query_data.get("intent", "GENERAL")  # GENERAL | MUSIC | NAVIGATION | FAQ
                    
                    # 🆕 รับ slug (ถ้ามี) สำหรับ Navigation / System Commands
                    slug = query_data.get("slug")
                    entity_query = query_data.get("entity_query") # manual query text if slug is missing
                    
                    logging.info(f"💬 [WS] ข้อความ: {query_text} | โหมด: {ai_mode} | เจตนา: {intent} | Slug: {slug}")
                    
                    result = await orchestrator.answer_query(
                        query_text, 
                        mode='text', 
                        ai_mode=ai_mode,
                        frontend_intent=intent,
                        slug=slug,
                        entity_query=entity_query
                    )
                    await websocket.send_json(result)
                except Exception as e:
                    logging.error(f"❌ [WS] ข้อผิดพลาดในการประมวลผลข้อความ: {e}")
                    await websocket.send_json({"answer": "เกิดข้อผิดพลาดในการประมวลผลค่ะ"})

            elif "bytes" in data:
                try:
                    audio_bytes = data["bytes"]
                    logging.info(f"🎤 [WS] ได้รับข้อมูลเสียง: {len(audio_bytes)} bytes")
                    
                    transcribed_text = await speech_handler_instance.transcribe_audio_bytes(audio_bytes)
                    if transcribed_text:
                        logging.info(f"👂 [WS] ถอดเสียง: {transcribed_text}")
                        result = await orchestrator.answer_query(transcribed_text, mode='text')
                        result["transcribed_query"] = transcribed_text
                        await websocket.send_json(result)
                    else:
                        await websocket.send_json({"answer": "ขออภัยค่ะ ไม่ได้ยินเสียงเลย"})
                except Exception as e:
                    logging.error(f"❌ [WS] ข้อผิดพลาดในการประมวลผลเสียง: {e}")
                    await websocket.send_json({"answer": "เกิดข้อผิดพลาดในการประมวลผลเสียงค่ะ"})

    except WebSocketDisconnect:
        logging.info("🔌 [WS] ไคลเอนต์ตัดการเชื่อมต่อ")
    except RuntimeError as e:
        if "Cannot call \"receive\" once a disconnect message has been received" in str(e):
            logging.info("🔌 [WS] ไคลเอนต์ตัดการเชื่อมต่อ (จัดการ RuntimeError)")
        else:
            logging.error(f"❌ [WS] ข้อผิดพลาด Runtime: {e}")
    except Exception as e:
        logging.error(f"❌ [WS] ข้อผิดพลาดที่ไม่คาดคิด: {e}")
