import logging
from fastapi import APIRouter, HTTPException, Depends, status, File, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
import os
import glob
import json
from ..schemas import ChatQuery, ChatResponse 
from core.ai_models.rag_orchestrator import RAGOrchestrator
from core.config import settings
from ..dependencies import get_rag_orchestrator, get_analytics_service
from core.services.analytics_service import AnalyticsService

from core.ai_models.speech_handler import speech_handler_instance


def construct_full_image_url(image_path: str | None) -> str | None:
    if not image_path: return None
    
    # 🧹 Sanitize: If DB has hardcoded localhost/127.0.0.1 (even with wrong port like 9090), strip it!
    if 'static/images' in image_path:
        # Extract just the filename or path after static/images
        filename = image_path.split('static/images/')[-1]
        return f"/static/images/{filename}"

    if image_path.startswith(('http://', 'https://')):
        return image_path
    
    if image_path.startswith('/'):
        return image_path
    
    # Default to static/images if just a filename is provided
    # 🔍 Check if file exists, if not try to find best match
    
    # 1. Check exact match
    full_path = settings.STATIC_DIR / "images" / image_path
    if full_path.exists():
         return f"/static/images/{image_path}" # Rel Path
         
    # 2. Try extensions
    stem = full_path.stem
    for ext in ['.jpg', '.png', '.jpeg', '.webp']:
        p = settings.STATIC_DIR / "images" / f"{stem}{ext}"
        if p.exists():
            return f"/static/images/{p.name}" # Rel Path

    # 3. Fuzzy search (Smart Match)
    # Use glob to find files containing the stem name
    # e.g. 'aom-dao' -> matches 'aom-dao-restaurant-01.jpg'
    pattern = str(settings.STATIC_DIR / "images" / f"*{stem}*")
    matches = glob.glob(pattern)
    if matches:
         # Sort by length to find the most specific or shortest match? 
         # Usually the first match is fine, or we can pick the shortest valid one.
         best_match = min(matches, key=len) 
         return f"/static/images/{os.path.basename(best_match)}" # Rel Path
    
    # Return original relative path as fallback
    return f"/static/images/{image_path}"

def sanitize_response_images(result: dict) -> dict:
    """
    Helper function to sanitize all image URLs in a ChatResponse dict.
    Applies construct_full_image_url to:
    - image_url
    - image_gallery
    - sources (image_urls key)
    """
    if not result: return result
    
    # 0. Sanitize content in Markdown Answer (The missing piece!)
    if result.get("answer"):
        import re
        # Regex to find any http(s)://.../static/images/... and replace with /static/images/...
        # This catches 127.0.0.1:9090, localhost:8014, or any other hardcoded host
        result["answer"] = re.sub(
            r'http[s]?://[^/]+/static/images/', 
            '/static/images/', 
            result["answer"]
        )

    # 1. Main Image
    if result.get("image_url"):
        result["image_url"] = construct_full_image_url(result["image_url"])

    # 2. Image Gallery
    if result.get("image_gallery"):
        raw_gallery = result.get("image_gallery", [])
        result["image_gallery"] = [construct_full_image_url(url) for url in raw_gallery if url]

    # 3. Sources
    if result.get("sources"):
        for source in result["sources"]:
            raw_urls = source.get("image_urls", []) 
            source["image_urls"] = [construct_full_image_url(url) for url in raw_urls if url]
            
    return result

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

        if not result or "answer" not in result:
            raise HTTPException(status_code=500, detail="AI failed to generate a response.")

        # ✅ Sanitize Images
        result = sanitize_response_images(result)
        
        result["transcribed_query"] = transcribed_text
        
        # 🆕 Add avatar_mood based on action/content
        result["avatar_mood"] = _determine_avatar_mood(result)
        
        logging.info(f"✅ [API-Audio] กำลังส่งคำตอบกลับไปยังไคลเอนต์ (Mood: {result.get('avatar_mood')})")
        return result
    
    except Exception as e:
        logging.error(f"❌ [API-Audio] เกิดข้อผิดพลาดที่ไม่คาดคิด: {e}", exc_info=True)
        return ChatResponse(answer="ขออภัยค่ะ เกิดข้อผิดพลาดร้ายแรงในการประมวลผลเสียงค่ะ")

@router.post("/stt")
async def speech_to_text_only(
    file: UploadFile = File(...)
):
    """
    🎤 Pure STT Endpoint: Returns transcribed text only.
    """
    try:
        logging.info(f"🎤 [API-STT] Received audio for transcription: {file.filename}")
        audio_bytes = await file.read()
        transcribed_text = await speech_handler_instance.transcribe_audio_bytes(audio_bytes)
        
        if not transcribed_text:
            return {"text": ""}
            
        logging.info(f"📝 [API-STT] Transcribed: '{transcribed_text}'")
        return {"text": transcribed_text}
        
    except Exception as e:
        logging.error(f"❌ [API-STT] Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/", response_model=ChatResponse)
async def handle_text_chat(
    query: ChatQuery, 
    orchestrator: RAGOrchestrator = Depends(get_rag_orchestrator),
    analytics: AnalyticsService = Depends(get_analytics_service)
):
    try:
        query_data = query.query 
        session_id = query.session_id 
        ai_mode = query.ai_mode or "fast"  # 🆕 Read ai_mode from request
        
        result = None
        user_intent = None # To track for analytics

        if isinstance(query_data, dict) and (action := query_data.get("action")):
            # 🚀 [แก้ไข] เพิ่มการ log session_id
            logging.info(f"⚡️ [API-Text] ได้รับ EXPLICIT ACTION: '{action}' | Session: '{session_id}' | Mode: {ai_mode}")
            
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
            logging.info(f"💬 [API-Text] ได้รับ IMPLICIT query: '{query_data}' | Session: '{session_id}' | Mode: {ai_mode}")
            result = await orchestrator.answer_query(
                query=query_data, 
                mode='text', 
                session_id=session_id,
                ai_mode=ai_mode  # 🆕 Pass ai_mode to orchestrator! 
            )
        else:
            raise HTTPException(status_code=400, detail="Invalid query format.")
        
        if not result or "answer" not in result:
            raise HTTPException(status_code=500, detail="AI failed to generate a response.")
        
        # ✅ Sanitize Images (Fix connection refused 9090 issues)
        result = sanitize_response_images(result)

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

        # 🆕 Add avatar_mood for REST API responses too
        result["avatar_mood"] = _determine_avatar_mood(result)

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

# 🆕 TTS Endpoint
class TTSRequest(BaseModel):
    text: str
    language: str = "th"

@router.get("/languages")
async def get_supported_languages():
    """
    Get list of supported languages and their regex patterns.
    Used by Frontend to build dynamic language detection.
    """
    from core.services.language_detector import language_detector
    return language_detector.get_active_languages_config()

@router.post("/tts")
async def text_to_speech(request: TTSRequest):
    """
    🗣️ Generate TTS audio stream from text
    """
    try:
        if not request.text:
             raise HTTPException(status_code=400, detail="Text is required")
             
        logging.info(f"🗣️ [API-TTS] Requesting TTS for: {request.text[:50]}...")
        
        return StreamingResponse(
            speech_handler_instance.synthesize_speech_stream(request.text, language_hint=request.language),
            media_type="audio/mpeg"
        )
    except Exception as e:
        logging.error(f"❌ [API-TTS] Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket, orchestrator: RAGOrchestrator = Depends(get_rag_orchestrator)):
    await websocket.accept()
    
    # 🆕 State tracking per connection (ย้ายมาจาก avatar_api.py)
    current_ai_mode = 'fast'
    
    try:
        while True:
            data = await websocket.receive()
            
            if "text" in data:
                try:
                    query_data = json.loads(data["text"])
                    query_text = query_data.get("query", "")
                    
                    # 🔄 Update mode if provided (ย้ายมาจาก avatar_api.py)
                    if "ai_mode" in query_data:
                        current_ai_mode = query_data["ai_mode"]
                    
                    # 🆕 Handle SET_MODE action (ย้ายมาจาก avatar_api.py)
                    if query_data.get("action") == "SET_MODE":
                        logging.info(f"🔄 [WS] อัปเดตโหมดเป็น: {current_ai_mode}")
                        await websocket.send_json({"status": "ok", "ai_mode": current_ai_mode})
                        continue
                    
                    # 🆕 รับ intent จาก Frontend - ไม่ต้องใช้ LLM วิเคราะห์
                    intent = query_data.get("intent", "GENERAL")  # GENERAL | MUSIC | NAVIGATION | FAQ
                    
                    # 🆕 รับ slug (ถ้ามี) สำหรับ Navigation / System Commands
                    slug = query_data.get("slug")
                    entity_query = query_data.get("entity_query") # manual query text if slug is missing
                    language_hint = query_data.get("language", "th") # 🆕 Get language hint from query_data
                    
                    # ⛔ STRICT BLOCK: Empty Query (Prevent RAG waste)
                    if not query_text.strip() and not slug and not intent == "NAVIGATION":
                         # If it's a control message or empty, don't trigger RAG
                         if query_data.get("type") in ["ping", "pong", "setMood", "changeSkin", "resumeIdle", "action"]:
                             continue
                         
                         logging.warning("⚠️ [WS] Empty query received. Skipping RAG.")
                         # Optional: send ack?
                         continue

                    logging.info(f"💬 [WS] ข้อความ: {query_text} | โหมด: {current_ai_mode} | เจตนา: {intent} | Slug: {slug} | Lang: {language_hint}")
                    
                    result = await orchestrator.answer_query(
                        query_text, 
                        mode='text', 
                        ai_mode=current_ai_mode,
                        frontend_intent=intent,
                        slug=slug,
                        entity_query=entity_query,
                        language=language_hint # 🆕 Pass language hint
                    )
                    # ✅ Sanitize Images for WS too!
                    result = sanitize_response_images(result)
                    
                    # 🆕 Add avatar_mood based on action/content (ย้ายมาจาก avatar_api.py)
                    result["avatar_mood"] = _determine_avatar_mood(result)
                    
                    await websocket.send_json(result)
                except Exception as e:
                    logging.error(f"❌ [WS] ข้อผิดพลาดในการประมวลผลข้อความ: {e}")
                    await websocket.send_json({"answer": "เกิดข้อผิดพลาดในการประมวลผลค่ะ", "avatar_mood": "confused"})

            elif "bytes" in data:
                try:
                    audio_bytes = data["bytes"]
                    logging.info(f"🎤 [WS] ได้รับข้อมูลเสียง: {len(audio_bytes)} bytes")
                    
                    transcribed_text = await speech_handler_instance.transcribe_audio_bytes(audio_bytes)
                    if transcribed_text:
                        logging.info(f"👂 [WS] ถอดเสียง: {transcribed_text}")
                        result = await orchestrator.answer_query(transcribed_text, mode='text', ai_mode=current_ai_mode)
                        # ✅ Sanitize Images for Audio/WS
                        result = sanitize_response_images(result)
                        
                        result["transcribed_query"] = transcribed_text
                        # 🆕 Add avatar_mood for audio responses too
                        result["avatar_mood"] = _determine_avatar_mood(result)
                        
                        await websocket.send_json(result)
                    else:
                        await websocket.send_json({"answer": "ขออภัยค่ะ ไม่ได้ยินเสียงเลย", "avatar_mood": "confused"})
                except Exception as e:
                    logging.error(f"❌ [WS] ข้อผิดพลาดในการประมวลผลเสียง: {e}")
                    await websocket.send_json({"answer": "เกิดข้อผิดพลาดในการประมวลผลเสียงค่ะ", "avatar_mood": "confused"})

    except WebSocketDisconnect:
        logging.info("🔌 [WS] ไคลเอนต์ตัดการเชื่อมต่อ")
    except RuntimeError as e:
        if "Cannot call \"receive\" once a disconnect message has been received" in str(e):
            logging.info("🔌 [WS] ไคลเอนต์ตัดการเชื่อมต่อ (จัดการ RuntimeError)")
        else:
            logging.error(f"❌ [WS] ข้อผิดพลาด Runtime: {e}")
    except Exception as e:
        logging.error(f"❌ [WS] ข้อผิดพลาดที่ไม่คาดคิด: {e}")


def _determine_avatar_mood(result: dict) -> str:
    """
    🎭 กำหนด mood ของ Avatar ตามประเภทคำตอบ (ย้ายมาจาก avatar_api.py)
    - MUSIC action → listening (ใส่หูฟัง)
    - มีรูปภาพ → happy
    - ปกติ → talking
    """
    action = result.get("action", "")
    
    # เพลง = กำลังฟัง
    if action and "MUSIC" in action:
        return "listening"
    
    # มีรูปภาพหรือ sources = มีความสุข
    if result.get("image_url") or result.get("image_gallery") or result.get("sources"):
        return "happy"
    
    # ปกติ = กำลังพูด
    return "talking"

