import logging
from fastapi import APIRouter, HTTPException, Depends, status, File, UploadFile, WebSocket, WebSocketDisconnect
import json
from ..schemas import ChatQuery, ChatResponse 
from core.ai_models.rag_orchestrator import RAGOrchestrator
from core.config import settings
from ..dependencies import get_rag_orchestrator

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
        logging.info(f"💬 [API-Audio] Received audio file: {file.filename}")
        audio_bytes = await file.read()

        transcribed_text = await speech_handler_instance.transcribe_audio_bytes(audio_bytes)
        
        if not transcribed_text:
            logging.warning("[API-Audio] Transcription failed or was empty.")
            return ChatResponse(answer="ขออภัยค่ะ น้องน่านไม่ได้ยินที่คุณพูดเลย ลองพูดอีกครั้งนะคะ")

        logging.info(f"👂 [API-Audio] Heard (Transcribed): '{transcribed_text}'")
        
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
        
        logging.info(f"✅ [API-Audio] Sending response back to client.")
        return result
    
    except Exception as e:
        logging.error(f"❌ [API-Audio] An unexpected error occurred: {e}", exc_info=True)
        return ChatResponse(answer="ขออภัยค่ะ เกิดข้อผิดพลาดร้ายแรงในการประมวลผลเสียงค่ะ")

@router.post("/", response_model=ChatResponse)
async def handle_text_chat(
    query: ChatQuery, 
    orchestrator: RAGOrchestrator = Depends(get_rag_orchestrator)
):
    try:
        query_data = query.query 
        session_id = query.session_id 
        
        result = None

        if isinstance(query_data, dict) and (action := query_data.get("action")):
            # 🚀 [แก้ไข] เพิ่มการ log session_id
            logging.info(f"⚡️ [API-Text] Received EXPLICIT ACTION: '{action}' | Session: '{session_id}'")
            
            if action == "GET_DIRECTIONS":
                entity_slug = query_data.get("entity_slug")
                user_lat = query_data.get("user_lat")
                user_lon = query_data.get("user_lon")
                
                if not entity_slug or user_lat is None or user_lon is None:
                    raise HTTPException(status_code=400, detail="Missing data for GET_DIRECTIONS")
                
                result = await orchestrator.handle_get_directions(entity_slug, user_lat, user_lon)
            
            else:
                logging.warning(f"Received unknown explicit action: {action}")
                result = {"answer": "ขออภัยค่ะ ไม่รู้จักคำสั่ง Action นี้ค่ะ", "action": None}

        elif isinstance(query_data, str):
            logging.info(f"💬 [API-Text] Received IMPLICIT query: '{query_data}' | Session: '{session_id}'")
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
        logging.info(f"✅ [API-Text] Sending response back to client.")
        return result
    
    except Exception as e:
        logging.error(f"❌ [API-Text] An unexpected error occurred: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="An internal error occurred.")
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
                    logging.info(f"💬 [WS] Received text: {query_text}")
                    
                    result = await orchestrator.answer_query(query_text, mode='text')
                    await websocket.send_json(result)
                except Exception as e:
                    logging.error(f"❌ [WS] Error processing text: {e}")
                    await websocket.send_json({"answer": "เกิดข้อผิดพลาดในการประมวลผลค่ะ"})

            elif "bytes" in data:
                try:
                    audio_bytes = data["bytes"]
                    logging.info(f"🎤 [WS] Received audio bytes: {len(audio_bytes)} bytes")
                    
                    transcribed_text = await speech_handler_instance.transcribe_audio_bytes(audio_bytes)
                    if transcribed_text:
                        logging.info(f"👂 [WS] Transcribed: {transcribed_text}")
                        result = await orchestrator.answer_query(transcribed_text, mode='text')
                        result["transcribed_query"] = transcribed_text
                        await websocket.send_json(result)
                    else:
                        await websocket.send_json({"answer": "ขออภัยค่ะ ไม่ได้ยินเสียงเลย"})
                except Exception as e:
                    logging.error(f"❌ [WS] Error processing audio: {e}")
                    await websocket.send_json({"answer": "เกิดข้อผิดพลาดในการประมวลผลเสียงค่ะ"})

    except WebSocketDisconnect:
        logging.info("🔌 [WS] Client disconnected")
    except RuntimeError as e:
        if "Cannot call \"receive\" once a disconnect message has been received" in str(e):
            logging.info("🔌 [WS] Client disconnected (RuntimeError handled)")
        else:
            logging.error(f"❌ [WS] Runtime error: {e}")
    except Exception as e:
        logging.error(f"❌ [WS] Unexpected error: {e}")
