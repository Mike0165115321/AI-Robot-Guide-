# /api/routers/avatar_api.py (FINAL FIX - Corrected Payload Keys)

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from starlette.websockets import WebSocketState, WebSocketDisconnect as StarletteWebSocketDisconnect
import json
import random
import logging
import asyncio
from typing import Optional, List, Dict, Any 
from core.ai_models.rag_orchestrator import RAGOrchestrator
from core.ai_models.speech_handler import speech_handler_instance
from core.config import settings

def sanitize_for_json(obj):
    if isinstance(obj, dict):
        return {k: sanitize_for_json(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [sanitize_for_json(elem) for elem in obj]
    elif isinstance(obj, (str, int, float, bool)) or obj is None:
        return obj
    else:
        return str(obj)

def construct_full_image_url(image_path: str | None) -> str | None:
    if not image_path: return None
    if image_path.startswith(('http://', 'https://')):
        return image_path
    if image_path.startswith('/'):
        return f"http://{settings.API_HOST}:{settings.API_PORT}{image_path}"
    return image_path

router = APIRouter(tags=["Avatar"])

def is_websocket_active(ws: WebSocket) -> bool:
    return ws.application_state == WebSocketState.CONNECTED

async def _handle_idle_prompt(websocket: WebSocket):
    if not is_websocket_active(websocket): return
    idle_prompts = ["มีอะไรให้ช่วยเกี่ยวกับจังหวัดน่านไหมคะ?", "ลองถามเกี่ยวกับวัดสวยๆ ในเมืองน่านดูสิคะ"]
    text_to_speak = random.choice(idle_prompts)
    payload = {
        "answer": text_to_speak, # [FIX] Use 'answer' for consistency
        "emotion": "talking",
        "isIdlePrompt": True 
    }
    try:
        await websocket.send_text(json.dumps(payload))
        audio_bytes = await speech_handler_instance.synthesize_speech_to_bytes(text_to_speak)
        if is_websocket_active(websocket):
            await websocket.send_bytes(audio_bytes)
    except Exception as e:
        logging.error(f"❌ Error sending idle prompt data: {e}", exc_info=True)

async def process_orchestrator_result(result: Dict[str, Any]) -> Dict[str, Any]:
    """
    Translates the V5 orchestrator response into a WebSocket payload with consistent keys.
    """
    # [FIX] Start with the correct 'answer' key
    payload = {
        "answer": result.get("answer", "ขออภัยค่ะ มีบางอย่างผิดพลาด"),
        "action": result.get("action"),
        "action_payload": result.get("action_payload"),
    }

    image_url = result.get("image_url")
    image_gallery = result.get("image_gallery", [])
    sources = result.get("sources", [])
    
    if result.get("action") and "MUSIC" in result["action"]:
        payload["emotion"] = "listening"
    elif image_url or image_gallery or sources:
        payload["emotion"] = "happy"
    else:
        payload["emotion"] = "talking"

    # [FIX] Use consistent 'image_url' key with underscore
    payload["image_url"] = construct_full_image_url(image_url)
    payload["image_gallery"] = [construct_full_image_url(url) for url in image_gallery if url]
    
    processed_sources = []
    for source in sources:
        processed_source = source.copy()
        raw_urls = processed_source.get("image_urls", [])
        processed_source["image_urls"] = [construct_full_image_url(url) for url in raw_urls if url]
        processed_sources.append(processed_source)
    payload["sources"] = processed_sources

    logging.debug(f"Generated WebSocket Payload: {payload}")
    return payload

# --- The rest of the file is correct and does not need changes ---
# ... (_handle_audio_input, _handle_text_input, handle_avatar_chat) ...

async def _handle_audio_input(websocket: WebSocket, audio_bytes: bytes, orchestrator: RAGOrchestrator):
    if not is_websocket_active(websocket): return
    try:
        await websocket.send_text(json.dumps({"emotion": "thinking"}))
        transcribed_text = await speech_handler_instance.transcribe_audio_bytes(audio_bytes)
        if not transcribed_text:
            if is_websocket_active(websocket):
                await websocket.send_text(json.dumps({"emotion": "confused", "answer": "ไม่ได้ยินที่คุณพูดเลยค่ะ ลองพูดอีกครั้งนะคะ", "isEmpty": True}))
            return
        logging.info(f"👂 [Avatar WebSocket] Heard (Raw): '{transcribed_text}'")
        result = await orchestrator.answer_query(transcribed_text, mode='voice')
        payload = await process_orchestrator_result(result)
        text_to_speak = payload.get("answer")
        audio_task = None
        if text_to_speak:
            audio_task = asyncio.create_task(speech_handler_instance.synthesize_speech_to_bytes(text_to_speak))
        sanitized_payload = sanitize_for_json(payload)
        await websocket.send_text(json.dumps(sanitized_payload))
        if audio_task:
            response_audio_bytes = await audio_task
            if is_websocket_active(websocket):
                await websocket.send_bytes(response_audio_bytes)
    except (WebSocketDisconnect, StarletteWebSocketDisconnect):
        # Client disconnected during response - this is normal, don't log as error
        logging.info("📴 [WebSocket] Client disconnected during audio response (normal behavior)")
    except Exception as e:
        logging.error(f"❌ [WebSocket] Error in audio handling: {e}", exc_info=True)
        if is_websocket_active(websocket):
            try:
                await websocket.send_text(json.dumps({"emotion": "confused", "answer": "ขออภัยค่ะ มีข้อผิดพลาดภายใน โปรดลองอีกครั้ง"}))
            except:
                pass  # Client already disconnected

async def _handle_text_input(websocket: WebSocket, query_text: str, orchestrator: RAGOrchestrator):
    if not is_websocket_active(websocket): return
    try:
        await websocket.send_text(json.dumps({"emotion": "thinking"}))
        logging.info(f"⌨️ [Avatar WebSocket] Heard (Text): '{query_text}'")
        result = await orchestrator.answer_query(query_text, mode='voice')
        payload = await process_orchestrator_result(result)
        text_to_speak = payload.get("answer")
        audio_task = None
        if text_to_speak:
             audio_task = asyncio.create_task(speech_handler_instance.synthesize_speech_to_bytes(text_to_speak))
        sanitized_payload = sanitize_for_json(payload)
        await websocket.send_text(json.dumps(sanitized_payload))
        if audio_task:
            response_audio_bytes = await audio_task
            if is_websocket_active(websocket):
                await websocket.send_bytes(response_audio_bytes)
    except (WebSocketDisconnect, StarletteWebSocketDisconnect):
        # Client disconnected during response - this is normal, don't log as error
        logging.info("📴 [WebSocket] Client disconnected during text response (normal behavior)")
    except Exception as e:
        logging.error(f"❌ [WebSocket] Error in text handling: {e}", exc_info=True)
        if is_websocket_active(websocket):
            try:
                await websocket.send_text(json.dumps({"emotion": "confused", "answer": "ขออภัยค่ะ มีข้อผิดพลาดภายใน โปรดลองอีกครั้ง"}))
            except:
                pass  # Client already disconnected

@router.websocket("/ws")
async def handle_avatar_chat(websocket: WebSocket):
    await websocket.accept()
    logging.info("✅ [Avatar WebSocket] Client connected.")
    try:
        orchestrator: RAGOrchestrator = websocket.app.state.rag_orchestrator
    except AttributeError:
        await websocket.close(code=1011, reason="Server configuration error.")
        return
    try:
        while True:
            message = await websocket.receive()
            if message["type"] == "websocket.receive":
                if text_data := message.get("text"):
                    try:
                        data = json.loads(text_data)
                        if query := data.get("query"):
                            await _handle_text_input(websocket, query, orchestrator)
                        elif data.get("action") == "idle_prompt":
                            await _handle_idle_prompt(websocket)
                    except Exception as e:
                         logging.error(f"Error processing text message: {e}", exc_info=True)
                elif bytes_data := message.get("bytes"):
                    await _handle_audio_input(websocket, bytes_data, orchestrator)
            elif message["type"] == "websocket.disconnect":
                break 
    except Exception as e:
        logging.error(f"❌ [Avatar WebSocket] Major error in loop: {e}", exc_info=True)
    finally:
        logging.info("🛑 [Avatar WebSocket] Connection handler finished.")