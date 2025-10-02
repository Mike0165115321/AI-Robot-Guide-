# /Back-end/api/routers/chat_api.py (Final Version with Image Logic)

from fastapi import APIRouter, HTTPException, Depends, status
from fastapi import WebSocket, WebSocketDisconnect
import json
import random # [Image Logic] เพิ่ม random สำหรับ idle prompt

from ..schemas import ChatQuery, ChatResponse 
from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from core.ai_models.rag_orchestrator import RAGOrchestrator
from core.ai_models.speech_handler import speech_handler_instance
from core.config import settings # [Image Logic] Import settings เพื่อเอา HOST, PORT

# --- Dependency Injection Setup ---
mongo_manager = MongoDBManager()
qdrant_manager = QdrantManager()
rag_orchestrator = RAGOrchestrator(
    mongo_manager=mongo_manager,
    qdrant_manager=qdrant_manager
)
def get_rag_orchestrator():
    return rag_orchestrator
def get_speech_handler():
    return speech_handler_instance

router = APIRouter(
    tags=["Chat"]
)

# --- [Image Logic] 1. สร้าง Helper Function สำหรับสร้าง URL เต็ม ---
def construct_full_image_url(image_path: str | None) -> str | None:
    """สร้าง URL เต็มสำหรับรูปภาพจาก path ที่ได้จาก orchestrator"""
    if not image_path:
        return None
    # image_path จะมาในรูปแบบ "/static/images/file.jpg"
    # เราต้องเติม http://host:port เข้าไปข้างหน้า
    return f"http://{settings.API_HOST}:{settings.API_PORT}{image_path}"

# ==================================
#  ENDPOINT สำหรับ TEXT CHAT MODE
# ==================================
@router.post("/", response_model=ChatResponse)
async def handle_chat_query(
    query: ChatQuery,
    orchestrator: RAGOrchestrator = Depends(get_rag_orchestrator)
):
    try:
        print(f"💬 [API-Text] Received query: '{query.query}'")
        
        result = orchestrator.answer_query(query.query, mode='text')
        
        if not result or "answer" not in result:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="AI failed to generate a response."
            )
        
        # --- [Image Logic] 2. สร้าง Full URL สำหรับรูปภาพหลักและรูปภาพใน sources ---
        result["image_url"] = construct_full_image_url(result.get("image_url"))
        if result.get("sources"):
            for source in result["sources"]:
                source["image_url"] = construct_full_image_url(source.get("image_url"))

        print(f"✅ [API-Text] Sending response back to client.")
        # FastAPI จะแปลง result dictionary ให้เป็น ChatResponse model โดยอัตโนมัติ
        return result
        
    except Exception as e:
        print(f"❌ [API-Text] An unexpected error occurred: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An internal error occurred while processing your request."
        )

# ====================================
#  ENDPOINT สำหรับ VOICE CHAT MODE
# ====================================
@router.websocket("/ws/avatar_chat")
async def handle_avatar_chat(
    websocket: WebSocket,
    orchestrator: RAGOrchestrator = Depends(get_rag_orchestrator),
    speech_handler = Depends(get_speech_handler)
):
    await websocket.accept()
    print("✅ [WebSocket] Client connected.")
    try:
        while True:
            data = await websocket.receive()

            if 'text' in data:
                message = json.loads(data['text'])
                if message.get("action") == "idle_prompt":
                    print("⏰ Received idle_prompt action.")
                    idle_phrases = [
                        "แข่งเรือลือเลื่อง เมืองงาช้างดำ จิตรกรรมวัดภูมินทร์ แดนดินส้มสีทอง เรืองรองพระธาตุแช่แห้ง",
                        "ฉันดีใจจังเลยค่ะที่ได้เกิดในจังหวัดน่าน",
                        "มีอะไรให้ช่วยถามได้เลยนะคะ"
                    ]
                    text_to_speak = random.choice(idle_phrases)
                    
                    await websocket.send_text(json.dumps({
                        "emotion": "talking",
                        "textContent": text_to_speak,
                        "imageUrl": None
                    }))
                    
                    audio_bytes = speech_handler.synthesize_speech_to_bytes(text_to_speak)
                    await websocket.send_bytes(audio_bytes)
                    continue 
            
            elif 'bytes' in data:
                full_audio_data = data['bytes']
                print(f"🎤 [WebSocket] Received complete audio ({len(full_audio_data)} bytes).")
                
                await websocket.send_text(json.dumps({"emotion": "thinking"}))
                
                transcribed_text = speech_handler.transcribe_audio_bytes(full_audio_data)

                if transcribed_text:
                    print(f"👂 [WebSocket] Heard: '{transcribed_text}'")
                    
                    result = orchestrator.answer_query(transcribed_text, mode='voice')
                    answer_text = result['answer']
                    
                    full_image_url = construct_full_image_url(result.get("image_url"))
                    
                    await websocket.send_text(json.dumps({
                        "emotion": "talking",
                        "textContent": answer_text,
                        "imageUrl": full_image_url
                    }))

                    response_audio_bytes = speech_handler.synthesize_speech_to_bytes(answer_text)
                    await websocket.send_bytes(response_audio_bytes)
                    print(f"🗣️  [WebSocket] Sent PCM audio response.")
                else:
                    print("📝 [STT] Transcription was empty. Resetting state.")
                    await websocket.send_text(json.dumps({"emotion": None}))

    except WebSocketDisconnect:
        print("🔌 [WebSocket] Client disconnected.")
    except Exception as e:
        print(f"❌ [WebSocket] An unexpected error occurred: {e}", exc_info=True)