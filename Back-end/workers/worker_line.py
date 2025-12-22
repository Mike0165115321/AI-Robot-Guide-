"""
LINE Worker - Consumer for LINE Message Queue

This worker pulls messages from Redis queue, processes them with RAG Orchestrator,
and sends rich responses back to LINE users with support for:
- Text messages
- Image carousels
- Location messages with Google Maps links
- YouTube song cards
- Source reference cards
"""

import sys
import os
import asyncio
import logging
from dotenv import load_dotenv

# Ensure we can import from core
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.database.redis_client import redis_client
from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from core.ai_models.query_interpreter import QueryInterpreter
from core.ai_models.rag_orchestrator import RAGOrchestrator
from core.services.line_message_builder import LineMessageBuilder
from linebot import LineBotApi
from linebot.models import TextSendMessage
from linebot.exceptions import LineBotApiError

# Load Env
load_dotenv()
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("WorkerLine")

LINE_CHANNEL_ACCESS_TOKEN = os.getenv("LINE_CHANNEL_ACCESS_TOKEN")
if not LINE_CHANNEL_ACCESS_TOKEN:
    logger.warning("⚠️ LINE_CHANNEL_ACCESS_TOKEN not set! Worker will fail to reply.")

line_bot_api = LineBotApi(LINE_CHANNEL_ACCESS_TOKEN) if LINE_CHANNEL_ACCESS_TOKEN else None

# Base URL for static files (for image URLs)
STATIC_BASE_URL = os.getenv("LINE_STATIC_BASE_URL", "")


async def process_message(orchestrator: RAGOrchestrator, data: dict) -> None:
    """
    Process a single message from the queue and reply via LINE.
    """
    user_id = data.get("user_id")
    text = data.get("text")
    reply_token = data.get("reply_token")
    
    logger.info(f"📩 Received message from {user_id}: {text}")
    
    try:
        # Call AI with LINE-specific intent routing
        response = await orchestrator.answer_query(
            query=text, 
            session_id=user_id,
            frontend_intent="LINE"  # Let orchestrator know this is from LINE
        )
        
        # Log the response structure for debugging
        logger.info(f"🔍 Response keys: {response.keys() if isinstance(response, dict) else 'N/A'}")
        
        # Build LINE messages from the response
        messages = LineMessageBuilder.build_response_messages(
            response=response,
            base_url=STATIC_BASE_URL
        )
        
        if not messages:
            # Fallback: If no messages were built, create a simple text reply
            answer = "ขออภัยครับ ไม่สามารถประมวลผลคำตอบได้"
            if isinstance(response, dict):
                answer = response.get("answer", answer)
            messages = [TextSendMessage(text=answer)]
        
        # Log what we're sending
        logger.info(f"📤 Sending {len(messages)} message(s) to LINE:")
        for i, msg in enumerate(messages):
            msg_type = type(msg).__name__
            logger.info(f"   [{i+1}] {msg_type}")
        
        # Send reply via LINE API
        if line_bot_api:
            try:
                line_bot_api.reply_message(reply_token, messages)
                logger.info("✅ Reply sent to LINE successfully!")
            except LineBotApiError as e:
                logger.error(f"❌ LINE API Error: {e.status_code} - {e.error.message}")
                # If reply token expired (> 30 seconds), try push message instead
                if e.status_code == 400 and "Invalid reply token" in str(e.error.message):
                    logger.info("⏱️ Reply token expired, attempting push message...")
                    try:
                        line_bot_api.push_message(user_id, messages)
                        logger.info("✅ Push message sent successfully!")
                    except LineBotApiError as push_err:
                        logger.error(f"❌ Push message also failed: {push_err.error.message}")
        else:
            logger.error("❌ Cannot reply: LINE_CHANNEL_ACCESS_TOKEN missing.")
            
    except Exception as ai_err:
        logger.error(f"❌ AI Processing Error: {ai_err}", exc_info=True)
        # Send error message to user
        if line_bot_api:
            try:
                error_msg = TextSendMessage(text="ขออภัยครับ เกิดข้อผิดพลาดในการประมวลผล กรุณาลองใหม่อีกครั้งนะครับ 🙏")
                line_bot_api.reply_message(reply_token, error_msg)
            except:
                pass  # Best effort


async def main():
    logger.info("🚀 Starting LINE Worker (Full Feature Mode)...")
    
    logger.info("📦 Initializing Managers...")
    mongo_manager = MongoDBManager()
    qdrant_manager = QdrantManager()
    query_interpreter = QueryInterpreter()
    
    logger.info("⚙️ Initializing RAG Orchestrator...")
    orchestrator = RAGOrchestrator(
        mongo_manager=mongo_manager,
        qdrant_manager=qdrant_manager,
        query_interpreter=query_interpreter
    )
    
    # Init DB connections (Async)
    try:
        logger.info("🔌 Connecting to Qdrant...")
        await qdrant_manager.initialize()
        logger.info("✅ Qdrant initialized")
    except Exception as e:
        logger.error(f"❌ Failed to init Qdrant: {e}")

    print("\n" + "="*60)
    print("✅ LINE Worker (Full Feature Mode) is ready!")
    print("   รองรับ: ข้อความ | รูปภาพ | แผนที่ | เพลง | การ์ดอ้างอิง")
    print("   (ส่งข้อความในไลน์ได้เลยครับ)")
    print("="*60 + "\n")
    logger.info("⌛ Waiting for messages from Redis queue...")

    while True:
        try:
            # Use asyncio.to_thread to run the blocking Redis pop
            data = await asyncio.to_thread(redis_client.pop_message)
            
            if data:
                await process_message(orchestrator, data)
                
        except Exception as e:
            logger.error(f"❌ Worker Loop Error: {e}", exc_info=True)
            await asyncio.sleep(5)


if __name__ == "__main__":
    asyncio.run(main())
