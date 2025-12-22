import os
import json
from dotenv import load_dotenv
from fastapi import APIRouter, Request, HTTPException, Header
from linebot import LineBotApi, WebhookHandler
from linebot.exceptions import InvalidSignatureError
from linebot.models import MessageEvent, TextMessage
from core.database.redis_client import redis_client

# Load environment variables from .env file
load_dotenv()

router = APIRouter()

# LINE Configuration
LINE_CHANNEL_ACCESS_TOKEN = os.getenv("LINE_CHANNEL_ACCESS_TOKEN")
LINE_CHANNEL_SECRET = os.getenv("LINE_CHANNEL_SECRET")

# Initialize SDK (Verify if tokens exist to avoid startup errors if not set)
if LINE_CHANNEL_ACCESS_TOKEN and LINE_CHANNEL_SECRET:
    # Debug: Show that tokens are loaded (masked for security)
    print(f"[LINE Webhook] Secret loaded: {LINE_CHANNEL_SECRET[:4]}...{LINE_CHANNEL_SECRET[-4:]} (len={len(LINE_CHANNEL_SECRET)})")
    line_bot_api = LineBotApi(LINE_CHANNEL_ACCESS_TOKEN)
    handler = WebhookHandler(LINE_CHANNEL_SECRET)
else:
    print(f"[WARNING] LINE tokens not set! TOKEN={bool(LINE_CHANNEL_ACCESS_TOKEN)}, SECRET={bool(LINE_CHANNEL_SECRET)}")
    handler = None

@router.post("/webhook")
async def line_webhook(request: Request, x_line_signature: str = Header(None)):
    """
    Receive Webhook from LINE
    """
    if not handler:
        raise HTTPException(status_code=500, detail="LINE Messaging API not configured.")

    if x_line_signature is None:
        raise HTTPException(status_code=400, detail="X-Line-Signature header missing")

    body = await request.body()
    body_str = body.decode('utf-8')

    try:
        # We manually parse logic here instead of using handler.handle(body_str, signature)
        # because we want to push raw event data to Redis, not process it immediately.
        # But we MUST use handler to verify signature first.
        
        # Verify Signature logic derived from SDK
        if not handler.parser.signature_validator.validate(body_str, x_line_signature):
             raise InvalidSignatureError
        
        # Parse events
        events = handler.parser.parse(body_str, x_line_signature)

        for event in events:
            if isinstance(event, MessageEvent) and isinstance(event.message, TextMessage):
                # Construct payload for Worker
                payload = {
                    "reply_token": event.reply_token,
                    "user_id": event.source.user_id,
                    "text": event.message.text,
                    "type": "text"
                }
                
                # Push to Redis Queue
                redis_client.push_message(payload)
                print(f"[LINE Webhook] Event pushed for user: {event.source.user_id}")

        return "OK"

    except InvalidSignatureError:
        print("[LINE Webhook] Invalid Signature")
        raise HTTPException(status_code=400, detail="Invalid signature")
    except Exception as e:
        print(f"[LINE Webhook] Error: {e}")
        raise HTTPException(status_code=500, detail="Internal Server Error")
