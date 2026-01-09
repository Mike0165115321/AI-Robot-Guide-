import logging
import time
from fastapi import APIRouter, HTTPException, BackgroundTasks
from pydantic import BaseModel
from typing import Optional, Dict, Any

from core.ai_models.frontline_handler import frontline_handler

router = APIRouter(
    prefix="/assistant",
    tags=["Google Assistant Gatekeeper"],
    responses={404: {"description": "Not found"}},
)

class AssistantQuery(BaseModel):
    text: str
    session_id: str = "default_session"
    language: str = "th"

@router.post("/query")
async def query_assistant(request: AssistantQuery):
    """
    Direct access to Google Assistant (Frontline).
    Returns raw intent/reply/metadata.
    Used by Frontend Director to decide next steps.
    """
    start_time = time.time()
    try:
        # 1. Process with Google Assistant (Pass Language!)
        result = await frontline_handler.process_query(request.text, language_code=request.language)
        
        # 2. Add processing time
        result["processing_time"] = round(time.time() - start_time, 2)
        
        # 3. Log
        logging.info(f"🛡️ [API-Assistant] '{request.text}' -> {result['intent']}")
        
        return {
            "success": True,
            "data": result
        }

    except Exception as e:
        logging.error(f"❌ [API-Assistant] Error: {e}")
        return {
            "success": False,
            "error": str(e),
            "data": {"intent": "ERROR", "reply": None}
        }
