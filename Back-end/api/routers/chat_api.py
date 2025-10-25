import logging
from fastapi import APIRouter, HTTPException, Depends, status
from ..schemas import ChatQuery, ChatResponse 
from core.ai_models.rag_orchestrator import RAGOrchestrator
from core.config import settings
from ..dependencies import get_rag_orchestrator


def construct_full_image_url(image_path: str | None) -> str | None:
    """
    (V5.1) แก้ไขเล็กน้อย: เช็คว่าเป็น relative path ก่อน
    """
    if not image_path: return None
    if image_path.startswith(('http://', 'https://')):
        return image_path
    if image_path.startswith('/'):
        return f"http://{settings.API_HOST}:{settings.API_PORT}{image_path}"
    return image_path

router = APIRouter(tags=["Text Chat"])

@router.post("/", response_model=ChatResponse)
async def handle_text_chat(
    query: ChatQuery, 
    orchestrator: RAGOrchestrator = Depends(get_rag_orchestrator)
):
    try:
        logging.info(f"💬 [API-Text] Received raw query: '{query.query}'")
        result = await orchestrator.answer_query(query.query, mode='text')

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