from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel
from typing import Optional
from datetime import datetime

router = APIRouter(tags=["Analytics"])

class FeedbackSchema(BaseModel):
    session_id: str
    query: str
    response: str
    feedback_type: str # "like" or "dislike"
    reason: Optional[str] = None

@router.post("/submit_feedback")
async def submit_feedback(request: Request, feedback: FeedbackSchema):
    """
    Endpoint for submitting user feedback (Like/Dislike).
    """
    # Access AnalyticsService from app state
    if not hasattr(request.app.state, 'analytics_service'):
        raise HTTPException(status_code=500, detail="Analytics service not initialized")
    
    analytics_service = request.app.state.analytics_service
    
    await analytics_service.log_feedback(
        session_id=feedback.session_id,
        query=feedback.query,
        response=feedback.response,
        feedback_type=feedback.feedback_type,
        reason=feedback.reason
    )
    
    return {"status": "success", "message": "Feedback recorded"}
