# /core/config.py

import os
from dotenv import load_dotenv
from typing import Optional 


load_dotenv()

class Settings:
    """
    แผงควบคุมหลักสำหรับโปรเจกต์ AI Guide จังหวัดน่าน
    โหลดและจัดการการตั้งค่าทั้งหมดของระบบ
    """
    MONGO_URI = os.getenv("MONGO_URI", "mongodb://localhost:27017/")
    MONGO_DATABASE_NAME = "nanaiguide"
    DEVICE: str = "cpu"  # หรือ "cuda" ถ้ามี GPU
    QDRANT_TOP_K: int = 5
    IMAGE_FALLBACK_THRESHOLD: int = 2
    GOOGLE_IMAGE_MAX_RESULTS: int = 3
    SOURCE_CARD_IMAGE_LIMIT: int = 3
    FINAL_GALLERY_IMAGE_LIMIT: int = 5
    
    QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
    QDRANT_PORT = int(os.getenv("QDRANT_PORT", 6333))
    QDRANT_COLLECTION_NAME = "nan_locations"
    
    GOOGLE_API_KEY: str | None = os.getenv("GOOGLE_API_KEY") 
    GOOGLE_CSE_ID: str | None = os.getenv("GOOGLE_CSE_ID")
    
    TOP_K_RERANK_VOICE = 5
    TOP_K_RERANK_TEXT = 7

    EMBEDDING_MODEL_NAME = "intfloat/multilingual-e5-large"
    RERANKER_MODEL_NAME = 'BAAI/bge-reranker-base'
    
    GEMINI_API_KEYS = [key.strip() for key in os.getenv("GEMINI_API_KEYS", "").split(',') if key.strip()]
    GROQ_API_KEYS = [key.strip() for key in os.getenv("GROQ_API_KEYS", "").split(',') if key.strip()]
    YOUTUBE_API_KEY: Optional[str] = os.getenv("YOUTUBE_API_KEY", None)
    
    GEMINI_MODEL = "gemini-2.5-flash"
    GROQ_LLAMA_MODEL = "llama-3.3-70b-versatile" 

    API_HOST: str = os.getenv("API_HOST", "127.0.0.1")
    API_PORT: int = int(os.getenv("API_PORT", 9090))

settings = Settings()