# /core/config.py

import os
from dotenv import load_dotenv

load_dotenv()

class Settings:
    """
    แผงควบคุมหลักสำหรับโปรเจกต์ AI Guide จังหวัดน่าน
    โหลดและจัดการการตั้งค่าทั้งหมดของระบบ
    """
    MONGO_URI = os.getenv("MONGO_URI", "mongodb://localhost:27017/")
    MONGO_DATABASE_NAME = "nanaiguide"
    
    QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
    QDRANT_PORT = int(os.getenv("QDRANT_PORT", 6333))
    QDRANT_COLLECTION_NAME = "nan_locations"

    EMBEDDING_MODEL_NAME = "intfloat/multilingual-e5-large"
    RERANKER_MODEL_NAME = 'BAAI/bge-reranker-base'
    
    GEMINI_API_KEYS = [key.strip() for key in os.getenv("GEMINI_API_KEYS", "").split(',') if key.strip()]
    GROQ_API_KEYS = [key.strip() for key in os.getenv("GROQ_API_KEYS", "").split(',') if key.strip()]
    
    GEMINI_MODEL = "gemini-1.5-flash-latest"
    GROQ_LLAMA_MODEL = "llama-3.3-70b-versatile" 

settings = Settings()