# /core/config.py

import os
from dotenv import load_dotenv
from typing import Optional

load_dotenv()

class Settings:
    MONGO_URI = os.getenv("MONGO_URI", "mongodb://localhost:27017/")
    MONGO_DATABASE_NAME = "nanaiguide"
    import torch
    DEVICE: str = "cuda" if torch.cuda.is_available() else "cpu"
    
    EMBEDDING_MODEL_NAME = os.getenv("EMBEDDING_MODEL_NAME", "intfloat/multilingual-e5-large")
    RERANKER_MODEL_NAME = os.getenv("RERANKER_MODEL_NAME", "BAAI/bge-reranker-base")
    
    # 2. LLM Models
    GEMINI_MODEL = "gemini-2.5-flash"          # โมเดลหลัก (Google)
    GROQ_LLAMA_MODEL = "llama-3.3-70b-versatile" # โมเดลหลัก (Groq - RAG/Interpreter)
    GROQ_SMALL_TALK_MODEL = "llama-3.1-8b-instant" # โมเดลคุยเล่น (เน้นเร็ว)
    
    # 3. Speech & Audio Models
    GROQ_WHISPER_MODEL = "whisper-large-v3" # Groq STT (เร็วสุด)
    WHISPER_MODEL_SIZE = "medium"                       # Local Whisper Fallback (base/small/medium)
    TTS_VOICE = "th-TH-PremwadeeNeural"               # เสียงพูด (Edge TTS)
    
    QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
    QDRANT_PORT = int(os.getenv("QDRANT_PORT", 6333))
    QDRANT_COLLECTION_NAME = "nan_locations"

    QDRANT_TOP_K: int = 8
    IMAGE_FALLBACK_THRESHOLD: int = 2
    GOOGLE_IMAGE_MAX_RESULTS: int = 3
    SOURCE_CARD_IMAGE_LIMIT: int = 1
    FINAL_GALLERY_IMAGE_LIMIT: int = 3
    TOP_K_RERANK_VOICE = 5
    TOP_K_RERANK_TEXT = 5

    GEMINI_API_KEYS = [key.strip() for key in os.getenv("GEMINI_API_KEYS", "").split(',') if key.strip()]
    GROQ_API_KEYS = [key.strip() for key in os.getenv("GROQ_API_KEYS", "").split(',') if key.strip()]
    YOUTUBE_API_KEY: Optional[str] = os.getenv("YOUTUBE_API_KEY", None)
    GOOGLE_API_KEY: str | None = os.getenv("GOOGLE_API_KEY")
    GOOGLE_CSE_ID: str | None = os.getenv("GOOGLE_CSE_ID")

    API_HOST: str = os.getenv("API_HOST", "127.0.0.1")
    API_PORT: int = int(os.getenv("API_PORT", 9090))

    # Path Configuration (Computed relative to this file: Back-end/core/config.py)
    # config.py is in Back-end/core, so parent.parent.parent is the root
    from pathlib import Path
    PROJECT_ROOT: Path = Path(__file__).resolve().parent.parent.parent
    FRONTEND_DIR: Path = PROJECT_ROOT / "frontend"
    IMG_DIR: Path = PROJECT_ROOT / "Back-end" / "static" / "images"

settings = Settings()