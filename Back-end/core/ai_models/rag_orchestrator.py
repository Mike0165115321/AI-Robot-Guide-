# /core/ai_models/rag_orchestrator.py

# --- [Image Logic] 1. Import ไลบรารีที่จำเป็น ---
import os
import random
from pathlib import Path

from sentence_transformers import CrossEncoder 
from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from utils.helper_functions import create_synthetic_document
from . import llm_handler

# --- [Image Logic] 2. กำหนด Path ไปยังโฟลเดอร์รูปภาพ ---
# Path(__file__) -> rag_orchestrator.py
# .parent -> ai_models/
# .parent -> core/
# .parent -> Back-end/
# จากนั้นต่อไปยัง static/images/
IMAGE_DIR = Path(__file__).resolve().parent.parent.parent / "static" / "images"

class RAGOrchestrator:
    def __init__(self, mongo_manager: MongoDBManager, qdrant_manager: QdrantManager):
        print("⚙️  RAG Orchestrator (CPU-Mode) is initializing...")
        self.mongo_manager = mongo_manager
        self.qdrant_manager = qdrant_manager
        print("🔄 Loading Re-ranker Model...")
        self.reranker_model_name = 'BAAI/bge-reranker-base'
        self.reranker = CrossEncoder(self.reranker_model_name, device='cpu') 
        print(f"✅ Re-ranker Model '{self.reranker_model_name}' loaded.")
        
        if not IMAGE_DIR.is_dir():
            print(f"⚠️ WARNING: Image directory not found at {IMAGE_DIR}. Image feature will be disabled.")
            self.image_dir_exists = False
        else:
            print(f"🏞️ Image directory found at: {IMAGE_DIR}")
            self.image_dir_exists = True
            
        print("✅ RAG Orchestrator is ready.")

    def _find_random_image_by_prefix(self, prefix: str) -> str | None:
        """ค้นหาไฟล์ทั้งหมดที่ขึ้นต้นด้วย prefix ที่กำหนด แล้วสุ่มมา 1 ไฟล์"""
        if not prefix or not self.image_dir_exists:
            return None
        
        try:
            matching_files = [f for f in os.listdir(IMAGE_DIR) if f.startswith(prefix)]
            
            if not matching_files:
                print(f"🏜️ No images found with prefix: '{prefix}'")
                return None
            
            random_image_filename = random.choice(matching_files)
            
            return f"/static/images/{random_image_filename}"
            
        except Exception as e:
            print(f"❌ Error finding image with prefix '{prefix}': {e}")
            return None

    def answer_query(self, query: str, mode: str = 'voice') -> dict:
        print(f"🔍 [RAG] ({mode.upper()} MODE) Step 1: Retrieving top 10 candidates for '{query}'...")
        search_results = self.qdrant_manager.search_similar(query_text=query, top_k=10, collection_name="nan_locations")

        if not search_results:
            return {"answer": "ขออภัยค่ะ ไม่พบข้อมูลที่เกี่ยวข้องกับคำถามนี้", "sources": [], "image_url": None}

        print("📜 [RAG] Step 2: Hydrating data from MongoDB...")
        retrieved_docs = []
        for res in search_results:
            mongo_id = res.payload.get('mongo_id')
            if not mongo_id:
                continue
            
            doc = self.mongo_manager.get_location_by_id(mongo_id, collection_name="nan_locations")
            if doc:
                retrieved_docs.append(doc)

        if not retrieved_docs:
             return {"answer": "ขออภัยค่ะ พบข้อมูลที่เกี่ยวข้องแต่ไม่สามารถดึงรายละเอียดได้", "sources": [], "image_url": None}

        print("⚖️  [RAG] Step 3: Re-ranking candidates...")
        sentence_pairs = [[query, create_synthetic_document(doc)] for doc in retrieved_docs]
        scores = self.reranker.predict(sentence_pairs, show_progress_bar=False)
        
        reranked_docs = sorted(zip(scores, retrieved_docs), key=lambda x: x[0], reverse=True)
        
        top_k_rerank = 5 if mode == 'voice' else 7
        final_docs = [doc for score, doc in reranked_docs[:top_k_rerank]]

        print(f"📝 [RAG] Step 4: Formatting context from top {len(final_docs)} results...")
        context_str = "\n\n---\n\n".join([create_synthetic_document(doc) for doc in final_docs])

        final_answer = llm_handler.get_llama_response(user_query=query, context=context_str) if mode == 'voice' else llm_handler.get_gemini_response(user_query=query, context=context_str)

        image_url_to_send = None
        if final_docs:
            for doc in final_docs:
                image_prefix = doc.get("metadata", {}).get("image_prefix")
                if image_prefix:
                    image_url_to_send = self._find_random_image_by_prefix(image_prefix)
                    if image_url_to_send:
                        break
        
        source_info = []
        if final_docs:
             for doc in final_docs:
                source_image_prefix = doc.get("metadata", {}).get("image_prefix")
                source_image_url = self._find_random_image_by_prefix(source_image_prefix)

                source_info.append({
                    "title": doc.get('title', 'N/A'),
                    "summary": doc.get('summary', ''),
                    "image_url": source_image_url 
                })

        return {
            "answer": final_answer,
            "image_url": image_url_to_send,
            "sources": source_info 
        }