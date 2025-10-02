# /core/database/qdrant_manager.py (ฉบับแก้ไขที่ถูกต้อง)

import uuid
from qdrant_client import QdrantClient, models
from sentence_transformers import SentenceTransformer
from core.config import settings
import numpy as np 

class QdrantManager:
    def __init__(self):
        self.client = QdrantClient(host=settings.QDRANT_HOST, port=settings.QDRANT_PORT)
        
        print("🔄 Loading Embedding Model...")
        self.embedding_model = SentenceTransformer(settings.EMBEDDING_MODEL_NAME, device='cpu')
        print(f"✅ Embedding Model '{settings.EMBEDDING_MODEL_NAME}' loaded.")

        self.collection_name = settings.QDRANT_COLLECTION_NAME
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        try:
            self.client.get_collection(collection_name=self.collection_name)
            print(f"✅ Collection '{self.collection_name}' already exists.")
        except Exception:
            print(f"⚠️ Collection '{self.collection_name}' not found. Creating new one...")
            self.client.recreate_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=self.embedding_model.get_sentence_embedding_dimension(),
                    distance=models.Distance.COSINE
                )
            )
            print(f"✅ Collection '{self.collection_name}' created successfully.")

    def _create_vector(self, text: str) -> np.ndarray:
        return self.embedding_model.encode(text, convert_to_tensor=False)

    def upsert_location(self, mongo_id: str, description: str):
        vector = self._create_vector(description)
        point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, mongo_id))

        self.client.upsert(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=point_id,
                    vector=vector.tolist(),
                    payload={"mongo_id": mongo_id}
                )
            ],
            wait=True
        )
        print(f"✅ Upserted vector for mongo_id '{mongo_id}' to Qdrant.")
        return True
    
    # ====================================================================
    # --- แก้ไขฟังก์ชันนี้ทั้งหมด ---
    # ====================================================================
    def search_similar(self, query_text: str, top_k: int = 3, collection_name: str = None):
        """
        ค้นหา vector ที่คล้ายกันใน Qdrant
        - แก้ไขให้รับ collection_name ได้
        - แก้ไขให้ดึง payload กลับมาด้วย
        """
        query_vector = self._create_vector(query_text)

        # ถ้าไม่ได้ระบุ collection_name มา ให้ใช้ค่า default ของ class
        effective_collection_name = collection_name or self.collection_name

        search_results = self.client.search(
            collection_name=effective_collection_name,
            query_vector=query_vector.tolist(),
            limit=top_k,
            with_payload=True  # <-- เพิ่มบรรทัดนี้เข้าไป! สำคัญมาก
        )
        
        print(f"🔍 Found {len(search_results)} similar results for query: '{query_text}'")
        return search_results
    # ====================================================================
    
    def delete_vector(self, mongo_id: str):
        """
        ลบ Vector Point ที่มี mongo_id ที่ระบุ ออกจาก Collection
        """
        # สร้าง Point ID ที่ "คงที่" แบบเดียวกับตอน upsert เพื่อให้หาเจอ
        point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, mongo_id))

        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(points=[point_id]),
                wait=True
            )
            print(f"✅ Deleted vector for mongo_id '{mongo_id}' from Qdrant.")
            return True
        except Exception as e:
            print(f"❌ Error deleting vector for mongo_id '{mongo_id}': {e}")
            return False