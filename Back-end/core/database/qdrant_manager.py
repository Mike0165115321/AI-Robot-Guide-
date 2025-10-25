import uuid
import asyncio
import logging
from qdrant_client import QdrantClient, AsyncQdrantClient, models 
from sentence_transformers import SentenceTransformer
from core.config import settings
import numpy as np 

class QdrantManager:
    def __init__(self):
        self.client = AsyncQdrantClient(host=settings.QDRANT_HOST, port=settings.QDRANT_PORT)
        
        logging.info("🔄 Loading Embedding Model...") 
        
        self.embedding_model = SentenceTransformer(
            settings.EMBEDDING_MODEL_NAME, 
            device=settings.DEVICE 
        )
        logging.info(f"✅ Embedding Model '{settings.EMBEDDING_MODEL_NAME}' loaded on '{settings.DEVICE}'.")

        self.collection_name = settings.QDRANT_COLLECTION_NAME
        
    async def initialize(self):
        """
        [V6.2 - NO-HYBRID] Reverted to V5.1 logic.
        Ensures the collection exists (Vector-Only).
        """
        try:
            await self.client.get_collection(collection_name=self.collection_name)
            logging.info(f"✅ Collection '{self.collection_name}' already exists (Vector-Only).")
        except Exception:
            logging.warning(f"⚠️ Collection '{self.collection_name}' not found. Creating new one (Vector-Only)...") 
            # [REVERT V6.2] ถอด Logic การสร้าง Payload Index ที่เป็นปัญหาออก
            await self.client.recreate_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=self.embedding_model.get_sentence_embedding_dimension(),
                    distance=models.Distance.COSINE
                )
            )
            logging.info(f"✅ Collection '{self.collection_name}' created successfully (Vector-Only).") 

    async def close(self):
        logging.info("⏳ Closing Qdrant client connection...")
        try:
            await self.client.close()
            logging.info("✅ Qdrant client closed.")
        except Exception as e:
            logging.error(f"❌ Error closing Qdrant client: {e}")

    def _create_vector_sync(self, text: str) -> np.ndarray:
        return self.embedding_model.encode(text, convert_to_tensor=False)

    async def _create_vector(self, text: str) -> np.ndarray:
        return await asyncio.to_thread(self._create_vector_sync, text)

    async def upsert_location(self, mongo_id: str, description: str):
        """
        [V6.2 - NO-HYBRID]
        Upserts vector and payload. (We can still keep text_content, it just won't be indexed)
        """
        vector = await self._create_vector(description)
        point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, mongo_id))

        await self.client.upsert(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=point_id,
                    vector=vector.tolist(),
                    # [REVERT V6.2] เรายังเก็บ text_content ไว้ได้ มันไม่ทำพัง แค่ไม่ได้ใช้ค้นหา
                    payload={
                        "mongo_id": mongo_id,
                        "text_content": description 
                    }
                )
            ],
            wait=True
        )
        logging.info(f"✅ Upserted vector (NoHybrid) for mongo_id '{mongo_id}' to Qdrant.") 
        return True
    
    async def search_similar(self, query_text: str, top_k: int = settings.QDRANT_TOP_K): 
        """
        [V6.2 - NO-HYBRID] Reverted to V5.1 (Vector-Only Search).
        """
        query_vector = await self._create_vector(query_text)

        # [REVERT V6.2] กลับมาใช้ .search() แบบ Vector ธรรมดา
        search_results = await self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector.tolist(),
            limit=top_k,
            with_payload=True
        )
        
        logging.info(f"🔍 Found {len(search_results)} vector results for query: '{query_text}'") 
        return search_results
    
    async def delete_vector(self, mongo_id: str):
        point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, mongo_id))

        try:
            await self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(points=[point_id]),
                wait=True
            )
            logging.info(f"✅ Deleted vector for mongo_id '{mongo_id}' from Qdrant.")
            return True
        except Exception as e:
            logging.error(f"❌ Error deleting vector for mongo_id '{mongo_id}': {e}")
            return False