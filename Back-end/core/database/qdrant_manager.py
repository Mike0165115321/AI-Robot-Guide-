# /core/database/qdrant_manager.py
# (โค้ดที่แก้ไขแล้ว พร้อมคำอธิบายละเอียด)

import uuid
import asyncio
import logging
from qdrant_client import QdrantClient, AsyncQdrantClient, models 
from sentence_transformers import SentenceTransformer
from core.config import settings
import numpy as np 

class QdrantManager:
    def __init__(self):
        # สร้าง Client สำหรับเชื่อมต่อ Qdrant แบบ Asynchronous ตาม Host/Port ที่ตั้งค่าไว้
        self.client = AsyncQdrantClient(host=settings.QDRANT_HOST, port=settings.QDRANT_PORT)
        
        logging.info("🔄 กำลังโหลดโมเดล Embedding...") 
        
        # โหลดโมเดล SentenceTransformer (เช่น intfloat/multilingual-e5-large) 
        # เพื่อใช้แปลงข้อความเป็น Vector (Embedding)
        # device=settings.DEVICE จะกำหนดว่าจะรันบน CPU หรือ GPU (cuda)
        self.embedding_model = SentenceTransformer(
            settings.EMBEDDING_MODEL_NAME, 
            device=settings.DEVICE 
        )
        logging.info(f"✅ โหลดโมเดล Embedding '{settings.EMBEDDING_MODEL_NAME}' บน '{settings.DEVICE}' เรียบร้อยแล้ว")

        # ชื่อ Collection ที่จะใช้เก็บข้อมูลใน Qdrant
        self.collection_name = settings.QDRANT_COLLECTION_NAME
        
    async def initialize(self):
        """เริ่มการทำงาน: ตรวจสอบว่ามี Collection หรือยัง ถ้ายังไม่มีให้สร้างใหม่"""
        try:
            # ลองดึงข้อมูล Collection ดูว่ามีอยู่จริงไหม
            await self.client.get_collection(collection_name=self.collection_name)
            logging.info(f"✅ Collection '{self.collection_name}' already exists (Vector-Only).")
        except Exception:
            # ถ้าไม่มี (เกิด Error) ให้สร้าง Collection ใหม่
            logging.warning(f"⚠️ ไม่พบ Collection '{self.collection_name}' กำลังสร้างใหม่ (Vector-Only)...") 
            await self.client.recreate_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    # กำหนดขนาด Vector ตามขนาดของโมเดล Embedding ที่ใช้
                    size=self.embedding_model.get_sentence_embedding_dimension(),
                    # ใช้ Cosine Distance ในการวัดความเหมือน
                    distance=models.Distance.COSINE
                )
            )
            logging.info(f"✅ สร้าง Collection '{self.collection_name}' สำเร็จแล้ว (Vector-Only)") 

    async def close(self):
        """ปิดการเชื่อมต่อกับ Qdrant เมื่อเลิกใช้งาน"""
        logging.info("⏳ Closing Qdrant client connection...")
        try:
            await self.client.close()
            logging.info("✅ Qdrant client closed.")
        except Exception as e:
            logging.error(f"❌ เกิดข้อผิดพลาดในการปิด Qdrant client: {e}")

    def _create_vector_sync(self, text: str) -> np.ndarray:
        """ฟังก์ชันภายใน: แปลงข้อความเป็น Vector (ทำงานแบบ Synchronous)"""
        return self.embedding_model.encode(text, convert_to_tensor=False)

    async def _create_vector(self, text: str) -> np.ndarray:
        """ฟังก์ชันภายใน: แปลงข้อความเป็น Vector แบบ Asynchronous เพื่อไม่ให้บล็อก Event Loop"""
        return await asyncio.to_thread(self._create_vector_sync, text)

    async def upsert_location(self, mongo_id: str, description: str, metadata: dict = None):
        """เพิ่มหรืออัปเดตข้อมูลลงใน Qdrant พร้อม Metadata"""
        logging.info(f"กำลังใช้ prefix 'passage:' สำหรับการจัดทำดัชนีด้วย e5-large...")
        
        # เติม prefix 'passage: ' (เป็นข้อกำหนดของโมเดล E5 เวลา index ข้อมูล)
        passage_with_prefix = f"passage: {description}"
        
        # แปลงข้อความ (ที่มี prefix) ให้เป็น Vector
        vector = await self._create_vector(passage_with_prefix)
        
        # สร้าง ID ที่ไม่ซ้ำกันสำหรับ Qdrant โดยอิงจาก mongo_id (เพื่อให้ id เดิมได้ผลลัพธ์เดิมเสมอ)
        point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, mongo_id))

        # เตรียม Payload พื้นฐาน
        payload = {
            "mongo_id": mongo_id,
            "text_content": description
        }
        
        # 🆕 ผสาน Metadata ลงใน Payload (เช่น district, sub_district, category)
        if metadata:
            # กรองเอาเฉพาะข้อมูลที่จำเป็นเพื่อไม่ให้ Payload ใหญ่เกินไป
            allowed_keys = ["district", "sub_district", "category", "title", "slug"]
            for k in allowed_keys:
                if k in metadata and metadata[k]:
                    payload[k] = metadata[k]
            # หรือจะใส่ทั้งหมดก็ได้ถ้าไม่เยอะ
            # payload.update(metadata)
            logging.info(f"➕ [Qdrant] Adding Metadata to Payload: {payload.keys()}")

        # สั่ง Upsert (Update หรือ Insert) ลง Qdrant
        await self.client.upsert(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=point_id,
                    vector=vector.tolist(), # แปลง numpy array เป็น list ปกติเพื่อส่งไป Qdrant
                    payload=payload
                )
            ],
            wait=True # รอจนกว่าจะเขียนเสร็จจริง
        )
        logging.info(f"✅ อัปเดต Vector (e5-prefixed) สำหรับ mongo_id '{mongo_id}' ลงใน Qdrant เรียบร้อยแล้ว") 
        return True
    
    async def search_similar(self, query_text: str, top_k: int = settings.QDRANT_TOP_K, metadata_filter: dict = None): 
        """
        ค้นหาข้อมูลที่ใกล้เคียงกับ query_text
        Args:
            query_text: ข้อความค้นหา
            top_k: จำนวนผลลัพธ์
            metadata_filter: Dict ระบุเงื่อนไขกรอง เช่น {"district": "ปัว", "sub_district": "ศิลาแลง"}
        """
        logging.info(f"กำลังใช้ prefix 'query:' สำหรับการค้นหาด้วย e5-large...")
        
        # เติม prefix 'query: ' (ข้อกำหนดของโมเดล E5 เวลาค้นหา)
        query_with_prefix = f"query: {query_text}"
        
        # แปลงคำค้นหาเป็น Vector
        query_vector = await self._create_vector(query_with_prefix) 
        
        # 🛡️ Construct Qdrant Filter
        qdrant_filter = None
        if metadata_filter:
            conditions = []
            if "district" in metadata_filter and metadata_filter["district"]:
                conditions.append(models.FieldCondition(
                    key="district", 
                    match=models.MatchValue(value=metadata_filter["district"])
                ))
            if "sub_district" in metadata_filter and metadata_filter["sub_district"]:
                conditions.append(models.FieldCondition(
                    key="sub_district", 
                    match=models.MatchValue(value=metadata_filter["sub_district"])
                ))
            # 🆕 Category Filter
            if "category" in metadata_filter and metadata_filter["category"]:
                # Note: This assumes 'category' field exists in payload. 
                # If using LLM based category extraction, make sure data has this field or use it as a 'should' condition.
                # For now, we use strict filtering as per Flexible RAG Design.
                 conditions.append(models.FieldCondition(
                    key="category", 
                    match=models.MatchValue(value=metadata_filter["category"])
                ))
            
            # 🆕 [SMART] Category Exclusion Filter - ไม่รวม "ข้อมูลอำเภอ" ในผลลัพธ์
            # สำหรับ Broad Query จะช่วยให้ไม่แนะนำอำเภอ
            must_not_conditions = []
            if metadata_filter.get("exclude_categories"):
                for cat in metadata_filter["exclude_categories"]:
                    must_not_conditions.append(models.FieldCondition(
                        key="category",
                        match=models.MatchValue(value=cat)
                    ))
            
            if conditions or must_not_conditions:
                qdrant_filter = models.Filter(
                    must=conditions if conditions else None,
                    must_not=must_not_conditions if must_not_conditions else None
                )
                logging.info(f"🛡️ [Qdrant] Applied Filter: {metadata_filter}")

        try:
            # ส่งคำสั่งค้นหาไปที่ Qdrant
            search_results = await self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector.tolist(),
                query_filter=qdrant_filter, # Apply Filter here
                limit=top_k,       # จำนวนผลลัพธ์สูงสุดที่ต้องการ
                with_payload=True  # ขอข้อมูล payload (เนื้อหา) กลับมาด้วย
            )
            
            logging.info(f"✅ [Qdrant Raw Results] คำค้น '{query_text}' พบ {len(search_results)} ผลลัพธ์ (ก่อน Reranking):")
            if not search_results and metadata_filter:
                 logging.warning(f"⚠️ [Qdrant] ไม่พบผลลัพธ์ภายใต้ Filter: {metadata_filter}")
            
            # วนลูปแสดงผลลัพธ์ใน Log เพื่อตรวจสอบความถูกต้อง
            for i, result in enumerate(search_results):
                text_preview = result.payload.get('text_content', 'N/A')[:100].strip() + "..."
                
                logging.info(
                    f"  ผลลัพธ์ #{i+1} | "
                    f"คะแนน: {result.score:.4f} | " 
                    f"Mongo_ID: {result.payload.get('mongo_id')} | "
                    f"เนื้อหา: '{text_preview}'"
                )
            return search_results
        except Exception as e:
            logging.error(f"❌ [Qdrant] การค้นหาล้มเหลว (DB อาจจะล่ม): {e}")
            return []
    
    async def delete_vector(self, mongo_id: str):
        """ลบข้อมูลออกจาก Qdrant ตาม mongo_id"""
        # คำนวณ Point ID เดิมจาก mongo_id เพื่อหาตัวที่จะลบ
        point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, mongo_id))
        try:
            # สั่งลบ Point นั้นออกจาก Collection
            await self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(points=[point_id]),
                wait=True
            )
            logging.info(f"✅ ลบ Vector สำหรับ mongo_id '{mongo_id}' จาก Qdrant เรียบร้อยแล้ว")
            return True
        except Exception as e:
            logging.error(f"❌ เกิดข้อผิดพลาดในการลบ Vector สำหรับ mongo_id '{mongo_id}': {e}")
            return False