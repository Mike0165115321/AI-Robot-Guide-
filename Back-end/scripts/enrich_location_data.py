
import asyncio
import os
import sys
import logging
import json
from typing import List, Optional
from pymongo import MongoClient
from qdrant_client import QdrantClient, models
from groq import AsyncGroq

# Setup paths
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.config import settings
from core.ai_models.key_manager import groq_key_manager

# Configure Logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class LocationEnricher:
    def __init__(self):
        self.mongo = MongoClient(settings.MONGO_URI)
        self.db = self.mongo[settings.MONGO_DATABASE_NAME]
        self.collection = self.db["nan_locations"]
        
        # Initialize Groq
        api_key = groq_key_manager.get_key()
        if not api_key:
            raise ValueError("Groq API Key not found")
        self.groq = AsyncGroq(api_key=api_key)
        
        # Initialize Qdrant for updating payload
        self.qdrant = QdrantClient(host=settings.QDRANT_HOST, port=settings.QDRANT_PORT)
        self.collection_name = settings.QDRANT_COLLECTION_NAME

    async def extract_info(self, text: str) -> dict:
        """
        Ask LLM to extract district, sub-district, AND category from text.
        """
        prompt = f"""
        วิเคราะห์ข้อความต่อไปนี้แล้วระบุ "อำเภอ", "ตำบล" และ "หมวดหมู่" ของสถานที่ท่องเที่ยวในจังหวัดน่าน
        
        ข้อความ: "{text[:1500]}"
        
        ตอบกลับเป็น JSON Format เท่านั้น:
        {{
            "district": "ชื่ออำเภอภาษาไทย (ตัดคำว่าอำเภอออก)",
            "sub_district": "ชื่อตำบลภาษาไทย (ตัดคำว่าตำบลออก) หรือ null ถ้าไม่มี",
            "category": "หมวดหมู่หลัก (lowercase ภาษาอังกฤษ) เลือกจาก: accommodation, food, attraction, souvenir, culture, nature, cafe, other"
        }}
        
        ตัวอย่าง:
        - "อ.เมืองน่าน" -> {{"district": "เมืองน่าน", "sub_district": null, "category": "attraction"}}
        - "ร้านกาแฟดอยกว่าง ปัว" -> {{"district": "ปัว", "sub_district": null, "category": "cafe"}}
        - "โรงแรมภูเพียง" -> {{"district": "ภูเพียง", "sub_district": null, "category": "accommodation"}}
        - "ไม่ระบุ" -> {{"district": null, "sub_district": null, "category": null}}
        """
        
        try:
            chat_completion = await self.groq.chat.completions.create(
                messages=[
                    {"role": "system", "content": "คุณคือผู้เชี่ยวชาญข้อมูลภูมิศาสตร์จังหวัดน่าน response JSON only."},
                    {"role": "user", "content": prompt}
                ],
                model=settings.GROQ_LLAMA_MODEL,
                temperature=0.0,
                response_format={"type": "json_object"},
            )
            return json.loads(chat_completion.choices[0].message.content)
        except Exception as e:
            logging.error(f"LLM Extraction failed: {e}")
            return {"district": None, "sub_district": None, "category": None}

    async def process_locations(self):
        """
        Iterate through all locations and enrich them.
        """
        cursor = self.collection.find({})
        total = self.collection.count_documents({})
        logging.info(f"🚀 Starting enrichment for {total} locations...")
        
        count = 0
        updated_count = 0
        
        for doc in cursor:
            count += 1
            # Check if already processed (Skip ONLY if both district AND category exist)
            current_metadata = doc.get("metadata", {})
            if current_metadata.get("district") and current_metadata.get("category"):
                logging.info(f"⏭️  Skipping {doc.get('title')} (Already has full metadata)")
                continue
                
            # Prepare text for analysis
            text_context = f"{doc.get('title')} {doc.get('summary')} {doc.get('category', '')} {doc.get('location_data', {}).get('address', '')}"
            logging.info(f"🧩 Processing {count}/{total}: {doc.get('title')}...")
            
            # 1. Extract Info
            extracted = await self.extract_info(text_context)
            district = extracted.get("district")
            sub_district = extracted.get("sub_district")
            category = extracted.get("category")
            
            if district or category:
                # 2. Update MongoDB
                update_fields = {}
                if district: update_fields["metadata.district"] = district
                if sub_district: update_fields["metadata.sub_district"] = sub_district
                if category: update_fields["metadata.category"] = category
                
                self.collection.update_one(
                    {"_id": doc["_id"]},
                    {"$set": update_fields}
                )
                
                # 3. Update Qdrant Payload (Critical for Filtering)
                try:
                    # Simpler approach: Filter by payload.mongo_id
                    scroll_result = self.qdrant.scroll(
                        collection_name=self.collection_name,
                        scroll_filter=models.Filter(
                            must=[models.FieldCondition(key="mongo_id", match=models.MatchValue(value=str(doc["_id"])))]
                        ),
                        limit=1
                    )
                    
                    if scroll_result[0]:
                        point_id = scroll_result[0][0].id
                        payload_update = {}
                        if district: payload_update["district"] = district
                        if sub_district: payload_update["sub_district"] = sub_district
                        if category: payload_update["category"] = category
                        
                        self.qdrant.set_payload(
                            collection_name=self.collection_name,
                            payload=payload_update,
                            points=[point_id]
                        )
                        logging.info(f"✅ Updated Qdrant payload for {doc.get('title')}")
                except Exception as q_e:
                    logging.error(f"❌ Failed to update Qdrant for {doc.get('title')}: {q_e}")

                logging.info(f"✅ Enriched: {doc.get('title')} -> {district} / {category}")
                updated_count += 1
            else:
                logging.warning(f"⚠️  Could not identify metadata for: {doc.get('title')}")
                
        logging.info(f"🎉 Enrichment Complete! Updated {updated_count} documents.")

if __name__ == "__main__":
    enricher = LocationEnricher()
    asyncio.run(enricher.process_locations())
