import asyncio
import logging
import os
import random
import re
import math
import urllib.parse
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from sentence_transformers import CrossEncoder

# 🆕 แยก Gemini และ Groq handlers ออกจากกัน
from core.ai_models.gemini_handler import get_gemini_response
from core.ai_models.groq_handler import get_groq_response, get_small_talk_response
from core.ai_models.query_interpreter import QueryInterpreter
from core.ai_models.youtube_handler import youtube_handler_instance
from core.config import settings
from .handlers.analytics_handler import AnalyticsHandler
from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from core.tools.image_search_tool import image_search_tool_instance
from core.services.calculator_service import calculator_service  # 🧮 เครื่องคิดเลข Python
from utils.helper_functions import create_synthetic_document
from .services.session_manager import SessionManager
from .services.navigation_service import NavigationService
from .services.prompt_engine import PromptEngine
from core.services.image_service import ImageService

BACKEND_ROOT = Path(__file__).resolve().parent.parent.parent

def construct_full_image_url(image_path: str | None) -> str | None:
    if not image_path: return None
    if image_path.startswith(('http://', 'https://')):
        return image_path
    if image_path.startswith('/'):
        return f"http://{settings.API_HOST}:{settings.API_PORT}{image_path}"
    return image_path

class RAGOrchestrator:
    def __init__(
        self,
        mongo_manager: MongoDBManager,
        qdrant_manager: QdrantManager,
        query_interpreter: QueryInterpreter,
    ):
        logging.info("⚙️  กำลังเริ่มต้นระบบ RAG Orchestrator (Refactored V8.1)...")
        self.mongo_manager = mongo_manager
        self.qdrant_manager = qdrant_manager
        self.query_interpreter = query_interpreter
        self.session_manager = SessionManager(mongo_manager)
        self.prompt_engine = PromptEngine()
        self.nav_service = NavigationService(mongo_manager, self.prompt_engine)
        self.image_service = ImageService(mongo_manager)

        self.reranker_model_name = settings.RERANKER_MODEL_NAME
        self.device = settings.DEVICE

        logging.info(f"🔄 กำลังโหลดโมเดล Re-ranker ('{self.reranker_model_name}' บน '{self.device}')...")
        # โหลดโมเดล CrossEncoder สำหรับทำ Reranking
        # ช่วยจัดลำดับความสำคัญของเอกสารที่ค้นหาเจอ ให้แม่นยำขึ้น
        self.reranker = CrossEncoder(self.reranker_model_name, device=self.device)
        logging.info("✅ โหลดโมเดล Re-ranker เรียบร้อยแล้ว")

        self.log_collection = self.mongo_manager.get_collection("query_logs")
        
        self.analytics_handler = AnalyticsHandler(
            mongo_manager=self.mongo_manager,
            query_interpreter=self.query_interpreter,
            orchestrator_callback=self.answer_query
        )
        logging.info("✅ RAG Orchestrator พร้อมใช้งาน")

    def _prepare_source_and_image_data(self, docs_to_show: List[Dict[str, Any]]) -> Dict[str, Any]:
        source_info: List[dict] = []
        static_image_gallery: List[str] = []
        processed_prefixes = set()
        for doc in docs_to_show:
            if not doc: continue
            prefix = doc.get("metadata", {}).get("image_prefix")
            doc_images = []
            if prefix and prefix not in processed_prefixes:
                found_images = self.image_service.find_all_images_by_prefix(prefix)
                if found_images:
                    doc_images.extend(found_images)
                    for img_url in found_images:
                        if img_url not in static_image_gallery:
                            static_image_gallery.append(img_url)
                    processed_prefixes.add(prefix)
            source_info.append({
                "title": doc.get("title", "N/A"),
                "summary": doc.get("summary", ""),
                "image_urls": doc_images[: settings.SOURCE_CARD_IMAGE_LIMIT],
            })
        return {"source_info": source_info, "image_gallery": static_image_gallery}

    async def _handle_welcome_flow(self, session_id: Optional[str] = None, **kwargs) -> dict:
        # 🆕 Dynamic Greeting (Requested by User)
        # ไม่ใช้ Fixed Text แล้ว ให้โมเดล 8B (Small Talk) คิดคำตอบเองเลย
        corrected_query = kwargs.get('corrected_query') or "สวัสดี"
        
        logging.info(f"👋 [Welcome] Generating Dynamic Greeting for: '{corrected_query}'")
        final_answer = await get_small_talk_response(user_query=corrected_query)
        
        return {
            "answer": final_answer,
            "action": None, # ไม่ต้อง Force Analytics แล้ว
            "action_payload": None, "image_url": None, "image_gallery": [], "sources": [],
        }

    def _map_frontend_intent(self, frontend_intent: str) -> str:
        """
        🆕 แปลง frontend intent เป็น internal intent
        ไม่ต้องใช้ LLM วิเคราะห์ - ประหยัด tokens!
        """
        intent_map = {
            "MUSIC": "PLAY_MUSIC",
            "NAVIGATION": "NAVIGATE_TO",
            "FAQ": "INFORMATIONAL",
            "GENERAL": "INFORMATIONAL",
            "WELCOME": "WELCOME_FLOW",
            # เครื่องคิดเลขถูกลบออก - จัดการโดย widget ฝั่ง frontend
        }
        return intent_map.get(frontend_intent.upper(), "INFORMATIONAL")

    async def _handle_small_talk(self, corrected_query: str, **kwargs) -> dict:
        final_answer = await get_small_talk_response(user_query=corrected_query)
        return {"answer": final_answer, "action": None, "sources": [], "image_url": None, "image_gallery": []}

    async def _handle_calculate(self, corrected_query: str, **kwargs) -> dict:
        """
        🧮 Calculator handler - Hybrid Mode:
        Pure math → Python ตรง | Text+math → AI 70B ช่วยวิเคราะห์
        """
        return await calculator_service.calculate(corrected_query)

    async def _handle_play_music(self, corrected_query: str = "", **kwargs) -> dict:
        """
        🎵 Play music handler - ใช้ corrected_query เป็น search term ตรงๆ
        Frontend ส่ง song name มาใน query แล้ว
        """
        search_query = corrected_query.strip() if corrected_query else ""
        
        # ถ้า query ว่างหรือเป็นคำทั่วไป
        generic_triggers = ["เพลง", "เปิดเพลง", "ฟังเพลง", "music", "song", "อยากฟังเพลง", ""]
        if search_query in generic_triggers:
            return {
                "answer": "ได้เลยค่ะ! อยากฟังเพลงอะไร หรือศิลปินคนไหน บอกน้องน่านได้เลยนะคะ 🎧",
                "action": "PROMPT_FOR_SONG_INPUT",
                "action_payload": {"placeholder": "เช่น น่านเนิบๆ, ปู่จ๋าน ลองไมค์..."},
                "sources": [], "image_url": None, "image_gallery": []
            }
        
        logging.info(f"🎵 [Music] กำลังค้นหาเพลงใน YouTube สำหรับ: '{search_query}'")
        search_results = await youtube_handler_instance.search_music(query=search_query)
        
        if not search_results:
            return {
                "answer": f"ขออภัยค่ะ หาเพลง '{search_query}' ไม่เจอเลยค่ะ ลองชื่ออื่นดูมั้ยคะ?",
                "action": None,
                "action_payload": None,
                "sources": [], "image_url": None, "image_gallery": []
            }
        
        return {
            "answer": f"จัดให้ตามคำขอค่ะ! เพลงเกี่ยวกับ **'{search_query}'**",
            "action": "SHOW_SONG_CHOICES", 
            "action_payload": search_results,
            "sources": [], "image_url": None, "image_gallery": []
        }

    # _handle_system_command ถูกลบออก - เครื่องคิดเลขถูกจัดการโดย frontend widget แล้ว

    async def _handle_informational(
        self, corrected_query: str, entity: Optional[str], sub_queries: List[str], mode: str, 
        turn_count: int = 1, session_id: Optional[str] = None, ai_mode: str = "fast", 
        original_query: str = None,
        interpretation: Dict[str, Any] = None,
        **kwargs
    ) -> dict:
        interpretation = interpretation or kwargs.get("interpretation", {})
        
        unique_queries = interpretation.get("sub_queries") or [corrected_query]
        entity = interpretation.get("entity")
        
        print(f"DEBUG PRINT: _handle_informational CALLED. Entity=[{entity}]")
        logging.info(f"🔎 [DEBUG] _handle_informational Called. Args Entity: {entity}, Kwargs Interpretation Keys: {interpretation.keys()}")
        if entity:
             logging.info(f"🔎 [DEBUG] Entity is present: '{entity}'")
        else:
             logging.info(f"🔎 [DEBUG] Entity is NONE or EMPTY.")
        
        
        # 🛡️ Construct Metadata Filter (Location + Category)
        # 2024-12-16: User requested to DISABLE location/district filtering for simplicity.
        # location_filter = interpretation.get("location_filter", {}) 
        location_filter = {} # Force empty to disable
        
        category = interpretation.get("category")
        
        metadata_filter = location_filter.copy()
        if category:
            metadata_filter["category"] = category
            logging.info(f"🏷️ [Filter] Applying Category Filter: {category}")
        
        # 🆕 [SMART] Always exclude district/province data from search results
        # ไม่แนะนำ "ข้อมูลอำเภอ" หรือ "ข้อมูลภาพรวมจังหวัด" ในผลลัพธ์
        EXCLUDED_CATEGORIES = [
            "ข้อมูลอำเภอ",
            "ข้อมูลภาพรวมจังหวัด",
            "ข้อมูลเศรษฐกิจ"
        ]
        metadata_filter["exclude_categories"] = EXCLUDED_CATEGORIES
        logging.info(f"🚫 [Filter] Excluding categories: {EXCLUDED_CATEGORIES}")

        # 2. ค้นหาข้อมูล (Retrieval)
        # ใช้ Qdrant สำหรับค้นหาด้วยความหมาย (Semantic Search)
        # เราจะค้นหาด้วยทุก sub-query เพื่อความครอบคลุม
        mongo_ids_from_search = []
        qdrant_results_combined = []
        
        logging.info(f"🔎 [RAG] กำลังค้นหาข้อมูล... (Queries: {unique_queries}, Filter: {metadata_filter})")

        # 1.5 SMART FEATURE: Direct Entity Search
        # If Interpreter detected an entity, try to find it directly in DB (Accuracy Boost)
        # This bypasses Semantic Search limitations for specific names.
        found_direct_entity = False
        if entity:
             logging.info(f"🎯 [RAG] Specific Entity Detected: '{entity}' - Attempting Direct DB Lookup...")
             direct_doc = await asyncio.to_thread(self.mongo_manager.get_location_by_title, entity)
             
             if direct_doc:
                 logging.info(f"✅ [RAG] Found Direct Match: {direct_doc.get('title')}")
                 found_direct_entity = True
                 mock_result = {
                    "payload": {
                        "mongo_id": str(direct_doc.get("_id")),
                        "title": direct_doc.get("title"),
                        "summary": direct_doc.get("summary"),
                        "category": direct_doc.get("category"),
                        "slug": direct_doc.get("slug"),
                        "location_data": direct_doc.get("location_data"),
                        "image_urls": direct_doc.get("image_urls", []),
                        "metadata": direct_doc.get("metadata", {}),
                        "is_direct_match": True # Mark as direct match
                    },
                    "score": 1.5 # Boost score above everything else (Typical vector score < 1.0)
                 }
                 qdrant_results_combined.append(mock_result)

        # 🔥 SMART FEATURE: Trending Recommendations for Broad Queries
        # If no specific entity is requested AND no specific filters (except maybe general category),
        # we consider it a "Broad Query" and inject recommended tourist attractions.
        is_broad_query = (not found_direct_entity) and (entity is None) and (not location_filter)
        
        if is_broad_query:
            logging.info("🔥 [RAG] Broad Query Detected! Fetching Recommended Attractions...")
            try:
                # 🆕 ใช้ get_recommended_attractions แทน get_trending_locations
                # เพื่อให้แนะนำสถานที่ท่องเที่ยว ไม่ใช่อำเภอ
                recommended_docs = await asyncio.to_thread(
                    self.mongo_manager.get_recommended_attractions, 
                    limit=5
                )
                
                if recommended_docs:
                    logging.info(f"🎯 [Recommended] Found {len(recommended_docs)} attractions")
                    for doc in recommended_docs:
                        logging.info(f"   - {doc.get('title')} ({doc.get('category')})")
                        mock_result = {
                            "payload": {
                                "mongo_id": str(doc.get("_id")),
                                "title": doc.get("title"),
                                "summary": doc.get("summary"),
                                "category": doc.get("category"),
                                "slug": doc.get("slug"),
                                "location_data": doc.get("location_data"),
                                "image_urls": doc.get("image_urls", []),
                                "metadata": doc.get("metadata", {}),
                                "is_recommended": True  # Mark as recommended
                            },
                            "score": 0.85  # Good score but let semantic match win if very specific
                        }
                        qdrant_results_combined.append(mock_result)
                else:
                    logging.warning("⚠️ [Recommended] No attractions found, falling back to semantic search")
            except Exception as e:
                logging.error(f"❌ [RAG] Error fetching recommended attractions: {e}")

        for q in unique_queries:
            # Pass metadata_filter to search_similar
            qdrant_results = await self.qdrant_manager.search_similar(
                query_text=q, 
                top_k=settings.QDRANT_TOP_K,
                metadata_filter=metadata_filter # 🆕 Apply Merged Filter
            )
            qdrant_results_combined.extend(qdrant_results)
            for res in qdrant_results:
                if res.payload and res.payload.get("mongo_id"):
                    mongo_ids_from_search.append(res.payload.get("mongo_id"))

        # [แผนสำรอง] หาก Qdrant ไม่พบผลลัพธ์ (หรือระบบล่ม) ให้ลองค้นหาข้อความใน MongoDB แทน
        if not qdrant_results_combined:
            logging.info("⚠️ [RAG] Qdrant ไม่พบผลลัพธ์ กำลังลองค้นหาด้วยข้อความใน MongoDB...")
            # ใช้ entity ถ้ามี มิฉะนั้นใช้ corrected_query
            search_term = entity if entity else corrected_query
            
            # TODO: Improve MongoDB Fallback to support filter (Optional for now)
            logging.info(f"⚠️ [RAG] Qdrant ไม่พบผลลัพธ์ กำลังลองค้นหาด้วยข้อความใน MongoDB ด้วยคำว่า: '{search_term}'")
            mongo_results = await asyncio.to_thread(self.mongo_manager.get_location_by_title, search_term)
            if mongo_results:
                # แปลงผลลัพธ์จาก MongoDB ให้อยู่ในรูปแบบคล้ายกับ payload ของ Qdrant
                # หมายเหตุ: ผลลัพธ์ของ Qdrant มักจะมี 'payload' และ 'score'
                # เราจะสร้างข้อมูลจำลองแบบ Qdrant เพื่อความสม่ำเสมอ
                mock_qdrant_result = {
                    "payload": {
                        "mongo_id": str(mongo_results.get("_id")), # ตรวจสอบให้แน่ใจว่าได้รวม _id ไว้สำหรับการดึงข้อมูลในภายหลัง
                        "title": mongo_results.get("title"),
                        "summary": mongo_results.get("summary"),
                        "category": mongo_results.get("category"),
                        "slug": mongo_results.get("slug"),
                        "location_data": mongo_results.get("location_data"),
                        "image_urls": mongo_results.get("image_urls", []),
                        "metadata": mongo_results.get("metadata", {})
                    },
                    "score": 1.0 # ให้คะแนนสูงสำหรับการค้นหาสำรอง
                }
                qdrant_results_combined.append(mock_qdrant_result)
                mongo_ids_from_search.append(str(mongo_results.get("_id")))
                logging.info(f"✅ [RAG] พบผลลัพธ์สำรองใน MongoDB: {mongo_results.get('title')}")


        unique_ids = list(dict.fromkeys(mongo_ids_from_search))
        
        # Capture Trending IDs to re-apply metadata later
        # FIX: Check if res is object (ScoredPoint) or dict
        def get_payload(res):
            if hasattr(res, 'payload'): return res.payload
            if isinstance(res, dict): return res.get('payload', {})
            return {}

        trending_ids = {get_payload(res).get('mongo_id') for res in qdrant_results_combined 
                        if get_payload(res).get('is_trending')}
                        
        direct_match_ids = {get_payload(res).get('mongo_id') for res in qdrant_results_combined 
                            if get_payload(res).get('is_direct_match')}

        # Re-populate unique_ids from ALL results including trending
        all_mongo_ids = [get_payload(res).get('mongo_id') for res in qdrant_results_combined 
                         if get_payload(res).get('mongo_id')]
        unique_ids = list(dict.fromkeys(all_mongo_ids))

        if not unique_ids:
            return {"answer": "ขออภัยค่ะ ไม่พบข้อมูลที่เกี่ยวข้องในระบบ", "action": None, "sources": [], "image_url": None, "image_gallery": []}

        retrieved_docs = await asyncio.to_thread(self.mongo_manager.get_locations_by_ids, unique_ids)
        if not retrieved_docs:
            return {"answer": "พบข้อมูลแต่ดึงรายละเอียดไม่ได้ค่ะ", "action": None, "sources": [], "image_url": None, "image_gallery": []}
            
        # 🆕 Re-inject Trending & Score Info
        # Note: We can't rely on Qdrant scores for trending items (they are mock scores)
        # We just mark them.
        for doc in retrieved_docs:
            doc_id = str(doc.get('_id'))
            if doc_id in trending_ids:
                doc['is_trending'] = True
                doc['title'] = f"🔥 {doc.get('title')}" # Hack: Add fire to title for Reranker context too
            if doc_id in direct_match_ids:
                doc['is_direct_match'] = True
                doc['title'] = f"🎯 {doc.get('title')}" # Hack: Add target to title
        
        # TODO: Consider synthetic doc creation - we might need to update it
        # from core.ai_models.utils.summarizer import create_synthetic_document # Removed as it's imported globally now
        docs_with_synthetic = await asyncio.to_thread(lambda docs: [(doc, create_synthetic_document(doc)) for doc in docs], retrieved_docs)
        # 🔄 [RERANKING] ขั้นตอนการจัดลำดับใหม่
        # จับคู่ (User Query, Document) เพื่อให้โมเดล Reranker ให้คะแนนความเกี่ยวข้อง 
        sentence_pairs = [[corrected_query, synthetic_doc] for doc, synthetic_doc in docs_with_synthetic]
        
        # ให้คะแนนความเหมือน (Score) ยิ่งเยอะยิ่งเกี่ยวข้องกันมาก
        scores = await asyncio.to_thread(self.reranker.predict, sentence_pairs, show_progress_bar=False)
        
        # �️ [Score Boosting] ดันคะแนน Trending/Direct ให้ชนะ Semantic เสมอ
        final_scores = []
        for score, (doc, _) in zip(scores, docs_with_synthetic):
            boosted_score = float(score)
            if doc.get('is_direct_match'):
                boosted_score = max(boosted_score, 0.99) # Direct Match = Almost 1.0
            elif doc.get('is_trending'):
                # Trending items get a floor, but can go higher if relevant
                boosted_score = max(boosted_score, 0.85) 
            final_scores.append(boosted_score)
        
        # �🔍 [Debug Log] แสดงคะแนน Reranking ของแต่ละเอกสาร
        logging.info(f"📊 [Reranking] กำลังจัดลำดับเอกสาร {len(final_scores)} รายการ...")
        for i, (score, (doc, _)) in enumerate(zip(final_scores, docs_with_synthetic)):
            logging.info(f"   🔹 เอกสาร: {doc.get('title')} | คะแนน: {score:.4f} | Trending: {doc.get('is_trending', False)} | Direct: {doc.get('is_direct_match', False)}")

        # เรียงลำดับใหม่ตามคะแนน Boosted (มากไปน้อย)
        reranked_results = sorted(zip(final_scores, docs_with_synthetic), key=lambda x: x[0], reverse=True)
        
        # 🔍 [Debug Log] ผลลัพธ์หลังจัดลำดับ (Top 3)
        logging.info(f"🏆 [Reranking] 3 อันดับแรกหลังจัดลำดับใหม่:")
        for i, (score, (doc, _)) in enumerate(reranked_results[:3]):
             logging.info(f"   🥇 #{i+1}: {doc.get('title')} (คะแนน: {score:.4f})")

        # เลือกเฉพาะเอกสารที่มีคะแนนสูงสุด Top K อันดับแรก
        top_k = settings.TOP_K_RERANK_VOICE if mode == "voice" else settings.TOP_K_RERANK_TEXT
        
        # 🛡️ [Self-Correction] Confidence Check
        is_low_confidence = False
        if reranked_results:
            top_score = reranked_results[0][0]
            # Trust Trending AND Direct Matches
            has_trusted_source = any(d.get('is_trending') or d.get('is_direct_match') for _, (d, _) in reranked_results[:top_k])
            
            if top_score < settings.RAG_CONFIDENCE_THRESHOLD and not has_trusted_source:
                # Only flag low confidence if NO trusted items are in top K
                # (Trending/Direct items are high value regardless of semantic score)
                is_low_confidence = True
                logging.warning(f"⚠️ [Low Confidence] คะแนนสูงสุด ({top_score:.4f}) ต่ำกว่าเกณฑ์ ({settings.RAG_CONFIDENCE_THRESHOLD})")
                
                # 📝 [Log Quality] บันทึกเหตุการณ์นี้เพื่อนำไปปรับปรุง (Active Learning)
                logging.info(f"📉 [Quality Log] Triggered Low Confidence for query: '{corrected_query}'")
        
        final_docs = [doc for score, (doc, _) in reranked_results[:top_k]]
        
        context_str = ""
        if final_docs:
            context_parts = []
            for i, doc in enumerate(final_docs, 1):
                doc_text = create_synthetic_document(doc)
                if doc.get('is_trending'):
                    doc_text = f"🔥 [POPULAR/TRENDING] นี่ยอดนิยมในช่วงนี้: {doc_text}"
                context_parts.append(f"[Document {i}]\nTitle: {doc.get('title')}\nInfo: {doc_text}")
            context_str = "\n\n----------------\n\n".join(context_parts)

        history = []
        if session_id:
            session = await self.session_manager.get_session(session_id)
            history = session.get("history", [])

        prompt_dict = self.prompt_engine.build_rag_prompt(
            user_query=original_query or corrected_query, # 🆕 ใช้คำถามเริ่มต้นของผู้ใช้เพื่อตรวจจับภาษาได้ถูกต้อง 
            context=context_str, 
            history=history,
            ai_mode=ai_mode,  # 🆕 ส่ง mode ไปเลือก prompt ที่เหมาะสม
            is_low_confidence=is_low_confidence # 🛡️ [Self-Correction]
        )
        
        messages = [
            {"role": "system", "content": prompt_dict["system"]},
            {"role": "user", "content": prompt_dict["user"]}
        ]

        logging.info(f"🤖 [LLM] กำลังใช้โหมด AI: {ai_mode}")
        
        if ai_mode == "detailed":
            # ใช้ Gemini สำหรับคำตอบที่ละเอียด 
            raw_answer = await get_gemini_response(
                user_query=prompt_dict["user"],
                system_prompt=prompt_dict["system"],
                max_tokens=8192
            )
        else:
            # ใช้ Groq/Llama สำหรับการตอบกลับที่รวดเร็ว
            try:
                raw_answer = await get_groq_response(
                    messages=messages,
                    model_name=settings.GROQ_LLAMA_MODEL
                )
            except Exception as e:
                logging.error(f"⚠️ [Groq] การสร้างข้อความล้มเหลว: {e} กำลังเปลี่ยนไปใช้ Gemini...")
                # สลับไปใช้ Gemini โดยอัตโนมัติ
                raw_answer = await get_gemini_response(
                    user_query=prompt_dict["user"],
                    system_prompt=prompt_dict["system"],
                    max_tokens=8192
                )
        
        final_answer_with_images = await self.image_service.inject_images_into_text(raw_answer)
        
        docs_to_show = final_docs[:5]
        prepared_data = self._prepare_source_and_image_data(docs_to_show)
        static_gallery = prepared_data["image_gallery"]
        
        if len(static_gallery) < settings.IMAGE_FALLBACK_THRESHOLD and final_docs:
            search_q = f"{final_docs[0].get('title')} จังหวัดน่าน" 
            try:
                google_imgs = await image_search_tool_instance.get_image_urls(search_q, max_results=settings.GOOGLE_IMAGE_MAX_RESULTS)
                for url in google_imgs:
                    if url not in static_gallery: static_gallery.append(url)
            except Exception as e:
                logging.error(f"❌ การค้นหารูปภาพ Google ล้มเหลว: {e}")

        return {
            "answer": final_answer_with_images,
            "action": None,
            "image_url": None, 
            "image_gallery": static_gallery[:settings.FINAL_GALLERY_IMAGE_LIMIT],
            "sources": prepared_data["source_info"],
            "_primary_topic": final_docs[0].get("title") if final_docs else None # ส่ง Topic กลับไปบันทึก State
        }

    async def get_navigation_list(self, user_lat: float = None, user_lon: float = None) -> List[Dict[str, Any]]:
        try:
            collection = self.mongo_manager.get_collection("nan_locations")
            if collection is None:
                logging.warning("⚠️ [NavList] ไม่สามารถเชื่อมต่อ MongoDB ได้ กำลังส่งคืนข้อมูลจำลอง")
                return []

            docs = await asyncio.to_thread(lambda: list(collection.find(
                {"doc_type": "Location"}, 
                {"title":1, "slug":1, "topic":1, "summary":1, "category":1, "location_data":1, "metadata":1, "_id":0}
            )))
            
            if user_lat and user_lon:
                docs = self.nav_service.sort_locations_by_distance(docs, user_lat, user_lon)
            
            for doc in docs:
                imgs = self.image_service.get_location_images(doc)
                doc["image_urls"] = [imgs[0]] if imgs else []
            
            return docs
        except Exception as e:
            logging.error(f"❌ [NavList] เกิดข้อผิดพลาด: {e}")
            return []

    async def handle_get_directions(self, entity_slug: str, user_lat: float = None, user_lon: float = None) -> dict:
        return await self.nav_service.handle_get_directions(entity_slug, user_lat, user_lon)
    
    async def answer_query(self, query: str, mode: str = "text", session_id: Optional[str] = None, ai_mode: str = "fast", frontend_intent: str = None, slug: Optional[str] = None, entity_query: Optional[str] = None, **kwargs) -> dict:
        """
        ai_mode: 'fast' = Llama/Groq, 'detailed' = Gemini
        frontend_intent: 'GENERAL' | 'MUSIC' | 'NAVIGATION' | 'FAQ' (จาก Frontend)
        """
        session_data = await self.session_manager.get_session(session_id)
        current_turn = session_data.get("turn_count", 0) + 1
        history = session_data.get("history", []) 
        
        if session_id and session_data.get("awaiting") == "analytics_origin_or_topic":
            self.session_manager.collection.update_one({"session_id": session_id}, {"$unset": {"awaiting": ""}})
            return await self.analytics_handler.handle_analytics_response(query, session_id, mode)

        start_time = time.perf_counter() # ⏱️ Start Timer

        logging.info(f"🔄 [Session] ID: {session_id} | รอบที่: {current_turn} | โหมด AI: {ai_mode} | เจตนาจาก Frontend: {frontend_intent}")

        # 🆕 ใช้ frontend_intent โดยตรง - ไม่ต้องเรียก LLM วิเคราะห์เจตนา
        # Note: "LINE" frontend_intent should use LLM analysis to detect music/navigation intents
        if frontend_intent and frontend_intent not in ["GENERAL", "LINE", None]:
            # Frontend บอก intent มาแล้ว ใช้เลย!
            intent = self._map_frontend_intent(frontend_intent)
            corrected_query = query
            entity = None  # จะหาจาก query หรือ Qdrant search
            
            # 🚀 [Direct Bypass] ถ้ามี intent NAVIGATE_TO + slug/entity_query ให้ทำงานเลย!
            if intent == "NAVIGATE_TO":
                 target_entity = slug or entity_query or query
                 if target_entity:
                     logging.info(f"🏎️ [Quick Nav] ข้าม Logic เพื่อนำทางไปยัง: '{target_entity}'")
                     return await self.handle_get_directions(entity_slug=target_entity)

            logging.info(f"🚀 [Intent] ใช้เจตนาจาก FRONTEND: {intent}")
            interpretation = {"intent": intent, "corrected_query": query, "entity": entity, "is_complex": False, "sub_queries": [query], "location_filter": {}}
        else:
            # ใช้ LLM วิเคราะห์เจตนาและ Filter (Dynamic)
            logging.info(f"🧠 [Router] เรียกใช้ Query Interpreter เพื่อวิเคราะห์เจตนาและหา Location Filter...")
            interpretation = await self.query_interpreter.interpret_and_route(query)
            intent = interpretation.get("intent", "INFORMATIONAL")
            corrected_query = interpretation.get("corrected_query", query)
            entity = interpretation.get("entity")
            # Note: location_filter is inside interpretation and handled in _handle_informational via interpretation object if passed, 
            # BUT _handle_informational signature expects separate args currently? 
            # Wait, verify _handle_informational signature again.
            
            logging.info(f"🚦 เจตนา: {intent} | คำค้น: {corrected_query} | Location Filter: {interpretation.get('location_filter')}")
        
        import sys
        
        navigation_keywords = ["นำทาง", "เส้นทาง", "พาไป", "ขอทาง", "ไปยัง", "ไปวัด", "ไปที่"]
        is_nav_request = any(kw in corrected_query for kw in navigation_keywords)
        
        if is_nav_request:
            target_entity = entity
            
            if not target_entity and session_id:
                last_topic = await self.session_manager.get_last_topic(session_id)
                if last_topic:
                    logging.info(f"🧠 [Memory] ผู้ใช้ไม่ได้ระบุสถานที่ สันนิษฐานว่าเป็นหัวข้อล่าสุด: '{last_topic}'")
                    target_entity = last_topic
                else:
                    return {"answer": "ได้เลยค่ะ! แต่ช่วยบอกน้องน่านหน่อยได้ไหมคะว่าจะให้ **นำทางไปที่ไหน?** 😊", "action": None, "sources": [], "image_url": None}

            if target_entity:
                logging.info(f"🗺️ [Smart Router] เปลี่ยนไปใช้ตัวจัดการการนำทางสำหรับ: '{target_entity}'")
                return await self.handle_get_directions(
                    entity_slug=target_entity, 
                    user_lat=kwargs.get('user_lat', 0.0),
                    user_lon=kwargs.get('user_lon', 0.0)
                )
        
        # 🧮 [Calculator Detection] ตรวจจับคำถามคณิตศาสตร์ก่อน (Hybrid Mode)
        # Pure math → Python ตรง | Text+math → AI 70B ช่วยวิเคราะห์
        if calculator_service.is_calculator_query(query):
            logging.info(f"🧮 [Calculator] ตรวจพบโจทย์คณิตศาสตร์: '{query}'")
            return await calculator_service.calculate(query)
        
        handler_map = {
            "WELCOME_GREETING": self._handle_welcome_flow,
            "SMALL_TALK": self._handle_small_talk,  # 👈 [จุดแยก] ถ้าเป็น SMALL_TALK ไปใช้โมเดลเล็ก (Llama 8B)
            "PLAY_MUSIC": self._handle_play_music,
            "CALCULATE": self._handle_calculate,  # 🧮 เครื่องคิดเลข Python
            "INFORMATIONAL": self._handle_informational,
        }
        handler = handler_map.get(intent, self._handle_informational)
        
        response = await handler(
            corrected_query=corrected_query,
            entity=entity,
            is_complex=interpretation.get("is_complex", False),
            sub_queries=interpretation.get("sub_queries", []),
            mode=mode,
            session_id=session_id,
            turn_count=current_turn,
            ai_mode=ai_mode,   # 🆕 ส่ง ai_mode ไปยัง handlers
            interpretation=interpretation, # 🆕 Send full interpretation object (with location_filter)
            original_query=query, # 🆕 ส่งคำถามต้นฉบับไปด้วย
            **kwargs
        )

        end_time = time.perf_counter()
        processing_time = round(end_time - start_time, 2)
        response["processing_time"] = processing_time
        
        logging.info(f"⏱️ [Performance] Total Processing Time: {processing_time}s")


        if session_id:
            primary_topic = response.pop("_primary_topic", None) 
            await self.session_manager.update_turn(
                session_id, 
                user_query=query, 
                ai_response=response.get("answer", ""), 
                topic=primary_topic
            )
            
            # 🚀 [Analytics] บันทึกเหตุการณ์ความสนใจหากพบหัวข้อ
            if primary_topic:
                await self.analytics_handler.log_interest_event(session_id, primary_topic, query)
            
        return response