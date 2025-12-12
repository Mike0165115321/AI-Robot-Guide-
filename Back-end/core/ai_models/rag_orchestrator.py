import asyncio
import logging
import os
import random
import re
import math
import urllib.parse
import json 
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
from core.tools.system_tool import system_tool_instance
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
        logging.info("⚙️  RAG Orchestrator (Refactored V8.1) is initializing...")
        self.mongo_manager = mongo_manager
        self.qdrant_manager = qdrant_manager
        self.query_interpreter = query_interpreter

        # ✅ เรียกใช้ Services ใหม่
        self.session_manager = SessionManager(mongo_manager)
        self.prompt_engine = PromptEngine()
        self.nav_service = NavigationService(mongo_manager, self.prompt_engine)
        self.image_service = ImageService(mongo_manager)

        self.reranker_model_name = settings.RERANKER_MODEL_NAME
        self.device = settings.DEVICE

        logging.info(f"🔄 Loading Re-ranker Model ('{self.reranker_model_name}' on '{self.device}')...")
        self.reranker = CrossEncoder(self.reranker_model_name, device=self.device)
        logging.info("✅ Re-ranker Model loaded.")

        self.log_collection = self.mongo_manager.get_collection("query_logs")
        
        self.analytics_handler = AnalyticsHandler(
            mongo_manager=self.mongo_manager,
            query_interpreter=self.query_interpreter,
            orchestrator_callback=self.answer_query
        )
        
        logging.info("✅ RAG Orchestrator is ready.")

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
        # ตั้งค่า session ให้รอรับข้อมูล analytics (ถ้ามี session_id)
        if session_id:
            try:
                self.session_manager.collection.update_one(
                    {"session_id": session_id},
                    {"$set": {"awaiting": "analytics_origin_or_topic", "last_active": datetime.now(timezone.utc)}},
                    upsert=True
                )
            except Exception as e:
                logging.error(f"Error updating welcome state: {e}")
        
        # ข้อความทักทายแบบเป็นกันเอง
        return {
            "answer": "สวัสดีค่ะ เราเป็น AI ไกด์จากน่านค่ะ ปรึกษาได้ทุกเรื่องเรามีข้อมูลสถานที่ท่องเที่ยวจังหวัดน่านแน่นๆเลย เลยอยากรู้ว่าเธอมาจากที่ไหน ไม่ต้องบอกแบบละเอียดก็ได้นะคะ แค่จังหวัดหรือประเทศก็พอค่ะ",
            "action": "AWAITING_ANALYTICS_DATA",
            "action_payload": None, "image_url": None, "image_gallery": [], "sources": [],
        }

    async def _handle_small_talk(self, corrected_query: str, **kwargs) -> dict:
        final_answer = await get_small_talk_response(user_query=corrected_query)
        return {"answer": final_answer, "action": None, "sources": [], "image_url": None, "image_gallery": []}

    async def _handle_play_music(self, entity: Optional[str], **kwargs) -> dict:
        generic_triggers = ["เพลง", "เปิดเพลง", "ฟังเพลง", "music", "song", "play music", "ร้องเพลง"]
        if not entity or entity.strip() in generic_triggers:
            return {
                "answer": "ได้เลยค่ะ! อยากฟังเพลงอะไร หรือศิลปินคนไหน บอกน้องน่านได้เลยนะคะ 🎧",
                "action": "PROMPT_FOR_SONG_INPUT",
                "action_payload": {"placeholder": "เช่น น่านเนิบๆ, ปู่จ๋าน ลองไมค์..."},
                "sources": [], "image_url": None, "image_gallery": []
            }
        search_query = entity
        search_results = await youtube_handler_instance.search_music(query=search_query)
        if not search_results:
            return {
                "answer": f"ขออภัยค่ะ หาเพลง '{search_query}' ไม่เจอเลยค่ะ ลองชื่ออื่นดูมั้ยคะ?",
                "action": "PROMPT_FOR_SONG_INPUT",
                "action_payload": {"placeholder": "ลองค้นหาใหม่..."},
                "sources": [], "image_url": None, "image_gallery": []
            }
        return {
            "answer": f"จัดให้ตามคำขอค่ะ! เพลงเกี่ยวกับ **'{search_query}'**",
            "action": "SHOW_SONG_CHOICES", 
            "action_payload": search_results,
            "sources": [], "image_url": None, "image_gallery": []
        }

    async def _handle_system_command(self, entity: Optional[str], **kwargs) -> dict:
        if not entity: return {"answer": "ไม่แน่ใจว่าให้เปิดแอปอะไรคะ?", "action": None, "sources": [], "image_url": None}
        result_text = await asyncio.to_thread(system_tool_instance.launch, entity)
        return {"answer": result_text, "action": None, "sources": [], "image_url": None, "image_gallery": []}

    async def _handle_informational(
        self, corrected_query: str, entity: Optional[str], sub_queries: List[str], mode: str, 
        turn_count: int = 1, session_id: Optional[str] = None, ai_mode: str = "fast", **kwargs
    ) -> dict:
        search_queries = [corrected_query] + sub_queries
        if entity:
            search_queries.append(entity)
            search_queries.append(f"ข้อมูลทั่วไป {entity}")
            search_queries.append(f"{entity} จังหวัดน่าน")
        
        unique_queries = list(set([q for q in search_queries if q.strip()]))
        logging.info(f"🛰️ [RAG] Searching for: {unique_queries}")
        
        mongo_ids_from_search = []
        qdrant_results_combined = []
        for q in unique_queries:
            qdrant_results = await self.qdrant_manager.search_similar(query_text=q, top_k=settings.QDRANT_TOP_K)
            qdrant_results_combined.extend(qdrant_results)
            for res in qdrant_results:
                if res.payload and res.payload.get("mongo_id"):
                    mongo_ids_from_search.append(res.payload.get("mongo_id"))

        # [Fallback] If Qdrant returns no results (or is down), try MongoDB text search
        if not qdrant_results_combined:
            logging.info("⚠️ [RAG] Qdrant returned no results. Trying MongoDB text search fallback...")
            # Use entity if available, otherwise corrected_query
            search_term = entity if entity else corrected_query
            logging.info(f"⚠️ [RAG] Qdrant returned no results. Trying MongoDB text search fallback with: '{search_term}'")
            mongo_results = await asyncio.to_thread(self.mongo_manager.get_location_by_title, search_term)
            if mongo_results:
                # Convert MongoDB result to a format similar to Qdrant payload
                # Note: Qdrant results typically have a 'payload' and 'score'
                # We'll create a mock Qdrant-like result for consistency
                mock_qdrant_result = {
                    "payload": {
                        "mongo_id": str(mongo_results.get("_id")), # Ensure _id is included for later retrieval
                        "title": mongo_results.get("title"),
                        "summary": mongo_results.get("summary"),
                        "category": mongo_results.get("category"),
                        "slug": mongo_results.get("slug"),
                        "location_data": mongo_results.get("location_data"),
                        "image_urls": mongo_results.get("image_urls", []),
                        "metadata": mongo_results.get("metadata", {})
                    },
                    "score": 1.0 # Assign a high score for fallback
                }
                qdrant_results_combined.append(mock_qdrant_result)
                mongo_ids_from_search.append(str(mongo_results.get("_id")))
                logging.info(f"✅ [RAG] Found fallback result in MongoDB: {mongo_results.get('title')}")

        unique_ids = list(dict.fromkeys(mongo_ids_from_search))
        if not unique_ids:
            return {"answer": "ขออภัยค่ะ ไม่พบข้อมูลที่เกี่ยวข้องในระบบ", "action": None, "sources": [], "image_url": None, "image_gallery": []}

        retrieved_docs = await asyncio.to_thread(self.mongo_manager.get_locations_by_ids, unique_ids)
        if not retrieved_docs:
            return {"answer": "พบข้อมูลแต่ดึงรายละเอียดไม่ได้ค่ะ", "action": None, "sources": [], "image_url": None, "image_gallery": []}

        docs_with_synthetic = await asyncio.to_thread(lambda docs: [(doc, create_synthetic_document(doc)) for doc in docs], retrieved_docs)
        sentence_pairs = [[corrected_query, synthetic_doc] for doc, synthetic_doc in docs_with_synthetic]
        
        scores = await asyncio.to_thread(self.reranker.predict, sentence_pairs, show_progress_bar=False)
        reranked_results = sorted(zip(scores, docs_with_synthetic), key=lambda x: x[0], reverse=True)

        top_k = settings.TOP_K_RERANK_VOICE if mode == "voice" else settings.TOP_K_RERANK_TEXT
        final_docs = [doc for score, (doc, _) in reranked_results[:top_k]]
        
        context_str = ""
        if final_docs:
            context_parts = []
            for i, doc in enumerate(final_docs, 1):
                doc_text = create_synthetic_document(doc)
                context_parts.append(f"[Document {i}]\nTitle: {doc.get('title')}\nInfo: {doc_text}")
            context_str = "\n\n----------------\n\n".join(context_parts)

        history = []
        if session_id:
            session = await self.session_manager.get_session(session_id)
            history = session.get("history", [])

        prompt_dict = self.prompt_engine.build_rag_prompt(
            user_query=corrected_query, 
            context=context_str, 
            history=history,
            ai_mode=ai_mode  # 🆕 ส่ง mode ไปเลือก prompt ที่เหมาะสม
        )
        
        messages = [
            {"role": "system", "content": prompt_dict["system"]},
            {"role": "user", "content": prompt_dict["user"]}
        ]

        # 🆕 Use ai_mode to select model: detailed = Gemini, fast = Groq/Llama
        logging.info(f"🤖 [LLM] Using AI Mode: {ai_mode}")
        
        if ai_mode == "detailed":
            # Use Gemini for detailed responses
            raw_answer = await get_gemini_response(
                user_query=prompt_dict["user"],
                system_prompt=prompt_dict["system"],
                max_tokens=8192
            )
        else:
            # Use Groq/Llama for fast responses
            raw_answer = await get_groq_response(
                messages=messages,
                model_name=settings.GROQ_LLAMA_MODEL
            )
        
        final_answer_with_images = await self.image_service.inject_images_into_text(raw_answer)
        
        docs_to_show = final_docs[:3]
        prepared_data = self._prepare_source_and_image_data(docs_to_show)
        static_gallery = prepared_data["image_gallery"]
        
        if len(static_gallery) < settings.IMAGE_FALLBACK_THRESHOLD and final_docs:
            search_q = f"{final_docs[0].get('title')} จังหวัดน่าน" 
            try:
                google_imgs = await image_search_tool_instance.get_image_urls(search_q, max_results=settings.GOOGLE_IMAGE_MAX_RESULTS)
                for url in google_imgs:
                    if url not in static_gallery: static_gallery.append(url)
            except Exception as e:
                logging.error(f"❌ Google Image Search failed: {e}")

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
                logging.warning("⚠️ [NavList] MongoDB not available. Returning mock data.")
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
            logging.error(f"❌ [NavList] Error: {e}")
            return []

    async def handle_get_directions(self, entity_slug: str, user_lat: float = None, user_lon: float = None) -> dict:
        return await self.nav_service.handle_get_directions(entity_slug, user_lat, user_lon)
    
    async def answer_query(self, query: str, mode: str = "text", session_id: Optional[str] = None, ai_mode: str = "fast", **kwargs) -> dict:
        """
        ai_mode: 'fast' = Llama/Groq, 'detailed' = Gemini
        """
        session_data = await self.session_manager.get_session(session_id)
        current_turn = session_data.get("turn_count", 0) + 1
        history = session_data.get("history", []) 
        
        if session_id and session_data.get("awaiting") == "analytics_origin_or_topic":
            self.session_manager.collection.update_one({"session_id": session_id}, {"$unset": {"awaiting": ""}})
            return await self.analytics_handler.handle_analytics_response(query, session_id, mode)

        logging.info(f"🔄 [Session] ID: {session_id} | Turn: {current_turn} | AI Mode: {ai_mode}")

        try:
            interpretation = await self.query_interpreter.interpret_and_route(query)
        except Exception as e:
            logging.error(f"❌ Router Error: {e}")
            return {"answer": "ขออภัยค่ะ ระบบขัดข้องชั่วคราว", "action": None, "sources": [], "image_url": None}
        
        intent = interpretation.get("intent", "INFORMATIONAL")
        corrected_query = interpretation.get("corrected_query", query)
        entity = interpretation.get("entity")
        
        logging.info(f"🚦 Intent: {intent} | Query: {query} | Entity: {entity}")

        navigation_keywords = ["นำทาง", "เส้นทาง", "พาไป", "ขอทาง", "ไปยัง", "ไปวัด", "ไปที่"]
        is_nav_request = any(kw in corrected_query for kw in navigation_keywords)
        
        if is_nav_request:
            target_entity = entity
            
            if not target_entity and session_id:
                last_topic = await self.session_manager.get_last_topic(session_id)
                if last_topic:
                    logging.info(f"🧠 [Memory] User didn't say where, assuming last topic: '{last_topic}'")
                    target_entity = last_topic
                else:
                    return {"answer": "ได้เลยค่ะ! แต่ช่วยบอกน้องน่านหน่อยได้ไหมคะว่าจะให้ **นำทางไปที่ไหน?** 😊", "action": None, "sources": [], "image_url": None}

            if target_entity:
                logging.info(f"🗺️ [Smart Router] Switching to Navigation Handler for: '{target_entity}'")
                return await self.handle_get_directions(
                    entity_slug=target_entity, 
                    user_lat=kwargs.get('user_lat', 0.0),
                    user_lon=kwargs.get('user_lon', 0.0)
                )
        handler_map = {
            "WELCOME_GREETING": self._handle_welcome_flow,
            "SMALL_TALK": self._handle_small_talk,
            "PLAY_MUSIC": self._handle_play_music,
            "INFORMATIONAL": self._handle_informational,
            "SYSTEM_COMMAND": self._handle_system_command,
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
            **kwargs
        )

        if session_id:
            primary_topic = response.pop("_primary_topic", None) 
            await self.session_manager.update_turn(
                session_id, 
                user_query=query, 
                ai_response=response.get("answer", ""), 
                topic=primary_topic
            )
            
            # 🚀 [Analytics] Log Interest Event if topic is found
            if primary_topic:
                await self.analytics_handler.log_interest_event(session_id, primary_topic, query)
            
        return response