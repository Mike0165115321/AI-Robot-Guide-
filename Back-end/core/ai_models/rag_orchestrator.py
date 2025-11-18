# /Back-end/core/ai_models/rag_orchestrator.py (V6.4 - Robust Source Filter)

import asyncio
import logging
import os
import random
import re
import urllib.parse
import json 
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from sentence_transformers import CrossEncoder

from core.ai_models.llm_handler import (
    get_groq_rag_response_async,
    get_insights_from_logs,
    get_llama_response_direct_async,
)
from core.ai_models.query_interpreter import QueryInterpreter
from core.ai_models.youtube_handler import youtube_handler_instance
from core.config import settings
from .handlers.analytics_handler import AnalyticsHandler
from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from core.tools.image_search_tool import image_search_tool_instance
from core.tools.system_tool import system_tool_instance
from utils.helper_functions import create_synthetic_document

BACKEND_ROOT = Path(__file__).resolve().parent.parent.parent
IMAGE_DIR = BACKEND_ROOT / "static" / "images"

logging.info(f"📂 [RAGOrchestrator] IMAGE_DIR set to: {IMAGE_DIR.absolute()}")


class RAGOrchestrator:
    def __init__(
        self,
        mongo_manager: MongoDBManager,
        qdrant_manager: QdrantManager,
        query_interpreter: QueryInterpreter,
    ):
        logging.info("⚙️  RAG Orchestrator (V6.4 - Robust Source Filter) is initializing...")
        self.mongo_manager = mongo_manager
        self.qdrant_manager = qdrant_manager
        self.query_interpreter = query_interpreter

        self.reranker_model_name = settings.RERANKER_MODEL_NAME
        self.device = settings.DEVICE

        logging.info(f"🔄 Loading Re-ranker Model ('{self.reranker_model_name}' on '{self.device}')...")
        self.reranker = CrossEncoder(self.reranker_model_name, device=self.device)
        logging.info("✅ Re-ranker Model loaded.")

        self.log_collection = self.mongo_manager.get_collection("query_logs")
        if self.log_collection is not None:
            logging.info("📝 Analytics logging is enabled.")

        self.session_states: Dict[str, Dict[str, Any]] = {}
        logging.info("🧠 [Session] In-memory session state initialized.")

        self.analytics_handler = AnalyticsHandler(
            mongo_manager=self.mongo_manager,
            query_interpreter=self.query_interpreter,
            orchestrator_callback=self.answer_query
        )
        self.all_image_files: List[str] = []
        self.prefixed_image_map: Dict[str, List[str]] = {}

        logging.info(f"🏞️ Scanning image directory: {IMAGE_DIR.absolute()}")
        self.image_dir_exists = IMAGE_DIR.is_dir()

        if self.image_dir_exists:
            try:
                for f in IMAGE_DIR.iterdir():
                    if f.is_file() and f.suffix.lower() in (".jpg", ".jpeg", ".png", ".webp"):
                        file_path_str = f"/static/images/{f.name}"
                        self.all_image_files.append(file_path_str)
                        prefix = ""
                        if "-" in f.name:
                            prefix = f.name.rsplit("-", 1)[0] + "-"
                        elif "_" in f.name:
                            prefix = f.name.split("_")[0] + "_"

                        if prefix:
                            if prefix not in self.prefixed_image_map:
                                self.prefixed_image_map[prefix] = []
                            self.prefixed_image_map[prefix].append(file_path_str)

                logging.info(
                    f"✅ Image Cache: Found {len(self.all_image_files)} images and {len(self.prefixed_image_map)} prefixes."
                )
            except Exception as e:
                logging.error(f"❌ Error scanning image directory: {e}")

        logging.info("✅ RAG Orchestrator is ready.")

    def _find_any_random_image(self) -> str | None:
        if not self.all_image_files:
            return None
        try:
            return random.choice(self.all_image_files)
        except Exception as e:
            logging.error(f"❌ Error finding a generic random image from cache: {e}")
            return None

    def _find_all_images_by_prefix(self, prefix: str) -> List[str]:
        if not prefix:
            return []
        cached_files = self.prefixed_image_map.get(prefix, [])
        if not cached_files:
            return []
        try:
            matching_files = list(cached_files)
            random.shuffle(matching_files)
            return matching_files
        except Exception as e:
            logging.error(f"❌ Error shuffling cached images for prefix '{prefix}': {e}")
            return list(cached_files)
            
    def _prepare_source_and_image_data(
        self, docs_to_show: List[Dict[str, Any]], priority_prefix: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Processes a list of documents to generate source info cards and an image gallery.
        """
        source_info: List[dict] = []
        static_image_gallery: List[str] = []
        processed_prefixes = set()

        for doc in docs_to_show:
            if not doc:
                continue

            prefix = doc.get("metadata", {}).get("image_prefix")
            doc_static_images = []

            if prefix and prefix not in processed_prefixes:
                found_images = self._find_all_images_by_prefix(prefix)
                if found_images:
                    doc_static_images.extend(found_images)
                    # Add all unique images from this prefix to the main gallery
                    for img_url in found_images:
                        if img_url not in static_image_gallery:
                            static_image_gallery.append(img_url)
                    processed_prefixes.add(prefix)
            elif prefix and prefix == priority_prefix:
                doc_static_images = self._find_all_images_by_prefix(prefix)

            source_info.append(
                {
                    "title": doc.get("title", "N/A"),
                    "summary": doc.get("summary", ""),
                    "image_urls": doc_static_images[: settings.SOURCE_CARD_IMAGE_LIMIT],
                }
            )

        return {"source_info": source_info, "image_gallery": static_image_gallery}

    async def _handle_welcome_flow(self, session_id: Optional[str] = None, **kwargs) -> dict:
        logging.info(f"🚦 [Router] Routing to: New Welcome Flow Handler | Session: {session_id}")
        if session_id:
            self.session_states[session_id] = {
                "awaiting": "analytics_origin_or_topic", 
                "timestamp": datetime.now(timezone.utc)
            }
            logging.info(f"🧠 [Session] Set state for Session '{session_id}' to 'awaiting_analytics_origin_or_topic'")
        
        welcome_message = (
            "สวัสดีค่ะ ยินดีต้อนรับสู่จังหวัดน่านค่ะ! "
            "น้องน่านเป็นไกด์ AI ที่ช่วยแนะนำข้อมูลที่เที่ยว ที่กิน หรือวัฒนธรรมในน่านได้เลยนะคะ\n\n"
            "เพื่อให้น้องน่านช่วยแนะนำได้ตรงใจคุณที่สุด "
            "พอจะบอกน้องน่านหน่อยได้ไหมคะว่า...\n"
            "1.  **มาจากที่ไหน?** หรือ\n"
            "2.  **สนใจเที่ยวแนวไหนเป็นพิเศษ (เช่น ธรรมชาติ, วัฒนธรรม, คาเฟ่)?**"
        )
        return {
            "answer": welcome_message, "action": "AWAITING_ANALYTICS_DATA", 
            "action_payload": None, "image_url": None, "image_gallery": [], "sources": [],
        }

    async def _handle_small_talk(self, corrected_query: str, session_id: Optional[str] = None, **kwargs) -> dict:
        final_answer = await get_llama_response_direct_async(user_query=corrected_query)
        return {
            "answer": final_answer, "action": None, "sources": [], "image_url": None, "image_gallery": [],
        }

    async def _handle_play_music(self, corrected_query: str, entity: Optional[str], mode: str, session_id: Optional[str] = None, **kwargs) -> dict:
        generic_music_requests = ["เพลง", "ฟังเพลง", "เปิดเพลง", "ร้องเพลง", "มิวสิค", "สับเพลง"]
        if not entity or entity.lower().strip() in generic_music_requests:
            return {
                "answer": "ได้เลยค่ะ! คุณอยากฟังเพลงอะไรเป็นพิเศษไหมคะ?",
                "action": "PROMPT_FOR_SONG_INPUT", "action_payload": {"placeholder": "เช่น Lover - Taylor Swift"},
                "sources": [], "image_url": None, "image_gallery": [],
            }
        else:
            search_query = entity
            search_results = await youtube_handler_instance.search_music(query=search_query)
            if not search_results:
                return {
                    "answer": f"ขออภัยค่ะ หาเพลง '{search_query}' ไม่เจอเลยค่ะ",
                    "action": "PROMPT_FOR_SONG_INPUT", "action_payload": {"placeholder": "ลองอีกครั้ง..."},
                    "sources": [], "image_url": None, "image_gallery": [],
                }
            return {
                "answer": f"เจอเพลงจาก '{search_query}' แล้วค่ะ! เลือกได้เลย",
                "action": "SHOW_SONG_CHOICES", "action_payload": search_results,
                "sources": [], "image_url": None, "image_gallery": [],
            }

    async def _handle_system_command(self, corrected_query: str, entity: Optional[str], session_id: Optional[str] = None, **kwargs) -> dict:
        logging.info("🚦 [Router] Routing to: System Command Handler")
        entity_to_launch = entity
        if not entity_to_launch:
            answer = "ขออภัยค่ะ ไม่เข้าใจว่าต้องการให้เปิดอะไร ลองบอกให้ชัดเจนขึ้นนะคะ เช่น 'เปิดเครื่องคิดเลข' หรือ 'เปิดเว็บ google'"
            return {
                "answer": answer, "action": None, "sources": [], "image_url": None, "image_gallery": [],
            }
        result_text = await asyncio.to_thread(system_tool_instance.launch, entity_to_launch)
        return {
            "answer": result_text, "action": None, "sources": [], "image_url": None, "image_gallery": [],
        }

    async def _handle_informational(
        self,
        corrected_query: str,
        entity: Optional[str],
        is_complex: bool,
        sub_queries: List[str],
        mode: str,
        session_id: Optional[str] = None, 
        **kwargs
    ) -> dict:
        
        priority_doc: Optional[Dict[str, Any]] = None
        priority_prefix: Optional[str] = None
        mongo_ids_from_search = []
        logging.info(f"🛰️ [Hybrid Retrieval] Starting retrieval for {len(sub_queries)} sub-queries...")
        for sub_q in sub_queries:
            if not sub_q.strip(): continue
            qdrant_results = await self.qdrant_manager.search_similar(query_text=sub_q, top_k=settings.QDRANT_TOP_K)
            for res in qdrant_results:
                if res.payload and res.payload.get("mongo_id"):
                    mongo_ids_from_search.append(res.payload.get("mongo_id"))
        unique_search_ids = list(dict.fromkeys(mongo_ids_from_search))
        logging.info(f"📚 [Hybrid Retrieval] Found {len(unique_search_ids)} unique documents.")
        fallback_image = self._find_any_random_image()
        if not unique_search_ids and not priority_doc:
            logging.warning(f"No results from Qdrant for: {corrected_query}")
            return {"answer": "ขออภัยค่ะ ไม่พบข้อมูลที่เกี่ยวข้อง", "action": None, "sources": [], "image_url": fallback_image, "image_gallery": [fallback_image] if fallback_image else []}
        final_mongo_ids = unique_search_ids
        if priority_doc and priority_doc.get("_id"):
            priority_id_str = str(priority_doc.get("_id"))
            if priority_id_str in final_mongo_ids: final_mongo_ids.remove(priority_id_str)
            final_mongo_ids.insert(0, priority_id_str)
        retrieved_docs = (await asyncio.to_thread(self.mongo_manager.get_locations_by_ids, final_mongo_ids) if final_mongo_ids else [])
        if not retrieved_docs:
            logging.error(f"No documents found in Mongo for IDs: {final_mongo_ids}")
            return {"answer": "ขออภัยค่ะ พบข้อมูลแต่ดึงรายละเอียดไม่ได้", "action": None, "sources": [], "image_url": fallback_image, "image_gallery": [fallback_image] if fallback_image else []}
        docs_with_synthetic = await asyncio.to_thread(lambda docs: [(doc, create_synthetic_document(doc)) for doc in docs], retrieved_docs)
        sentence_pairs = [[corrected_query, synthetic_doc] for doc, synthetic_doc in docs_with_synthetic]
        scores = await asyncio.to_thread(self.reranker.predict, sentence_pairs, show_progress_bar=False)
        reranked_docs_with_synthetic = sorted(zip(scores, docs_with_synthetic), key=lambda x: x[0], reverse=True)
        top_k_rerank = settings.TOP_K_RERANK_VOICE if mode == "voice" else settings.TOP_K_RERANK_TEXT
        final_docs = [doc for score, (doc, _) in reranked_docs_with_synthetic[:top_k_rerank]]
        final_synthetic_docs = [synth for score, (_, synth) in reranked_docs_with_synthetic[:top_k_rerank]]
        logging.info(f"✅ [Reranker] Kept top {len(final_docs)} documents.")

        primary_doc = final_docs[0] if final_docs else None
        primary_topic_fallback = primary_doc.get("title", "Unknown") if primary_doc else "สถานที่ในจังหวัดน่าน"
        if primary_doc and self.log_collection is not None:
            try:
                log_document = {"query": corrected_query, "primary_topic": primary_topic_fallback, "timestamp": datetime.now(timezone.utc)}
                await asyncio.to_thread(self.log_collection.insert_one, log_document)
            except Exception as e:
                logging.error(f"❌ [Analytics] Failed to log query: {e}")
        context_str = "\n\n---\n\n".join(final_synthetic_docs)
        insights = await asyncio.to_thread(get_insights_from_logs, self.log_collection)
        llm_response_dict = await get_groq_rag_response_async(
            user_query=corrected_query, context=context_str, insights=insights
        )

        final_answer = llm_response_dict.get("answer", "ขออภัยค่ะ มีข้อผิดพลาดในการสร้างคำตอบ")
        llm_sources_used = llm_response_dict.get("sources_used") 
        
        docs_to_show: List[dict] = []
        
        if llm_sources_used is not None:
            logging.info(f"✅ [SmartSource] LLM explicitly used {len(llm_sources_used)} sources.")
            valid_titles = set(llm_sources_used)
            docs_to_show = [doc for doc in final_docs if doc and doc.get("title") in valid_titles]
            
            if not docs_to_show and final_docs:
                logging.warning("⚠️ [SmartSource] LLM returned an empty source list. Falling back to the single best-ranked document.")
                docs_to_show.append(final_docs[0]) 
                
        else: 
            logging.warning("⚠️ [SmartSource] LLM response missing 'sources_used' key. Falling back to full RAG list.")
            docs_to_show = final_docs

        prepared_data = self._prepare_source_and_image_data(docs_to_show, priority_prefix)
        source_info = prepared_data["source_info"]
        static_image_gallery = prepared_data["image_gallery"]
        
        all_topics_list = [doc.get("title") for doc in docs_to_show if doc and doc.get("title")]
        topics_str_for_query = ", ".join(list(dict.fromkeys(all_topics_list))) if all_topics_list else primary_topic_fallback
        
        if len(static_image_gallery) < settings.IMAGE_FALLBACK_THRESHOLD:
            logging.warning(f"⚠️ Static images insufficient ({len(static_image_gallery)} found). Falling back to Google Search.")
            smarter_query = f"{corrected_query} {topics_str_for_query} น่าน"
            google_images = await image_search_tool_instance.get_image_urls(query=smarter_query, max_results=settings.GOOGLE_IMAGE_MAX_RESULTS)
            for g_url in google_images:
                if g_url not in static_image_gallery:
                    static_image_gallery.append(g_url)

        valid_gallery_urls = [url for url in static_image_gallery if url and isinstance(url, str)]
        main_image_url = valid_gallery_urls[0] if valid_gallery_urls else fallback_image
        logging.info(f"🖼️ Final valid gallery size: {len(valid_gallery_urls)}. Sources to show: {len(source_info)}")

        return {
            "answer": final_answer,
            "action": None,
            "image_url": main_image_url,
            "image_gallery": valid_gallery_urls[: settings.FINAL_GALLERY_IMAGE_LIMIT],
            "sources": source_info,
        }
    
    async def get_navigation_list(self) -> List[Dict[str, Any]]:
        logging.info("🗺️  [V-Maps] get_navigation_list called.")
        
        query_filter = {
            "doc_type": "Location"
        }
        logging.info(f"✅ [V-Maps] Using dynamic V.2 query filter: {query_filter}")
        
        projection = {"title": 1, "slug": 1, "topic": 1, "_id": 0, "metadata": 1} 
        
        try:
            results_from_db = await asyncio.to_thread(
                lambda: list(self.mongo_manager.get_collection("nan_locations").find(query_filter, projection))
            )
            
            enriched_results = []
            for doc in results_from_db:
                image_url = None
                if doc.get("metadata") and doc["metadata"].get("image_prefix"):
                    prefix = doc["metadata"]["image_prefix"]
                    all_images = self._find_all_images_by_prefix(prefix)
                    if all_images: image_url = all_images[0]
                doc["image_urls"] = [image_url] if image_url else []
                enriched_results.append(doc)

            sorted_results = sorted(enriched_results, key=lambda x: (x.get('topic', 'Z').startswith('วัด'), x.get('title', '')))
            
            logging.info(f"[V-Maps] Found and enriched {len(sorted_results)} navigation-ready locations.")
            return sorted_results
            
        except Exception as e:
            logging.error(f"❌ [V-Maps] Error querying navigation list from MongoDB: {e}", exc_info=True)
            return []

    async def handle_get_directions(self, entity_slug: str, user_lat: float, user_lon: float) -> dict:
        logging.info(f"🗺️  [V-Maps] Handling GET_DIRECTIONS for slug: '{entity_slug}'")
        api_key = settings.GOOGLE_API_KEY
        if not api_key:
            logging.error("[V-Maps] CRITICAL: GOOGLE_API_KEY is not set!")
            return {"answer": "ขออภัยค่ะ ระบบแผนที่ยังไม่ได้ตั้งค่า API Key ค่ะ", "action": None, "sources": [], "image_url": None, "image_gallery": []}
        
        doc = await asyncio.to_thread(self.mongo_manager.get_location_by_slug, entity_slug)

        if not doc or not doc.get("location_data"):
            if not doc:
                logging.warning(f"[V-Maps] Slug not found: {entity_slug}")
                return {"answer": f"ขออภัยค่ะ ไม่พบสถานที่ '{entity_slug}' ค่ะ", "action": None, "sources": [], "image_url": None, "image_gallery": []}
            elif doc.get("doc_type") == "Knowledge":
                logging.warning(f"[V-Maps] Cannot navigate to 'Knowledge' doc: {entity_slug}")
                return {"answer": f"ขออภัยค่ะ '{doc.get('title')}' เป็นข้อมูลความรู้ ไม่ใช่สถานที่ จึงไม่สามารถนำทางได้ค่ะ", "action": None, "sources": [], "image_url": None, "image_gallery": []}
            else:
                logging.warning(f"[V-Maps] 'Location' doc missing 'location_data': {entity_slug}")
                return {"answer": f"ขออภัยค่ะ น้องน่านยังไม่มีพิกัดแผนที่ของ '{doc.get('title')}' เลยค่ะ", "action": None, "sources": [], "image_url": None, "image_gallery": []}

        nav_data = doc["location_data"] # 🚀 [แก้ไข] ใช้ "location_data"
        doc_title = doc.get("title", "ปลายทาง")
        try:
            origin = f"{user_lat},{user_lon}"
            dest_lat = nav_data.get("latitude")
            dest_lon = nav_data.get("longitude")
            if dest_lat is None or dest_lon is None:
                logging.error(f"[V-Maps] Missing Lat/Lon in location_data for slug: {entity_slug}")
                raise ValueError("Missing coordinates")
            destination = f"{dest_lat},{dest_lon}"
            logging.info(f"[V-Maps] Origin: {origin} | Destination: {destination}")
            base_url = "http://googleusercontent.com/maps/google.com/0"
            params = { "key": api_key, "origin": origin, "destination": destination, "mode": "driving" }
            embed_url = f"{base_url}?{urllib.parse.urlencode(params)}"
            logging.info(f"[V-Maps] Generated Embed URL: {embed_url}")
            return {"answer": f"พร้อมแล้วค่ะ! นี่คือเส้นทางจากตำแหน่งของคุณไปยัง **{doc_title}** นะคะ", "action": "SHOW_MAP_EMBED", "action_payload": {"embed_url": embed_url, "destination_name": doc_title}, "image_url": None, "image_gallery": [], "sources": []}
        except Exception as e:
            logging.error(f"❌ [V-Maps] Error building Embed URL: {e}", exc_info=True)
            return {"answer": "ขออภัยค่ะ เกิดข้อผิดพลาดในการสร้าง URL แผนที่ค่ะ", "action": None, "sources": [], "image_url": None, "image_gallery": []}
    
    async def answer_query(self, query: str, mode: str = "text", session_id: Optional[str] = None) -> dict:
        
        if session_id and (state := self.session_states.get(session_id)):
            if state.get("awaiting") == "analytics_origin_or_topic":
                logging.info(f"🚦 [Router] Intercepting query. Passing to AnalyticsHandler for Session: '{session_id}'")
                self.session_states.pop(session_id, None) 
                return await self.analytics_handler.handle_analytics_response(query, session_id, mode)

        try:
            interpretation = await self.query_interpreter.interpret_and_route(query)
        except Exception as e:
            logging.error(f"❌ [Router V6.4] CRITICAL: Failed to interpret query: {e}. Defaulting to INFORMATIONAL.", exc_info=True)
            fallback_image = self._find_any_random_image()
            return {"answer": "ขออภัยค่ะ มีปัญหาในการตีความคำถาม", "action": None, "sources": [], "image_url": fallback_image, "image_gallery": [fallback_image] if fallback_image else []}
        
        intent = interpretation.get("intent", "INFORMATIONAL")
        corrected_query = interpretation.get("corrected_query", query)
        entity = interpretation.get("entity")
        is_complex = interpretation.get("is_complex", False)
        sub_queries = interpretation.get("sub_queries", [corrected_query])
        
        logging.info(
            f"🚦 [Router V6.4] Intent: {intent} | Entity: {entity} | "
            f"Complex: {is_complex} | Sub-Queries: {sub_queries} | Mode: {mode}"
        )
        
        handler_map = {
            "WELCOME_GREETING": self._handle_welcome_flow, 
            "SMALL_TALK": self._handle_small_talk,
            "PLAY_MUSIC": self._handle_play_music, 
            "INFORMATIONAL": self._handle_informational,
            "SYSTEM_COMMAND": self._handle_system_command,
        }
        handler = handler_map.get(intent, self._handle_informational)
        
        try:
            handler_kwargs = {
                "corrected_query": corrected_query, "session_id": session_id, "mode": mode,
                "entity": entity, "is_complex": is_complex, "sub_queries": sub_queries
            }
            return await handler(**handler_kwargs)
        except Exception as e:
            logging.error(f"❌ [Router V6.4] Error executing handler for intent '{intent}': {e}", exc_info=True)
            fallback_image = self._find_any_random_image()
            return {"answer": "ขออภัยค่ะ เกิดข้อผิดพลาดภายใน", "action": None, "sources": [], "image_url": fallback_image, "image_gallery": [fallback_image] if fallback_image else []}