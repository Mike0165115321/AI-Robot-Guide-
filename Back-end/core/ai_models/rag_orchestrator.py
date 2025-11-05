import asyncio
import logging
import random
import re
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Any, List, Optional

from sentence_transformers import CrossEncoder

from core.config import settings
from core.ai_models.llm_handler import (
    get_llama_response_direct_async,
    get_groq_rag_response_async,
    get_insights_from_logs
)
from core.ai_models.query_interpreter import QueryInterpreter
from core.ai_models.youtube_handler import youtube_handler_instance
from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from core.tools.system_tool import system_tool_instance
from utils.helper_functions import create_synthetic_document
from core.tools.image_search_tool import image_search_tool_instance

BACKEND_ROOT = Path(__file__).resolve().parent.parent.parent
IMAGE_DIR = BACKEND_ROOT / "static" / "images"

logging.info(f"📂 [RAGOrchestrator] IMAGE_DIR set to: {IMAGE_DIR.absolute()}")

class RAGOrchestrator:
    def __init__(self, 
                 mongo_manager: MongoDBManager, 
                 qdrant_manager: QdrantManager,
                 query_interpreter: QueryInterpreter):
        
        logging.info("⚙️  RAG Orchestrator (V6.0 - Hybrid + Decompose) is initializing...") 
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
        
        self.all_image_files: List[str] = []
        self.prefixed_image_map: Dict[str, List[str]] = {}

        logging.info(f"🏞️ Scanning image directory: {IMAGE_DIR.absolute()}")
        self.image_dir_exists = IMAGE_DIR.is_dir()

        if self.image_dir_exists:
            try:
                for f in IMAGE_DIR.iterdir():
                    if f.is_file() and f.suffix.lower() in ('.jpg', '.jpeg', '.png', '.webp'):
                        file_path_str = f"/static/images/{f.name}"
                        self.all_image_files.append(file_path_str)
                        prefix = ""
                        if '-' in f.name:
                             prefix = f.name.rsplit('-', 1)[0] + '-'
                        elif '_' in f.name:
                             prefix = f.name.split('_')[0] + '_'
                        
                        if prefix:
                            if prefix not in self.prefixed_image_map:
                                self.prefixed_image_map[prefix] = []
                            self.prefixed_image_map[prefix].append(file_path_str)
                            
                logging.info(f"✅ Image Cache: Found {len(self.all_image_files)} images and {len(self.prefixed_image_map)} prefixes.")
            except Exception as e:
                logging.error(f"❌ Error scanning image directory: {e}")

        logging.info("✅ RAG Orchestrator is ready.")

    def _find_any_random_image(self) -> str | None:
        if not self.all_image_files: return None
        try: return random.choice(self.all_image_files)
        except Exception as e:
            logging.error(f"❌ Error finding a generic random image from cache: {e}")
            return None

    def _find_all_images_by_prefix(self, prefix: str) -> List[str]:
        if not prefix: return []
        cached_files = self.prefixed_image_map.get(prefix, [])
        if not cached_files: return []
        try:
            matching_files = list(cached_files) 
            random.shuffle(matching_files)
            return matching_files
        except Exception as e:
            logging.error(f"❌ Error shuffling cached images for prefix '{prefix}': {e}")
            return list(cached_files)

    async def _handle_small_talk(self, corrected_query: str) -> dict:
        final_answer = await get_llama_response_direct_async(user_query=corrected_query)
        return {"answer": final_answer, "action": None, "sources": [], "image_url": None, "image_gallery": []}

    async def _handle_play_music(self, corrected_query: str, entity: Optional[str], mode: str) -> dict:
        generic_music_requests = ["เพลง", "ฟังเพลง", "เปิดเพลง", "ร้องเพลง", "มิวสิค", "สับเพลง"]
        
        if not entity or entity.lower().strip() in generic_music_requests:
            return {"answer": "ได้เลยค่ะ! คุณอยากฟังเพลงอะไรเป็นพิเศษไหมคะ?", "action": "PROMPT_FOR_SONG_INPUT", "action_payload": {"placeholder": "เช่น Lover - Taylor Swift"}, "sources": [], "image_url": None, "image_gallery": []}
        else:
            search_query = entity
            search_results = await youtube_handler_instance.search_music(query=search_query)
            if not search_results:
                return {"answer": f"ขออภัยค่ะ หาเพลง '{search_query}' ไม่เจอเลยค่ะ", "action": "PROMPT_FOR_SONG_INPUT", "action_payload": {"placeholder": "ลองอีกครั้ง..."}, "sources": [], "image_url": None, "image_gallery": []}
            return {"answer": f"เจอเพลงจาก '{search_query}' แล้วค่ะ! เลือกได้เลย", "action": "SHOW_SONG_CHOICES", "action_payload": search_results, "sources": [], "image_url": None, "image_gallery": []}

    async def _handle_system_command(self, corrected_query: str, entity: Optional[str]) -> dict:
        logging.info("🚦 [Router] Routing to: System Command Handler")
        entity_to_launch = entity
        if not entity_to_launch:
            answer = "ขออภัยค่ะ ไม่เข้าใจว่าต้องการให้เปิดอะไร ลองบอกให้ชัดเจนขึ้นนะคะ เช่น 'เปิดเครื่องคิดเลข' หรือ 'เปิดเว็บ google'"
            return {"answer": answer, "action": None, "sources": [], "image_url": None, "image_gallery": []}
        result_text = await asyncio.to_thread(system_tool_instance.launch, entity_to_launch)
        return {"answer": result_text, "action": None, "sources": [], "image_url": None, "image_gallery": []}

    async def _handle_informational(
        self, 
        corrected_query: str, 
        mode: str, 
        entity: Optional[str], 
        is_complex: bool, 
        sub_queries: List[str]
    ) -> dict:
        
        logging.info(f"➡️ [Router V6] Handling informational query. Entity: {entity}, Complex: {is_complex}, Sub-Queries: {sub_queries}")
        
        priority_doc = None
        priority_prefix = None
        static_image_gallery: List[str] = [] 
        processed_prefixes = set()

        if entity:
            try:
                logging.info(f"🎯 [Static Search] Trying to find exact match for entity: '{entity}'")
                priority_doc = await asyncio.to_thread(self.mongo_manager.get_location_by_title, entity) 
                
                if priority_doc:
                    priority_prefix = priority_doc.get("metadata", {}).get("image_prefix")
                    if priority_prefix:
                        logging.info(f"🎯 [Static Search] Found priority prefix '{priority_prefix}'. Pre-loading images.")
                        found_images = self._find_all_images_by_prefix(priority_prefix)
                        if found_images:
                            static_image_gallery.extend(found_images)
                            processed_prefixes.add(priority_prefix)
            except Exception as e:
                logging.error(f"❌ Error during priority entity search: {e}")

        mongo_ids_from_search = []
        logging.info(f"🛰️ [Hybrid Retrieval] Starting retrieval for {len(sub_queries)} sub-queries...")
        
        for sub_q in sub_queries:
            if not sub_q.strip(): continue
            logging.info(f"🔍 [Hybrid Retrieval] Searching for: '{sub_q}'")
            qdrant_results = await self.qdrant_manager.search_similar(query_text=sub_q, top_k=settings.QDRANT_TOP_K)
            
            for res in qdrant_results:
                if res.payload and res.payload.get('mongo_id'):
                    mongo_ids_from_search.append(res.payload.get('mongo_id'))

        unique_search_ids = list(dict.fromkeys(mongo_ids_from_search))
        logging.info(f"📚 [Hybrid Retrieval] Found {len(unique_search_ids)} unique documents from all sub-queries.")

        fallback_image = self._find_any_random_image()

        if not unique_search_ids and not priority_doc:
            logging.warning(f"No results from Qdrant and no priority doc found for: {corrected_query}")
            return {"answer": "ขออภัยค่ะ ไม่พบข้อมูลที่เกี่ยวข้อง", "action": None, "sources": [], "image_url": fallback_image, "image_gallery": [fallback_image] if fallback_image else []}

        final_mongo_ids = unique_search_ids
        
        if priority_doc and priority_doc.get('_id'):
            priority_id_str = str(priority_doc.get('_id'))
            if priority_id_str in final_mongo_ids:
                 final_mongo_ids.remove(priority_id_str)
            
            logging.info(f"Injecting priority doc (ID: {priority_id_str}) into retrieved list.")
            final_mongo_ids.insert(0, priority_id_str)

        retrieved_docs = await asyncio.to_thread(self.mongo_manager.get_locations_by_ids, final_mongo_ids) if final_mongo_ids else []
        if not retrieved_docs:
            logging.error(f"No documents found in Mongo for IDs: {final_mongo_ids}")
            return {"answer": "ขออภัยค่ะ พบข้อมูลแต่ดึงรายละเอียดไม่ได้", "action": None, "sources": [], "image_url": fallback_image, "image_gallery": [fallback_image] if fallback_image else []}
        docs_with_synthetic = await asyncio.to_thread(lambda docs: [(doc, create_synthetic_document(doc)) for doc in docs], retrieved_docs)

        logging.info(f"🧠 [Reranker] Reranking {len(docs_with_synthetic)} documents against: '{corrected_query}'")
        sentence_pairs = [[corrected_query, synthetic_doc] for doc, synthetic_doc in docs_with_synthetic]
        scores = await asyncio.to_thread(self.reranker.predict, sentence_pairs, show_progress_bar=False)
        reranked_docs_with_synthetic = sorted(zip(scores, docs_with_synthetic), key=lambda x: x[0], reverse=True)

        top_k_rerank = settings.TOP_K_RERANK_VOICE if mode == 'voice' else settings.TOP_K_RERANK_TEXT
        final_docs = [doc for score, (doc, _) in reranked_docs_with_synthetic[:top_k_rerank]]
        final_synthetic_docs = [synth for score, (_, synth) in reranked_docs_with_synthetic[:top_k_rerank]]
        logging.info(f"✅ [Reranker] Kept top {len(final_docs)} documents.")

        primary_doc = final_docs[0] if final_docs else None
        primary_topic_fallback = primary_doc.get("title", "Unknown") if primary_doc else "สถานที่ในจังหวัดน่าน"

        all_topics_list = [doc.get("title") for doc in final_docs if doc and doc.get("title")]
        unique_topics = list(dict.fromkeys(all_topics_list))
        
        if not unique_topics:
            topics_str_for_query = primary_topic_fallback
        else:
            topics_str_for_query = ", ".join(unique_topics)

        if primary_doc and self.log_collection is not None:
            try:
                log_document = {"query": corrected_query, "primary_topic": primary_topic_fallback, "timestamp": datetime.now(timezone.utc)}
                await asyncio.to_thread(self.log_collection.insert_one, log_document)
            except Exception as e:
                logging.error(f"❌ [Analytics] Failed to log query: {e}")

        context_str = "\n\n---\n\n".join(final_synthetic_docs)
        
        source_info: List[dict] = []

        for doc in final_docs:
            if not doc: continue
            prefix = doc.get("metadata", {}).get("image_prefix")
            doc_static_images = []
            
            if prefix and prefix not in processed_prefixes:
                found_images = self._find_all_images_by_prefix(prefix)
                if found_images:
                    doc_static_images.extend(found_images)
                    for img_url in found_images:
                        if img_url not in static_image_gallery:
                            static_image_gallery.append(img_url)
                    processed_prefixes.add(prefix)
            
            elif prefix and prefix == priority_prefix:
                doc_static_images = self._find_all_images_by_prefix(prefix)

            source_info.append({
                "title": doc.get('title', 'N/A'),
                "summary": doc.get('summary', ''),
                "image_urls": doc_static_images[:settings.SOURCE_CARD_IMAGE_LIMIT]
            })

        if len(static_image_gallery) < settings.IMAGE_FALLBACK_THRESHOLD:
            logging.warning(f"⚠️ Static images insufficient (found {len(static_image_gallery)}). Falling back to Google Search for topics: '{topics_str_for_query}'...")
            
            smarter_query = f"{corrected_query} {topics_str_for_query} น่าน"
            
            logging.info(f"🔍 [Google Search] Using smarter query: '{smarter_query}'")
            google_images = await image_search_tool_instance.get_image_urls(
                query=smarter_query,
                max_results=settings.GOOGLE_IMAGE_MAX_RESULTS
            )
            for g_url in google_images:
                if g_url not in static_image_gallery:
                    static_image_gallery.append(g_url)
        
        insights = await asyncio.to_thread(get_insights_from_logs, self.log_collection)

        final_answer = await get_groq_rag_response_async(
            user_query=corrected_query,
            context=context_str,
            insights=insights
        )
        valid_gallery_urls = [url for url in static_image_gallery if url and isinstance(url, str)]
        main_image_url = valid_gallery_urls[0] if valid_gallery_urls else fallback_image
        
        logging.info(f"🖼️ Final valid gallery size: {len(valid_gallery_urls)}")

        return {
            "answer": final_answer, "action": None,
            "image_url": main_image_url,
            "image_gallery": valid_gallery_urls[:settings.FINAL_GALLERY_IMAGE_LIMIT],
            "sources": source_info
        }

    async def answer_query(self, query: str, mode: str = 'text') -> dict:
        try:
            interpretation = await self.query_interpreter.interpret_and_route(query)
        except Exception as e:
            logging.error(f"❌ [Router V6] CRITICAL: Failed to interpret query: {e}. Defaulting to INFORMATIONAL.", exc_info=True)
            fallback_image = self._find_any_random_image()
            return {"answer": "ขออภัยค่ะ มีปัญหาในการตีความคำถาม", "action": None, "sources": [], "image_url": fallback_image, "image_gallery": [fallback_image] if fallback_image else []}

        intent = interpretation.get("intent", "INFORMATIONAL")
        corrected_query = interpretation.get("corrected_query", query)
        entity = interpretation.get("entity")
        is_complex = interpretation.get("is_complex", False)
        sub_queries = interpretation.get("sub_queries", [corrected_query]) 

        logging.info(f"🚦 [Router V6] Intent: {intent} | Entity: {entity} | Complex: {is_complex} | Sub-Queries: {sub_queries} | Mode: {mode}")

        handler_map = {
            "SMALL_TALK": self._handle_small_talk,
            "PLAY_MUSIC": self._handle_play_music,
            "INFORMATIONAL": self._handle_informational,
            "SYSTEM_COMMAND": self._handle_system_command,
        }
        handler = handler_map.get(intent, self._handle_informational)

        try:
            handler_kwargs = {"corrected_query": corrected_query}
            
            if intent == "INFORMATIONAL":
                handler_kwargs["mode"] = mode
                handler_kwargs["entity"] = entity
                handler_kwargs["is_complex"] = is_complex
                handler_kwargs["sub_queries"] = sub_queries
            
            elif intent == "PLAY_MUSIC":
                handler_kwargs["mode"] = mode
                handler_kwargs["entity"] = entity

            elif intent == "SYSTEM_COMMAND":
                 handler_kwargs["entity"] = entity
            
            
            return await handler(**handler_kwargs) 
            
        except Exception as e:
            logging.error(f"❌ [Router V6] Error executing handler for intent '{intent}': {e}", exc_info=True)
            fallback_image = self._find_any_random_image()
            return {"answer": "ขออภัยค่ะ เกิดข้อผิดพลาดภายใน", "action": None, "sources": [], "image_url": fallback_image, "image_gallery": [fallback_image] if fallback_image else []}