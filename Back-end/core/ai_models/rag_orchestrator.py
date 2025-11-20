# /Back-end/core/ai_models/rag_orchestrator.py

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
        logging.info("⚙️  RAG Orchestrator (V7.4 - Memory Added) is initializing...")
        self.mongo_manager = mongo_manager
        self.qdrant_manager = qdrant_manager
        self.query_interpreter = query_interpreter

        self.reranker_model_name = settings.RERANKER_MODEL_NAME
        self.device = settings.DEVICE

        logging.info(f"🔄 Loading Re-ranker Model ('{self.reranker_model_name}' on '{self.device}')...")
        self.reranker = CrossEncoder(self.reranker_model_name, device=self.device)
        logging.info("✅ Re-ranker Model loaded.")

        self.log_collection = self.mongo_manager.get_collection("query_logs")
        
        self.session_states: Dict[str, Dict[str, Any]] = {}
        
        self.analytics_handler = AnalyticsHandler(
            mongo_manager=self.mongo_manager,
            query_interpreter=self.query_interpreter,
            orchestrator_callback=self.answer_query
        )
        
        self.all_image_files: List[str] = []
        self.prefixed_image_map: Dict[str, List[str]] = {}
        self._initialize_image_cache()
        
        logging.info("✅ RAG Orchestrator is ready.")

    def _initialize_image_cache(self):
        if IMAGE_DIR.is_dir():
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
                logging.info(f"✅ Image Cache: {len(self.all_image_files)} images.")
            except Exception as e:
                logging.error(f"❌ Error scanning image directory: {e}")

    def _find_any_random_image(self) -> str | None:
        if not self.all_image_files: return None
        try:
            return random.choice(self.all_image_files)
        except:
            return None

    def _find_all_images_by_prefix(self, prefix: str) -> List[str]:
        if not prefix: return []
        cached_files = self.prefixed_image_map.get(prefix, [])
        if not cached_files: return []
        matching_files = list(cached_files)
        random.shuffle(matching_files)
        return matching_files

    async def _inject_images_into_text(self, text: str) -> str:
        if not text: return ""
        pattern = r"\{\{IMAGE:\s*(.*?)\}\}"
        matches = re.findall(pattern, text)
        for keyword in matches:
            image_url = None
            safe_keyword = keyword.replace(" ", "-").lower()
            for prefix, paths in self.prefixed_image_map.items():
                 if (safe_keyword in prefix.lower() or prefix.lower().replace("-", " ") in keyword.lower()) and paths:
                     image_url = random.choice(paths)
                     break
            if not image_url:
                try:
                    search_q = f"{keyword} จังหวัดน่าน"
                    logging.info(f"🖼️ [Injection] Searching Google for tag: '{keyword}'")
                    google_urls = await image_search_tool_instance.get_image_urls(search_q, max_results=1)
                    if google_urls:
                        image_url = google_urls[0]
                except Exception as e:
                    logging.warning(f"Image injection search failed for {keyword}: {e}")
            if image_url:
                full_url = construct_full_image_url(image_url)
                replacement = f"\n\n![{keyword}]({full_url})\n\n"
            else:
                replacement = ""
            text = re.sub(r"\{\{IMAGE:\s*" + re.escape(keyword) + r"\}\}", replacement, text, count=1)
        return text
            
    def _prepare_source_and_image_data(self, docs_to_show: List[Dict[str, Any]]) -> Dict[str, Any]:
        source_info: List[dict] = []
        static_image_gallery: List[str] = []
        processed_prefixes = set()
        for doc in docs_to_show:
            if not doc: continue
            prefix = doc.get("metadata", {}).get("image_prefix")
            doc_images = []
            if prefix and prefix not in processed_prefixes:
                found_images = self._find_all_images_by_prefix(prefix)
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
        if session_id:
            # คงค่า turn_count ไว้ ไม่ reset
            if session_id not in self.session_states:
                 self.session_states[session_id] = {"turn_count": 0}
            
            self.session_states[session_id]["awaiting"] = "analytics_origin_or_topic"
            self.session_states[session_id]["timestamp"] = datetime.now(timezone.utc)
            
        return {
            "answer": "สวัสดีค่ะ ยินดีต้อนรับสู่จังหวัดน่านค่ะ! น้องน่านเป็นไกด์ AI ที่ช่วยแนะนำที่เที่ยว ที่กิน หรือวัฒนธรรมได้เลยนะคะ\n\nบอกหน่อยได้ไหมคะว่า **มาจากที่ไหน?** หรือ **สนใจเที่ยวแนวไหนเป็นพิเศษคะ?**",
            "action": "AWAITING_ANALYTICS_DATA", 
            "action_payload": None, "image_url": None, "image_gallery": [], "sources": [],
        }

    async def _handle_small_talk(self, corrected_query: str, **kwargs) -> dict:
        final_answer = await get_llama_response_direct_async(user_query=corrected_query)
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
        turn_count: int = 1, session_id: Optional[str] = None, **kwargs
    ) -> dict:
        search_queries = [corrected_query] + sub_queries
        if entity:
            search_queries.append(entity)
            search_queries.append(f"ข้อมูลทั่วไป {entity}")
            # 🚀 [เพิ่ม] ค้นหาแบบเจาะจงจังหวัดน่าน เพื่อลด Hallucination (เช่น ถนนหมายเลข 3)
            search_queries.append(f"{entity} จังหวัดน่าน")
        
        unique_queries = list(set([q for q in search_queries if q.strip()]))
        logging.info(f"🛰️ [RAG] Searching for: {unique_queries}")
        
        mongo_ids_from_search = []
        for q in unique_queries:
            qdrant_results = await self.qdrant_manager.search_similar(query_text=q, top_k=settings.QDRANT_TOP_K)
            for res in qdrant_results:
                if res.payload and res.payload.get("mongo_id"):
                    mongo_ids_from_search.append(res.payload.get("mongo_id"))

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
        
        # 🔥 [Memory] จำ Topic ล่าสุดที่คุยกัน ไว้ใช้กับคำสั่ง "นำทาง"
        if final_docs and session_id and session_id in self.session_states:
            best_topic = final_docs[0].get("title")
            self.session_states[session_id]["last_topic"] = best_topic
            logging.info(f"🧠 [Memory] Remembered topic: {best_topic}")

        if final_docs and self.log_collection is not None:
            try:
                primary_topic = final_docs[0].get("title", "General")
                log_document = {
                    "query": corrected_query,
                    "primary_topic": primary_topic,
                    "timestamp": datetime.now(timezone.utc)
                }
                asyncio.create_task(asyncio.to_thread(self.log_collection.insert_one, log_document))
            except Exception as e:
                logging.error(f"❌ [Analytics] Failed to log query: {e}")

        context_parts = []
        for i, doc in enumerate(final_docs, 1):
            doc_text = create_synthetic_document(doc)
            context_parts.append(f"[Document {i}]\nTitle: {doc.get('title')}\nInfo: {doc_text}")
        context_str = "\n\n----------------\n\n".join(context_parts)

        insights = await asyncio.to_thread(get_insights_from_logs, self.log_collection)
        llm_response = await get_groq_rag_response_async(corrected_query, context_str, insights, turn_count=turn_count)
        
        final_answer = llm_response.get("answer", "ขออภัยค่ะ ระบบประมวลผลคำตอบผิดพลาด")
        llm_sources = llm_response.get("sources_used", [])

        docs_to_show = []
        if llm_sources:
            docs_to_show = [d for d in final_docs if d.get("title") in llm_sources]
        if not docs_to_show and final_docs:
            docs_to_show = [final_docs[0]] 

        final_answer_with_images = await self._inject_images_into_text(final_answer)
        prepared_data = self._prepare_source_and_image_data(docs_to_show)
        static_gallery = prepared_data["image_gallery"]
        
        if len(static_gallery) < settings.IMAGE_FALLBACK_THRESHOLD:
            titles = [d.get("title") for d in docs_to_show]
            if titles:
                search_q = f"{titles[0]} จังหวัดน่าน" 
                logging.info(f"🖼️ [Fallback] Searching Google Image for: {search_q}")
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
        }

    def _calculate_distance(self, lat1, lon1, lat2, lon2):
        if None in [lat1, lon1, lat2, lon2]: return float('inf')
        R = 6371 
        dLat, dLon = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
        a = math.sin(dLat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dLon/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    async def get_navigation_list(self, user_lat: float = None, user_lon: float = None) -> List[Dict[str, Any]]:
        try:
            docs = await asyncio.to_thread(lambda: list(self.mongo_manager.get_collection("nan_locations").find(
                {"doc_type": "Location"}, 
                {"title":1, "slug":1, "topic":1, "summary":1, "category":1, "location_data":1, "metadata":1, "_id":0}
            )))
            for doc in docs:
                prefix = doc.get("metadata", {}).get("image_prefix")
                imgs = self._find_all_images_by_prefix(prefix)
                doc["image_urls"] = [imgs[0]] if imgs else []
                dist = float('inf')
                if user_lat and user_lon and doc.get("location_data"):
                    dist = self._calculate_distance(user_lat, user_lon, doc["location_data"].get("latitude"), doc["location_data"].get("longitude"))
                doc["distance_km"] = round(dist, 1) if dist != float('inf') else None
            docs.sort(key=lambda x: x["distance_km"] if x["distance_km"] is not None else 99999)
            return docs
        except Exception as e:
            logging.error(f"❌ [V-Maps] Error: {e}")
            return []

    async def handle_get_directions(self, entity_slug: str, user_lat: float = None, user_lon: float = None) -> dict:
        logging.info(f"🗺️  [V-Maps] Handling Directions for: '{entity_slug}'")
        
        # 1. ค้นหาสถานที่ (Slug หรือ Title)
        doc = await asyncio.to_thread(self.mongo_manager.get_location_by_slug, entity_slug)
        if not doc:
            logging.info(f"[V-Maps] Slug not found. Searching by title: '{entity_slug}'")
            doc = await asyncio.to_thread(self.mongo_manager.get_location_by_title, entity_slug)

        if not doc or not doc.get("location_data"):
            return {
                "answer": f"ขออภัยค่ะ ไม่พบพิกัดของ **{entity_slug}** ในระบบ ลองระบุชื่อให้ชัดเจนขึ้นอีกนิดนะคะ", 
                "action": None, "sources": [], "image_url": None
            }

        nav_data = doc["location_data"]
        dest_name = doc.get("title", "ปลายทาง")
        dest_lat = nav_data.get("latitude")
        dest_lon = nav_data.get("longitude")
        
        # 3. สร้าง URL (✅ สูตรมาตรฐาน: ใช้ maps.google.com/maps?output=embed)
        # สูตรนี้ขึ้นแน่นอน ไม่ต้องใช้ API Key และไม่โดนบล็อก Iframe
        embed_url = f"https://maps.google.com/maps?q={dest_lat},{dest_lon}&z=15&output=embed"

        # Link สำหรับกดเปิดแอป (External)
        if user_lat and user_lon:
             google_maps_link = f"https://www.google.com/maps/dir/?api=1&origin={user_lat},{user_lon}&destination={dest_lat},{dest_lon}&travelmode=driving"
        else:
             google_maps_link = f"https://www.google.com/maps/search/?api=1&query={dest_lat},{dest_lon}"

        # 4. ส่งผลลัพธ์
        return {
            "answer": f"จัดให้ตามคำขอค่ะ! นี่คือพิกัดของ **{dest_name}** \n\n(กดปุ่มด้านล่างเพื่อเปิดแอปนำทางได้เลยนะคะ 👇)",
            "action": "SHOW_MAP_EMBED",
            "action_payload": {
                "embed_url": embed_url,
                "destination_name": dest_name,
                "external_link": google_maps_link 
            },
            "image_url": None, "image_gallery": [], "sources": []
        }
    
    async def answer_query(self, query: str, mode: str = "text", session_id: Optional[str] = None, **kwargs) -> dict:
        # 1. Analytics Logic
        if session_id and (state := self.session_states.get(session_id)):
            if state.get("awaiting") == "analytics_origin_or_topic":
                self.session_states.pop(session_id, None)
                return await self.analytics_handler.handle_analytics_response(query, session_id, mode)

        # 2. Session Logic
        current_turn = 1
        if session_id:
            if session_id not in self.session_states:
                self.session_states[session_id] = {"turn_count": 0, "last_topic": None}
            
            self.session_states[session_id]["turn_count"] += 1
            current_turn = self.session_states[session_id]["turn_count"]
            logging.info(f"🔄 [Session] ID: {session_id} | Turn: {current_turn}")

        # 3. Interpretation
        try:
            interpretation = await self.query_interpreter.interpret_and_route(query)
        except Exception as e:
            logging.error(f"❌ Router Error: {e}")
            return {"answer": "ขออภัยค่ะ ระบบขัดข้องชั่วคราว", "action": None, "sources": [], "image_url": None}
        
        intent = interpretation.get("intent", "INFORMATIONAL")
        corrected_query = interpretation.get("corrected_query", query)
        entity = interpretation.get("entity")
        
        logging.info(f"🚦 Intent: {intent} | Query: {query} | Entity: {entity}")

        # 4. Smart Navigation Router
        navigation_keywords = ["นำทาง", "เส้นทาง", "พาไป", "ขอทาง", "ไปยัง", "ไปวัด", "ไปที่"]
        is_nav_request = any(kw in corrected_query for kw in navigation_keywords)
        
        if is_nav_request:
            target_entity = entity
            
            # 🔥 [Memory Magic] ถ้า User ไม่บอกสถานที่ ให้ไปขุดความจำเก่า (Last Topic)
            if not target_entity and session_id and session_id in self.session_states:
                last_topic = self.session_states[session_id].get("last_topic")
                if last_topic:
                    logging.info(f"🧠 [Memory] User didn't say where, assuming last topic: '{last_topic}'")
                    target_entity = last_topic
                else:
                    # ถ้าไม่เคยคุยเรื่องสถานที่มาก่อนเลย
                    return {"answer": "ได้เลยค่ะ! แต่ช่วยบอกน้องน่านหน่อยได้ไหมคะว่าจะให้ **นำทางไปที่ไหน?** 😊", "action": None, "sources": [], "image_url": None}

            if target_entity:
                logging.info(f"🗺️ [Smart Router] Switching to Navigation Handler for: '{target_entity}'")
                return await self.handle_get_directions(
                    entity_slug=target_entity, 
                    user_lat=kwargs.get('user_lat', 0.0),
                    user_lon=kwargs.get('user_lon', 0.0)
                )
        
        # 5. Normal Handlers
        handler_map = {
            "WELCOME_GREETING": self._handle_welcome_flow,
            "SMALL_TALK": self._handle_small_talk,
            "PLAY_MUSIC": self._handle_play_music,
            "INFORMATIONAL": self._handle_informational,
            "SYSTEM_COMMAND": self._handle_system_command,
        }
        handler = handler_map.get(intent, self._handle_informational)
        
        return await handler(
            corrected_query=corrected_query,
            entity=entity,
            is_complex=interpretation.get("is_complex", False),
            sub_queries=interpretation.get("sub_queries", []),
            mode=mode,
            session_id=session_id,
            turn_count=current_turn,
            **kwargs
        )