import logging
import json
import asyncio
from groq import AsyncGroq
from typing import Dict, Any, Optional, List
from .key_manager import groq_key_manager
from core.config import settings 

class QueryInterpreter:
    def __init__(self):
        self.model_to_use = settings.GROQ_LLAMA_MODEL 
        
        api_key = groq_key_manager.get_key()
        if not api_key:
            logging.error("🚨 [Interpreter] CRITICAL: No Groq API key found on init.")
            self.client = None
        else:
            self.client = AsyncGroq(api_key=api_key)
            
        # [NEW V6.1] อัปเดตชื่อเวอร์ชันใน log
        logging.info(f"🧠 Query Interpreter (V6.1 - Entity Patch) initialized with model: {self.model_to_use} (Async Mode)")

    async def close(self):
        """Closes the AsyncGroq client."""
        if self.client:
            logging.info("⏳ [Interpreter] Closing Groq client...")
            try:
                await self.client.close()
                logging.info("✅ [Interpreter] Groq client closed.")
            except Exception as e:
                logging.error(f"❌ Error closing Groq client: {e}")

    async def _get_groq_response(self, system_prompt: str, user_query: str) -> Optional[str]:
        if not self.client:
            logging.error("❌ [Interpreter] Groq client not initialized (No API key).")
            return None
            
        try:
            chat_completion = await self.client.chat.completions.create(
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_query}
                ],
                model=self.model_to_use,
                temperature=0.0,
                response_format={"type": "json_object"}, 
            )
            return chat_completion.choices[0].message.content.strip()
        except Exception as e:
            logging.error(f"❌ [Interpreter] Groq API Error: {e}", exc_info=True)
            return None

    async def interpret_and_route(self, query: str) -> Dict[str, Any]:
        if not query.strip():
            return {
                "corrected_query": "", 
                "intent": "SMALL_TALK", 
                "entity": None, 
                "is_complex": False, 
                "sub_queries": [""]
            }
            
        fallback_result = {
            "corrected_query": query, 
            "intent": "INFORMATIONAL", 
            "entity": None, # Fallback ที่ปลอดภัยคือไม่มี entity
            "is_complex": False, 
            "sub_queries": [query] 
        }

        # --- [ NEW V6.1 SYSTEM PROMPT (แก้ไขเรื่อง Entity) ] ---
        system_prompt = f"""You are an expert Thai language interpreter, router, and query decomposer for a Nan province tourism guide AI.
Your task is to analyze a noisy user query.
You MUST return a JSON object with exactly these 5 keys: "corrected_query", "intent", "entity", "is_complex", "sub_queries".

1.  **corrected_query**: Reconstruct the query into a clear, natural Thai sentence.
2.  **intent**: Classify into ONE: "INFORMATIONAL", "PLAY_MUSIC", "SYSTEM_COMMAND", "SMALL_TALK".
3.  **entity**: 
    - If "PLAY_MUSIC", extract song/artist.
    - If "SYSTEM_COMMAND", extract app name.
    - If "SMALL_TALK", return null.
    - If "INFORMATIONAL" AND `is_complex: true`, return null.
    - If "INFORMATIONAL" AND `is_complex: false`, extract the SINGLE main topic (e.g., "วัดภูมินทร์", "ปู่ม่านย่าม่าน", "ดอยเสมอดาว"). If no single topic, return null.
4.  **is_complex**: (Boolean) Is this a complex question that requires multiple separate information retrievals? 
    - `true` if it compares items (A vs B), asks for multiple distinct topics (A and B), or has sequential logic.
    - `false` if it's a simple, single-topic question.
5.  **sub_queries**: (List of strings)
    - If `is_complex: false`, return a list containing only the `corrected_query`.
    - If `is_complex: true`, break the `corrected_query` down into the simplest possible sub-queries.

**EXAMPLES (Crucial):**
* Input: "วัด พูมิน ไปไง"
  Output: {{"corrected_query": "วัดภูมินทร์ไปยังไง", "intent": "INFORMATIONAL", "entity": "วัดภูมินทร์", "is_complex": false, "sub_queries": ["วัดภูมินทร์ไปยังไง"]}}
* Input: "ขอดูรูปปู่ม่านย่าม่าน"
  Output: {{"corrected_query": "ขอดูรูปปู่ม่านย่าม่าน", "intent": "INFORMATIONAL", "entity": "ปู่ม่านย่าม่าน", "is_complex": false, "sub_queries": ["ขอดูรูปปู่ม่านย่าม่าน"]}}
* Input: "วัดสวยๆ ในน่านมีที่ไหนบ้าง" (Simple, but no single entity)
  Output: {{"corrected_query": "วัดสวยๆ ในน่านมีที่ไหนบ้าง", "intent": "INFORMATIONAL", "entity": null, "is_complex": false, "sub_queries": ["วัดสวยๆ ในน่านมีที่ไหนบ้าง"]}}
* Input: "เปิดเพงเศร้าๆ น่อย"
  Output: {{"corrected_query": "เปิดเพลงเศร้าๆ หน่อย", "intent": "PLAY_MUSIC", "entity": "เพลงเศร้าๆ", "is_complex": false, "sub_queries": ["เปิดเพลงเศร้าๆ หน่อย"]}}
* Input: "สวัสดี"
  Output: {{"corrected_query": "สวัสดี", "intent": "SMALL_TALK", "entity": null, "is_complex": false, "sub_queries": ["สวัสดี"]}}
* Input: "วัดภูมินทร์กับวัดแช่แห้งต่างกันยังไง แล้ววัดไหนจอดรถง่ายกว่า?" (Complex)
  Output: {{"corrected_query": "วัดภูมินทร์กับวัดพระธาตุแช่แห้งแตกต่างกันยังไง และวัดไหนมีที่จอดรถสะดวกกว่า?", "intent": "INFORMATIONAL", "entity": null, "is_complex": true, "sub_queries": ["เปรียบเทียบ วัดภูมินทร์ และ วัดพระธาตุแช่แห้ง", "ที่จอดรถ วัดภูมินทร์", "ที่จอดรถ วัดพระธาตุแช่แห้ง"]}}
* Input: "ประวัติศาสตร์น่าน และ ชนเผ่าที่น่าสนใจ" (Complex)
  Output: {{"corrected_query": "ประวัติศาสตร์น่าน และ ชนเผ่าที่น่าสนใจ", "intent": "INFORMATIONAL", "entity": null, "is_complex": true, "sub_queries": ["ประวัติศาสตร์จังหวัดน่าน", "ชนเผ่าที่น่าสนใจในจังหวัดน่าน"]}}
"""
        # --- [ END OF NEW V6.1 SYSTEM PROMPT ] ---
        
        logging.info(f"✍️🧠 [Interpreter V6.1] Interpreting and decomposing (async): '{query}'")
        
        response_str = await self._get_groq_response(system_prompt, query)
        
        if not response_str:
            logging.warning("[Interpreter V6.1] API call failed. Using fallback.")
            return fallback_result

        try:
            result = json.loads(response_str)
            
            if all(k in result for k in ["corrected_query", "intent", "is_complex", "sub_queries"]):
                if "entity" not in result:
                    result["entity"] = None
                    
                logging.info(f"✅ [Interpreter V6.1] Result: {result}")
                return result
            else:
                logging.error(f"Missing required keys in response: {response_str}")
                raise ValueError("Missing required keys")
                
        except Exception as e:
            logging.error(f"❌ [Interpreter V6.1] Failed to parse JSON response: {e}. Response was: {response_str}")
            return fallback_result