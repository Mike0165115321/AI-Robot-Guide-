import logging
import json
import os
import asyncio
from typing import Dict, Any, Optional

# 🩹 Workaround for Protobuf issue in ancient Assistant SDK
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import grpc
import google.auth.transport.grpc
import google.auth.transport.requests
import google.oauth2.credentials

from google.assistant.embedded.v1alpha2 import embedded_assistant_pb2
from google.assistant.embedded.v1alpha2 import embedded_assistant_pb2_grpc

# Configure Logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

ASSISTANT_API_ENDPOINT = 'embeddedassistant.googleapis.com'
DEFAULT_PROJECT_ID = "still-toolbox-479616-e4" # Hardcoded from creds
DEFAULT_MODEL_ID = "nan-robot-model-v1"
DEFAULT_DEVICE_ID = "nan-robot-device-v1"

class FrontlineHandler:
    """
    FrontlineHandler: The Gatekeeper AI (Powered by Google Assistant API)
    
    Strictly uses Google Assistant SDK/API via gRPC to process requests.
    """
    
    def __init__(self):
        # Try to find credentials in current dir or Back-end/ dir
        creds_path = "assistant_credentials.json"
        if not os.path.exists(creds_path):
            creds_path = "Back-end/assistant_credentials.json"
            
        self.credentials_file = os.getenv("GOOGLE_ASSISTANT_CREDENTIALS", creds_path)
        self.project_id = os.getenv("GOOGLE_CLOUD_PROJECT_ID", DEFAULT_PROJECT_ID)
        self.model_id = DEFAULT_MODEL_ID
        self.device_id = DEFAULT_DEVICE_ID
        self.service = None 
        logger.info("🤖 [Frontline] Google Assistant Handler Initialized")

    def _get_credentials(self):
        try:
            with open(self.credentials_file, 'r') as f:
                creds_data = json.load(f)
            
            # Construct Credentials object from JSON
            cr = google.oauth2.credentials.Credentials(
                token=None, # access token (will be refreshed)
                refresh_token=creds_data.get('refresh_token'),
                token_uri=creds_data.get('token_uri'),
                client_id=creds_data.get('client_id'),
                client_secret=creds_data.get('client_secret'),
                scopes=creds_data.get('scopes')
            )
            return cr
        except Exception as e:
            logger.error(f"❌ [Frontline] Failed to load credentials: {e}")
            return None

    # Language Mapping (Frontend Code -> Google Assistant Locale)
    LANGUAGE_MAP = {
        "th": "th-TH",
        "en": "en-US",
        "ja": "ja-JP",
        "ko": "ko-KR",
        "zh": "zh-CN",
        "fr": "fr-FR",
        "de": "de-DE"
    }

    async def process_query(self, query: str, language_code: str = "th") -> Dict[str, Any]:
        """
        Process the query using Google Assistant API (gRPC).
        Supports dynamic language selection.
        """
        # Map simple code (th) to full locale (th-TH)
        # Default to th-TH if not found, or use as-is if it looks like a locale
        locale = self.LANGUAGE_MAP.get(language_code, language_code if "-" in language_code else "th-TH")
        
        logger.info(f"🎤 [Frontline] Sending to Google Assistant ({locale}): {query}")

    # Embedded SDK often supports limited locales.
    # We explicitly list tested/working ones to avoid "INVALID_ARGUMENT".
    SUPPORTED_LOCALES = {
        "en-US", "en-GB", "en-AU", "en-CA",
        "ja-JP", 
        "de-DE", 
        "fr-FR"
    }

    async def process_query(self, query: str, language_code: str = "th") -> Dict[str, Any]:
        """
        Process the query using Google Assistant API (gRPC).
        Supports dynamic language selection.
        """
        # Map simple code (th) to full locale (th-TH)
        # Default to th-TH if not found, or use as-is if it looks like a locale
        locale = self.LANGUAGE_MAP.get(language_code, language_code if "-" in language_code else "th-TH")
        
        logger.info(f"🎤 [Frontline] Processing: '{query}' (Lang: {locale})")

        # --- STEP 1: Check Manual Fallbacks (Local Intent) ---
        # Do this FIRST for unsupported languages OR to override Google
        # This ensures "เปิดเพลง" works even if we skip Google for Thai
        manual_result = self._check_manual_intents(query)
        
        # --- STEP 2: Check SDK Support ---
        if locale not in self.SUPPORTED_LOCALES:
             if manual_result:
                 logger.info(f"⚡ [Frontline] Language '{locale}' unsupported, but matched Manual Intent.")
                 return manual_result
             
             logger.info(f"⚠️ [Frontline] Language '{locale}' not supported by Embedded SDK. Skipping API.")
             return {"intent": "RAG_QUERY", "reply": None}

        # --- STEP 3: Call Google Assistant API ---
        creds = self._get_credentials()
        if not creds:
             logger.warning("⚠️ [Frontline] No valid credentials. Falling back to RAG.")
             return {"intent": "RAG_QUERY", "reply": None}
        
        try:
            # Refresh request
            http_request = google.auth.transport.requests.Request()
            creds.refresh(http_request)

            # Create gRPC Channel
            channel = google.auth.transport.grpc.secure_authorized_channel(
                creds, http_request, ASSISTANT_API_ENDPOINT)
            assistant = embedded_assistant_pb2_grpc.EmbeddedAssistantStub(channel)
            
            # Create Request Iterator
            def request_generator():
                assist_config = embedded_assistant_pb2.AssistConfig(
                    text_query=query,
                    audio_out_config=embedded_assistant_pb2.AudioOutConfig(
                        encoding='LINEAR16',
                        sample_rate_hertz=16000,
                        volume_percentage=0, 
                    ),
                    device_config=embedded_assistant_pb2.DeviceConfig(
                        device_id=self.device_id,
                        device_model_id=self.model_id,
                    ),
                    dialog_state_in=embedded_assistant_pb2.DialogStateIn(
                        language_code=locale,
                        conversation_state=b'', 
                        is_new_conversation=True,
                    ),
                )
                yield embedded_assistant_pb2.AssistRequest(config=assist_config)

            # Send Request
            response_text = ""
            raw_responses = assistant.Assist(request_generator(), timeout=20) 
            
            for resp in raw_responses:
                if resp.dialog_state_out.supplemental_display_text:
                    response_text += resp.dialog_state_out.supplemental_display_text
                
            logger.info(f"🤖 [Assistant] Reply: {response_text}")
            
            if not response_text:
                 # Silent response -> Check Manual Fallback
                 if manual_result:
                     return manual_result
                 return {"intent": "RAG_QUERY", "reply": None}

            # --- INTENT MAPPING ---
            # 1. Music
            if any(w in response_text for w in ["เปิดเพลง", "Playing music", "กำลังเล่น"]):
                return {"intent": "CMD_MUSIC", "reply": response_text, "metadata": {"source": "google_assistant"}}
            
            # 2. News
            if "ข่าว" in response_text or "Headlines" in response_text:
                 return {"intent": "CMD_NEWS", "reply": response_text, "metadata": {"source": "google_assistant"}}
            
            # 3. Unknown/Error
            if "ไม่เข้าใจ" in response_text or "ขอโทษ" in response_text:
                 return {"intent": "RAG_QUERY", "reply": None}
                 
            # Default: Small Talk
            return {
                "intent": "SMALL_TALK",
                "reply": response_text,
                "confidence": 1.0,
                "metadata": {"source": "google_assistant"}
            }

        except grpc.RpcError as e:
            # Check for INVALID_ARGUMENT (Language Error)
            if e.code() == grpc.StatusCode.INVALID_ARGUMENT:
                logger.warning(f"⚠️ [Frontline] Language '{locale}' rejected by Google. Proceeding to RAG.")
            else:
                logger.error(f"❌ [Frontline] gRPC Error: {e.code()} - {e.details()}")
            return {"intent": "RAG_QUERY", "reply": None}
            
        except Exception as e:
            logger.error(f"❌ [Frontline] Unexpected Error: {e}")
            return {"intent": "RAG_QUERY", "reply": None}

    def _check_manual_intents(self, query: str) -> Optional[Dict[str, Any]]:
        """
        Check for hardcoded intents (Music/News) locally.
        Useful for unsupported languages or silent fallbacks.
        """
        query_lower = query.lower() if query else ""
        
        # Fallback Music
        if any(w in query_lower for w in ["เปิดเพลง", "ฟังเพลง", "play music"]):
           logger.info("⚡ [Frontline] Manual Match: Music Intent -> Passing to RAG for YouTube Search")
           # Don't return CMD_MUSIC here because we lack the song list.
           # Let RAG Orchestrator handle the YouTube search & payload.
           return {"intent": "RAG_QUERY", "reply": None}
        
        # Fallback News
        if any(w in query_lower for w in ["ข่าว", "news", "เล่าข่าว"]):
            logger.info("⚡ [Frontline] Manual Match: News Intent")
            return {"intent": "CMD_NEWS", "reply": "ได้เลยครับ มาฟังข่าวกัน", "metadata": {"source": "manual_fallback"}}

        # Fallback Navigation (Force RAG to handle Map)
        if any(w in query_lower for w in ["นำทาง", "พาไป", "เส้นทาง", "navigate", "route", "map", "แผนที่"]):
            logger.info("⚡ [Frontline] Manual Match: Navigation Intent -> Passing to RAG for Map")
            return {"intent": "RAG_QUERY", "reply": None}

        return None

# Singleton
frontline_handler = FrontlineHandler()
