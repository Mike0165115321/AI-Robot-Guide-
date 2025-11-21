# /core/ai_models/key_manager.py
import itertools
from typing import List
from core.config import settings
class KeyManager:
    def __init__(self, api_keys: List[str], service_name: str):
        self.keys = api_keys
        self.service_name = service_name
        if not self.keys:
            print(f"⚠️ WARNING: No API keys found for {self.service_name}. KeyManager will not function for this service.")
            self._key_cycler = itertools.cycle([])
        else:
            print(f"🔑 KeyManager for {self.service_name} initialized with {len(self.keys)} API key(s).")
            self._key_cycler = itertools.cycle(self.keys)
    def get_key(self) -> str | None:
        try:
            return next(self._key_cycler)
        except StopIteration:
            return None
gemini_key_manager = KeyManager(settings.GEMINI_API_KEYS, "Gemini")
groq_key_manager = KeyManager(settings.GROQ_API_KEYS, "Groq")