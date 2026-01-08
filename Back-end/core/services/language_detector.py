# Back-end/core/services/language_detector.py
"""
🌐 Language Detection Service
ตรวจจับภาษาจาก user input - ใช้ได้ทั้ง LLM, TTS, STT

รองรับ 7 ภาษา:
- th: Thai (ไทย)
- en: English (อังกฤษ)
- zh: Chinese (จีน)
- ja: Japanese (ญี่ปุ่น)
- hi: Hindi (อินเดีย)
- ru: Russian (รัสเซีย)
- ms: Malay (มาเลเซีย)
"""

import logging
from typing import Optional, Tuple
from pathlib import Path

# Try to import langdetect
try:
    from langdetect import detect, LangDetectException
    from langdetect import DetectorFactory
    DetectorFactory.seed = 0  # ให้ผลลัพธ์คงที่
    LANGDETECT_AVAILABLE = True
except ImportError:
    LANGDETECT_AVAILABLE = False
    logging.warning("⚠️ ไม่พบ langdetect กรุณาติดตั้ง: pip install langdetect")


class LanguageDetector:
    """
    Central Language Detection Service
    ใช้ร่วมกันทั้ง LLM, TTS, STT
    """
    
    # Supported languages with their codes, names, and REGEX patterns (for Frontend)
    # Note: Regex strings are sent to frontend to construct new RegExp()
    # Supported languages with their codes, names, and REGEX patterns (for Frontend)
    # Note: Regex strings are sent to frontend to construct new RegExp()
    SUPPORTED_LANGUAGES = {
        "th": {
            "name": "Thai", "native": "ไทย", 
            "tts_codes": ["th-TH-PremwadeeNeural", "th-TH-NiwatNeural"], 
            "regex": r"[\u0E00-\u0E7F]"
        },
        "en": {
            "name": "English", "native": "English", 
            "tts_codes": ["en-US-JennyNeural", "en-US-GuyNeural"], 
            "regex": r"[a-zA-Z]"
        },
        "zh": {
            "name": "Chinese", "native": "中文", 
            "tts_codes": ["zh-CN-XiaoxiaoNeural", "zh-CN-YunxiNeural"], 
            "regex": r"[\u4E00-\u9FFF]"
        },
        "ja": {
            "name": "Japanese", "native": "日本語", 
            "tts_codes": ["ja-JP-NanamiNeural", "ja-JP-KeitaNeural"], 
            "regex": r"[\u3040-\u309F\u30A0-\u30FF]"
        },
        "hi": {
            "name": "Hindi", "native": "हिन्दी", 
            "tts_codes": ["hi-IN-SwaraNeural", "hi-IN-MadhurNeural"], 
            "regex": r"[\u0900-\u097F]"
        },
        "ru": {
            "name": "Russian", "native": "Русский", 
            "tts_codes": ["ru-RU-SvetlanaNeural", "ru-RU-DmitryNeural"], 
            "regex": r"[\u0400-\u04FF]"
        },
        "ms": {
            "name": "Malay", "native": "Bahasa Melayu", 
            "tts_codes": ["ms-MY-YasminNeural", "ms-MY-OsmanNeural"], 
            "regex": r"[a-zA-Z]"
        }, 
    }
    
    DEFAULT_LANG = "th"
    
    # Mapping from langdetect codes to our codes
    LANG_MAP = {
        "th": "th",
        "en": "en",
        "zh-cn": "zh",
        "zh-tw": "zh",
        "zh": "zh",
        "ja": "ja",
        "hi": "hi",
        "ru": "ru",
        "ms": "ms",
        "id": "ms",  # Indonesian is similar to Malay
    }
    
    def __init__(self):
        self.prompts_dir = Path(__file__).parent.parent.parent / "prompts"
        self._active_languages = {}
        self.scan_active_languages() # Initial scan
    
    def scan_active_languages(self):
        """
        Scan prompts directory to see which languages are enabled.
        Source of Truth: Folder existing in prompts/
        """
        self._active_languages = {}
        if not self.prompts_dir.exists():
            logging.warning(f"⚠️ Prompts directory not found: {self.prompts_dir}")
            return

        for item in self.prompts_dir.iterdir():
            if item.is_dir():
                lang_code = item.name
                if lang_code in self.SUPPORTED_LANGUAGES:
                    # It's a supported language AND has a folder -> Activate it
                    config = self.SUPPORTED_LANGUAGES[lang_code].copy()
                    config["code"] = lang_code
                    self._active_languages[lang_code] = config
    
    def get_active_languages_config(self) -> list:
        """
        Return list of active language configs for Frontend
        """
        # Re-scan to catch new folders without restart (optional, but good for dynamic nature)
        self.scan_active_languages() 
        return list(self._active_languages.values())

    def detect(self, text: str) -> str:
        """
        ตรวจจับภาษาจาก text
        ถ้าเจอหลายภาษาผสม (mixed language) → fallback to English
        """
        if not text or len(text.strip()) < 3:
            return self.DEFAULT_LANG
        
        if not LANGDETECT_AVAILABLE:
            return self.DEFAULT_LANG
        
        try:
            from langdetect import detect_langs
            results = detect_langs(text)
            
            if len(results) == 0: return self.DEFAULT_LANG
            
            top_lang = results[0]
            top_code = self.LANG_MAP.get(top_lang.lang, None)
            
            # Check if this code is actually ACTIVE (has prompt folder)
            if top_code and top_code not in self._active_languages:
                 logging.info(f"🌐 [Language] Detected {top_code} but no prompt folder. Fallback to default.")
                 return self.DEFAULT_LANG

            # 🌐 Mixed Language Rule
            is_mixed = False
            if top_lang.prob < 0.7: is_mixed = True
            elif len(results) > 1 and results[1].prob > 0.25: is_mixed = True
            
            if is_mixed:
                # If mixed, fallback to English IF English is active, else Default
                return "en" if "en" in self._active_languages else self.DEFAULT_LANG
            
            if top_code:
                return top_code
            else:
                return self.DEFAULT_LANG
                
        except Exception:
            return self.DEFAULT_LANG
    
    def detect_with_confidence(self, text: str) -> Tuple[str, float]:
        lang, prob = self._detect_raw(text)
        if lang in self._active_languages:
            return lang, prob
        return self.DEFAULT_LANG, 0.0

    def _detect_raw(self, text):
        if not LANGDETECT_AVAILABLE: return self.DEFAULT_LANG, 0.0
        try:
            from langdetect import detect_langs
            results = detect_langs(text)
            if results:
                top = results[0]
                code = self.LANG_MAP.get(top.lang, self.DEFAULT_LANG)
                return code, top.prob
        except: pass
        return self.DEFAULT_LANG, 0.0
    
    def get_language_info(self, lang_code: str) -> dict:
        """
        ดึงข้อมูลภาษา
        
        Returns:
            {"name": "Thai", "native": "ไทย", "tts_code": "th-TH"}
        """
        return self.SUPPORTED_LANGUAGES.get(lang_code, self.SUPPORTED_LANGUAGES[self.DEFAULT_LANG])
    
    def get_tts_voices(self, lang_code: str) -> list:
        """
        ดึงรายการเสียง TTS ทั้งหมด (Primary + Backup)
        """
        info = self.get_language_info(lang_code)
        # Handle backward compatibility if dict still uses old key (unlikely after edit, but safe)
        return info.get("tts_codes", [info.get("tts_code")])

    def get_tts_code(self, lang_code: str) -> str:
        """
        ดึง TTS language code (ตัวแรกสุด)
        """
        voices = self.get_tts_voices(lang_code)
        return voices[0] if voices else "th-TH-PremwadeeNeural"
    
    def get_prompt(self, prompt_name: str, lang_code: str) -> str:
        """
        โหลด prompt จากไฟล์ตามภาษา
        
        Args:
            prompt_name: ชื่อ prompt เช่น "persona"
            lang_code: รหัสภาษา เช่น "en"
            
        Returns:
            เนื้อหา prompt
        """
        # Try requested language first
        prompt_path = self.prompts_dir / lang_code / f"{prompt_name}.txt"
        
        if prompt_path.exists():
            return prompt_path.read_text(encoding="utf-8")
        
        # Fallback to Thai
        fallback_path = self.prompts_dir / "th" / f"{prompt_name}.txt"
        if fallback_path.exists():
            return fallback_path.read_text(encoding="utf-8")
        
        # Final fallback - return empty
        logging.error(f"❌ [Prompt] ไม่พบ prompt สำหรับ: {prompt_name}")
        return ""
    
    def is_supported(self, lang_code: str) -> bool:
        """ตรวจสอบว่าภาษาได้รับการสนับสนุน"""
        return lang_code in self._active_languages
    
    def list_supported_languages(self) -> list:
        """แสดงรายการภาษาที่รองรับ"""
        return list(self.SUPPORTED_LANGUAGES.keys())


# Global singleton instance
language_detector = LanguageDetector()


# Convenience functions for easy import
def detect_language(text: str) -> str:
    """ตรวจจับภาษา - ใช้ง่ายๆ"""
    return language_detector.detect(text)

def get_prompt_for_language(prompt_name: str, text: str) -> str:
    """ตรวจจับภาษาแล้วโหลด prompt ให้"""
    lang = language_detector.detect(text)
    return language_detector.get_prompt(prompt_name, lang)

def get_tts_code_for_text(text: str) -> str:
    """ตรวจจับภาษาแล้วส่ง TTS code"""
    lang = language_detector.detect(text)
    return language_detector.get_tts_code(lang)
