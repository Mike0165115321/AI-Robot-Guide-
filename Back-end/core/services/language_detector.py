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
    
    # Supported languages with their codes and names
    SUPPORTED_LANGUAGES = {
        "th": {"name": "Thai", "native": "ไทย", "tts_code": "th-TH"},
        "en": {"name": "English", "native": "English", "tts_code": "en-US"},
        "zh": {"name": "Chinese", "native": "中文", "tts_code": "zh-CN"},
        "ja": {"name": "Japanese", "native": "日本語", "tts_code": "ja-JP"},
        "hi": {"name": "Hindi", "native": "हिन्दी", "tts_code": "hi-IN"},
        "ru": {"name": "Russian", "native": "Русский", "tts_code": "ru-RU"},
        "ms": {"name": "Malay", "native": "Bahasa Melayu", "tts_code": "ms-MY"},
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
    
    def detect(self, text: str) -> str:
        """
        ตรวจจับภาษาจาก text
        ถ้าเจอหลายภาษาผสม (mixed language) → fallback to English
        
        Args:
            text: ข้อความที่ต้องการตรวจจับ
            
        Returns:
            language code (th, en, zh, ja, hi, ru, ms)
        """
        if not text or len(text.strip()) < 3:
            return self.DEFAULT_LANG
        
        if not LANGDETECT_AVAILABLE:
            logging.warning("⚠️ langdetect ไม่พร้อมใช้งาน ใช้ภาษาเริ่มต้น")
            return self.DEFAULT_LANG
        
        try:
            from langdetect import detect_langs
            
            # ตรวจจับหลายภาษาพร้อม confidence
            results = detect_langs(text)
            
            if len(results) == 0:
                return self.DEFAULT_LANG
            
            top_lang = results[0]
            top_code = self.LANG_MAP.get(top_lang.lang, None)
            
            # 🌐 Mixed Language Rule:
            # ถ้า confidence < 0.8 หรือเจอมากกว่า 1 ภาษาที่ confidence > 0.2
            # → Fallback to English (ความเป็นกลาง)
            is_mixed = False
            if top_lang.prob < 0.7:
                is_mixed = True
            elif len(results) > 1 and results[1].prob > 0.25:
                is_mixed = True
            
            if is_mixed:
                print(f"🌐 ═══════════════════════════════════════════")
                print(f"🌐 [MIXED LANGUAGE] Detected multiple languages")
                print(f"🌐 [MIXED LANGUAGE] Top: {top_lang.lang} ({top_lang.prob:.2f})")
                if len(results) > 1:
                    print(f"🌐 [MIXED LANGUAGE] 2nd: {results[1].lang} ({results[1].prob:.2f})")
                print(f"🌐 [MIXED LANGUAGE] → Fallback to English (neutral)")
                print(f"🌐 ═══════════════════════════════════════════")
                logging.info(f"🌐 [Language] ตรวจพบภาษาผสม → ใช้ภาษาอังกฤษเป็นค่ากลาง")
                return "en"
            
            # Single language detected with high confidence
            if top_code:
                lang_name = self.SUPPORTED_LANGUAGES.get(top_code, {}).get("name", top_code)
                print(f"🌐 ═══════════════════════════════════════════")
                print(f"🌐 [LANGUAGE DETECTED] {top_lang.lang} → {top_code} ({lang_name})")
                print(f"🌐 [CONFIDENCE] {top_lang.prob:.2f}")
                print(f"🌐 ═══════════════════════════════════════════")
                logging.info(f"🌐 [Language] ตรวจพบภาษา: {top_lang.lang} → {top_code}")
                return top_code
            else:
                logging.info(f"🌐 [Language] ภาษาที่ตรวจพบ '{top_lang.lang}' ไม่รองรับ ใช้ภาษาเริ่มต้น")
                return self.DEFAULT_LANG
                
        except LangDetectException as e:
            logging.warning(f"⚠️ [Language] การตรวจจับล้มเหลว: {e}")
            return self.DEFAULT_LANG
        except Exception as e:
            logging.error(f"❌ [Language] ข้อผิดพลาด: {e}")
            return self.DEFAULT_LANG
    
    def detect_with_confidence(self, text: str) -> Tuple[str, float]:
        """
        ตรวจจับภาษาพร้อม confidence score
        
        Returns:
            (language_code, confidence)
        """
        if not LANGDETECT_AVAILABLE:
            return (self.DEFAULT_LANG, 0.0)
        
        try:
            from langdetect import detect_langs
            results = detect_langs(text)
            if results:
                top = results[0]
                lang_code = self.LANG_MAP.get(top.lang, self.DEFAULT_LANG)
                return (lang_code, top.prob)
            return (self.DEFAULT_LANG, 0.0)
        except:
            return (self.DEFAULT_LANG, 0.0)
    
    def get_language_info(self, lang_code: str) -> dict:
        """
        ดึงข้อมูลภาษา
        
        Returns:
            {"name": "Thai", "native": "ไทย", "tts_code": "th-TH"}
        """
        return self.SUPPORTED_LANGUAGES.get(lang_code, self.SUPPORTED_LANGUAGES[self.DEFAULT_LANG])
    
    def get_tts_code(self, lang_code: str) -> str:
        """
        ดึง TTS language code
        """
        info = self.get_language_info(lang_code)
        return info["tts_code"]
    
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
            content = prompt_path.read_text(encoding="utf-8")
            logging.info(f"📝 [Prompt] โหลดแล้ว: {prompt_path}")
            return content
        
        # Fallback to Thai
        fallback_path = self.prompts_dir / "th" / f"{prompt_name}.txt"
        if fallback_path.exists():
            logging.warning(f"⚠️ [Prompt] ไม่พบ {prompt_path} ใช้ภาษาไทยเป็น fallback")
            return fallback_path.read_text(encoding="utf-8")
        
        # Final fallback - return empty
        logging.error(f"❌ [Prompt] ไม่พบ prompt สำหรับ: {prompt_name}")
        return ""
    
    def is_supported(self, lang_code: str) -> bool:
        """ตรวจสอบว่าภาษาได้รับการสนับสนุน"""
        return lang_code in self.SUPPORTED_LANGUAGES
    
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
