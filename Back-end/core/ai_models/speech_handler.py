import io
import os
import re
import logging
import asyncio
import tempfile
from groq import Groq
import edge_tts
from gtts import gTTS  # Fallback TTS
from pydub import AudioSegment  # สำหรับ speed up เสียง
from core.config import settings
from core.ai_models.key_manager import groq_key_manager

def sanitize_text_for_speech(text: str) -> str:
    import unicodedata
    
    # 1. ลบ URL / Links
    text = re.sub(r'https?://\S+', '', text)
    text = re.sub(r'www\.\S+', '', text)
    
    # 2. ลบ markdown headers (#)
    text = re.sub(r'^\s*#+\s*', '', text, flags=re.MULTILINE)
    
    # 3. ลบ bold/italic markdown
    text = re.sub(r'\*\*(.*?)\*\*', r'\1', text)  # **bold**
    text = re.sub(r'\*(.*?)\*', r'\1', text)      # *italic*
    text = re.sub(r'__(.*?)__', r'\1', text)      # __underline__
    text = re.sub(r'_(.*?)_', r'\1', text)        # _italic_
    
    # 4. ลบ markdown links [text](url)
    text = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', text)
    
    # 5. ลบ code blocks และ inline code
    text = re.sub(r'```[\s\S]*?```', '', text)    # code blocks
    text = re.sub(r'`([^`]+)`', r'\1', text)      # inline code
    
    # 6. ลบ {{IMAGE: xxx}} tags
    text = re.sub(r'\{\{IMAGE:[^}]+\}\}', '', text)
    
    # 7. ลบ emoji ทั้งหมด (Unicode emoji ranges)
    emoji_pattern = re.compile(
        "["
        "\U0001F300-\U0001F9FF"  # Miscellaneous Symbols and Pictographs, Emoticons, etc.
        "\U00002702-\U000027B0"  # Dingbats
        "\U0001F600-\U0001F64F"  # Emoticons
        "\U0001F680-\U0001F6FF"  # Transport and Map Symbols
        "\U0001F1E0-\U0001F1FF"  # Flags
        "\U00002500-\U00002BEF"  # Various symbols
        "\U0001FA00-\U0001FAFF"  # Chess, Extended-A symbols
        "\U00002600-\U000026FF"  # Miscellaneous symbols
        "]+", 
        flags=re.UNICODE
    )
    text = emoji_pattern.sub('', text)
    
    # 8. ลบ bullets และสัญลักษณ์พิเศษ
    text = text.replace('▹', '')
    text = text.replace('•', '')
    text = text.replace('→', '')
    text = text.replace('←', '')
    text = text.replace('↓', '')
    text = text.replace('↑', '')
    text = text.replace('...', '. ')
    text = text.replace('…', '. ')
    
    # 9. แปลง _ และ - ให้เป็นช่องว่าง (ป้องกัน TTS ข้าม)
    text = text.replace('_', ' ')
    text = re.sub(r'(?<=[a-zA-Zก-ฮ])-(?=[a-zA-Zก-ฮ])', ' ', text)  # แปลง - ระหว่างคำ
    
    # 10. ลบอักขระพิเศษที่ TTS อ่านไม่ได้
    text = re.sub(r'[#\*\[\]\(\)\{\}\|\\/<>@&$%^~`]', '', text)
    
    # 11. ลบ whitespace ซ้ำและ trim
    text = re.sub(r'\s+', ' ', text).strip()
    
    # 12. ลบบรรทัดว่าง
    text = re.sub(r'\n\s*\n', '\n', text)
    
    return text


local_whisper_model = None

VOICE_MAP = {
    "th": ["th-TH-PremwadeeNeural", "th-TH-NiwatNeural"],
    "en": ["en-US-JennyNeural", "en-US-GuyNeural"],
    "zh": ["zh-CN-XiaoxiaoNeural", "zh-CN-YunxiNeural"],
    "ja": ["ja-JP-NanamiNeural", "ja-JP-KeitaNeural"],
    "hi": ["hi-IN-SwaraNeural", "hi-IN-MadhurNeural"],
    "ru": ["ru-RU-SvetlanaNeural", "ru-RU-DmitryNeural"],
    "ms": ["ms-MY-YasminNeural", "ms-MY-OsmanNeural"],
}

class SpeechHandler:
    def __init__(self):
        logging.info("🎤 [Speech] กำลังเริ่มต้น SpeechHandler (หลัก: Groq, สำรอง: Local)")
        try:
            from core.services.language_detector import language_detector
            self.lang_detector = language_detector
        except ImportError:
            self.lang_detector = None
            logging.warning("⚠️ [Speech] ไม่สามารถใช้งาน Language detector ได้")
        
    def _get_groq_client(self):
        api_key = groq_key_manager.get_key()
        if api_key:
            return Groq(api_key=api_key)
        return None

    def _transcribe_with_groq(self, file_path: str) -> str:
        client = self._get_groq_client()
        if not client:
            raise Exception("No Groq API Key available")

        with open(file_path, "rb") as file:
            transcription = client.audio.transcriptions.create(
                file=(file_path, file.read()),
                model=settings.GROQ_WHISPER_MODEL, 
                response_format="json",
                language="th",
                temperature=0.0
            )
        return transcription.text

    def _transcribe_with_local(self, file_path: str) -> str:
        global local_whisper_model
        import whisper
        
        if local_whisper_model is None:
            model_size = settings.WHISPER_MODEL_SIZE
            logging.info(f"🔄 [Speech] กำลังโหลด Local Whisper '{model_size}' (ระบบสำรอง)...")
            local_whisper_model = whisper.load_model(model_size, device=settings.DEVICE)
        
        logging.info("🐢 [Speech] กำลังแปลงเสียงเป็นข้อความด้วย Local Whisper...")
        result = local_whisper_model.transcribe(file_path, language="th")
        return result.get('text', '').strip()

    async def transcribe_audio_bytes(self, audio_bytes: bytes) -> str:
        if not audio_bytes: return ""
        
        with tempfile.NamedTemporaryFile(delete=False, suffix=".webm") as temp_file:
            temp_file.write(audio_bytes)
            temp_file_path = temp_file.name

        try:
            logging.info("🚀 [Speech] กำลังลองใช้ Groq Whisper...")
            text = await asyncio.to_thread(self._transcribe_with_groq, temp_file_path)
            logging.info(f"✅ [Speech] ผลลัพธ์จาก Groq: '{text}'")
            return text

        except Exception as e:
            logging.warning(f"⚠️ [Speech] Groq ล้มเหลว ({e}). กำลังเปลี่ยนไปใช้ Local Whisper...")
            try:
                text = await asyncio.to_thread(self._transcribe_with_local, temp_file_path)
                logging.info(f"✅ [Speech] ผลลัพธ์จาก Local: '{text}'")
                return text
            except Exception as local_e:
                logging.error(f"❌ [Speech] การแปลงเสียงเป็นข้อความล้มเหลวทั้งหมด: {local_e}")
                return ""
        finally:
            if os.path.exists(temp_file_path):
                try:
                    os.remove(temp_file_path)
                except:
                    pass

    async def synthesize_speech_stream(self, text: str):
        """
        Async Generator that yields audio chunks (bytes).
        - Uses Edge TTS by default (streaming with sentence buffering).
        - Falls back to gTTS (yields single full chunk).
        """
        if not text.strip():
            return

        clean_text = sanitize_text_for_speech(text)
        if not clean_text.strip():
            clean_text = "ขอโทษค่ะ ไม่สามารถอ่านข้อความได้"

        logging.info(f"🗣️  [TTS Stream] เริ่มสังเคราะห์เสียง: '{clean_text[:50]}...'")

        # 🌐 Detect language
        detected_lang = "th"
        if self.lang_detector:
            detected_lang = self.lang_detector.detect(text)
            logging.info(f"🌐 [TTS] ภาษา: {detected_lang}")

        voices_to_try = VOICE_MAP.get(detected_lang, VOICE_MAP["th"])
        
        # ========== Try Edge TTS (Streaming) ==========
        for voice in voices_to_try:
            try:
               logging.info(f"🚀 [TTS Stream] Edge TTS: {voice}")
               communicate = edge_tts.Communicate(clean_text, voice, rate="-10%")
               buffer = io.BytesIO()
               MIN_CHUNK_SIZE = 16 * 1024  # 16KB ~ 1 second of audio
               
               async for chunk in communicate.stream():
                   if chunk["type"] == "audio":
                       buffer.write(chunk["data"])
                       if buffer.tell() >= MIN_CHUNK_SIZE:
                           buffer.seek(0)
                           yield buffer.read()
                           buffer = io.BytesIO() # Reset buffer
                           
               # Yield remaining
               if buffer.tell() > 0:
                   buffer.seek(0)
                   yield buffer.read()
                   
               logging.info(f"✅ [TTS Stream] สำเร็จ (Edge TTS)")
               return # Success, exit function
               
            except Exception as e:
                logging.error(f"❌ [TTS Stream] Edge TTS Stream Error: {e}")
                continue # Try next voice

        # ========== Fallback to gTTS (One-shot) ==========
        logging.warning("⚠️ [TTS Stream] Edge TTS หมดทุกเสียง -> ใช้ gTTS สำรอง (ไม่ Stream)")
        try:
            # Re-use existing single-shot logic but yield it
            full_audio = await self.synthesize_speech_to_bytes(text) # Use the existing method logic (copied inside or called)
            if full_audio:
                yield full_audio
            return
        except Exception as e:
             logging.error(f"❌ [TTS Stream] gTTS Fallback Failed: {e}")
             
    # Keep original method for compatibility (lazy wrapper)
    async def synthesize_speech_to_bytes(self, text: str) -> bytes:
        chunks = []
        async for chunk in self.synthesize_speech_stream(text):
            chunks.append(chunk)
        return b"".join(chunks)

speech_handler_instance = SpeechHandler()