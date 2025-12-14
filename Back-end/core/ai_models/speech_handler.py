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

    async def synthesize_speech_to_bytes(self, text: str) -> bytes:
        if not text.strip():
            raise ValueError("Input text for TTS cannot be empty.")

        clean_text = sanitize_text_for_speech(text)
        
        # Validate cleaned text isn't empty
        if not clean_text.strip():
            logging.warning("⚠️ [TTS] ข้อความว่างเปล่าหลังจากการทำความสะอาด ใช้ข้อความสำรองแทน")
            clean_text = "ขอโทษค่ะ ไม่สามารถอ่านข้อความได้"
        
        print(f"🗣️  [TTS] กำลังสังเคราะห์เสียงสำหรับ: '{clean_text[:50]}...'")
        
        # 🌐 Detect language and select appropriate voices
        detected_lang = "th"  # default
        if self.lang_detector:
            detected_lang = self.lang_detector.detect(text)
            logging.info(f"🌐 [TTS] ภาษาที่ตรวจพบ: {detected_lang}")
        
        # Get voices for detected language
        voices_to_try = VOICE_MAP.get(detected_lang, VOICE_MAP["th"])
        logging.info(f"🔊 [TTS] กำลังใช้เสียง: {voices_to_try}")
        
        # ========== Try Edge TTS (Primary - Microsoft) ==========
        for voice in voices_to_try:
            try:
                logging.info(f"🚀 [TTS] กำลังลองใช้เสียง Edge TTS: {voice}")
                communicate = edge_tts.Communicate(clean_text, voice, rate="-10%")
                
                mp3_buffer = io.BytesIO()
                async for chunk in communicate.stream():
                    if chunk["type"] == "audio":
                        mp3_buffer.write(chunk["data"])
                
                if mp3_buffer.tell() == 0:
                    logging.warning(f"⚠️ [TTS] ไม่ได้รับข้อมูลเสียงจากเสียง {voice}")
                    continue
                    
                mp3_buffer.seek(0)
                logging.info(f"✅ [TTS] สำเร็จด้วยเสียง Edge TTS: {voice}")
                return mp3_buffer.read()

            except Exception as e:
                logging.warning(f"⚠️ [TTS] เสียง Edge TTS {voice} ล้มเหลว: {e}")
                continue
        
        # ========== Fallback to gTTS (Google TTS) ==========
        logging.warning("⚠️ [TTS] Edge TTS ล้มเหลว กำลังลองใช้ gTTS สำรอง...")
        try:
            from gtts import gTTS
            
            tts = gTTS(text=clean_text, lang='th', slow=False)
            mp3_buffer = io.BytesIO()
            tts.write_to_fp(mp3_buffer)
            mp3_buffer.seek(0)
            
            # --- Friend's Feature: Speed up 1.25x using pydub ---
            try:
                logging.info("⚡ [TTS] กำลังเร่งความเร็ว 1.25x ให้กับผลลัพธ์ gTTS...")
                audio = AudioSegment.from_mp3(mp3_buffer)
                faster_audio = audio.speedup(playback_speed=1.25)
                
                output_buffer = io.BytesIO()
                faster_audio.export(output_buffer, format='mp3')
                output_buffer.seek(0)
                
                logging.info("✅ [TTS] สำเร็จด้วย gTTS สำรอง (ความเร็ว 1.25x)")
                return output_buffer.read()
            except Exception as pydub_error:
                logging.warning(f"⚠️ [TTS] การเร่งความเร็วด้วย pydub ล้มเหลว คืนค่า gTTS ความเร็วปกติ: {pydub_error}")
                mp3_buffer.seek(0)
                return mp3_buffer.read()
            
        except Exception as gtts_error:
            logging.error(f"❌ [TTS] วิธีการ TTS ทั้งหมดล้มเหลว: {gtts_error}", exc_info=True)
            raise RuntimeError("ไม่สามารถสังเคราะห์เสียงด้วยทั้ง edge-tts และ gTTS")

speech_handler_instance = SpeechHandler()