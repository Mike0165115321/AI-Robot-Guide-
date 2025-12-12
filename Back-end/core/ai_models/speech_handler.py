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
    # ลบ markdown headers (#)
    text = re.sub(r'^\s*#+\s*', '', text, flags=re.MULTILINE)
    # ลบ bold (**text**)
    text = re.sub(r'\*\*(.*?)\*\*', r'\1', text)
    # ลบ italic (*text*)
    text = re.sub(r'\*(.*?)\*', r'\1', text)
    # ลบ underline markdown (__text__)
    text = re.sub(r'__(.*?)__', r'\1', text)
    
    # [FIX] แปลง _ และ - ให้เป็นช่องว่าง (ป้องกัน TTS ข้าม)
    text = text.replace('_', ' ')
    text = text.replace('-', ' ')
    
    # ลบ emoji ทั่วไปที่อาจทำให้ TTS พัง
    text = re.sub(r'[💡🎵🗺️📸😊❓🔢🛕⛰️🐘🚶✨🎉💕😢]', '', text)
    
    # ลบ bullets และสัญลักษณ์พิเศษ
    text = text.replace('▹', '')
    text = text.replace('•', '')
    text = text.replace('...', '. ')
    
    # ลบ whitespace ซ้ำ
    text = re.sub(r'\s+', ' ', text).strip()
    return text

local_whisper_model = None

class SpeechHandler:
    def __init__(self):
        logging.info("🎤 [Speech] Initializing SpeechHandler (Primary: Groq, Fallback: Local)")
        
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
            logging.info(f"🔄 [Speech] Loading Local Whisper '{model_size}' (Fallback)...")
            local_whisper_model = whisper.load_model(model_size, device=settings.DEVICE)
        
        logging.info("🐢 [Speech] Transcribing with Local Whisper...")
        result = local_whisper_model.transcribe(file_path, language="th")
        return result.get('text', '').strip()

    async def transcribe_audio_bytes(self, audio_bytes: bytes) -> str:
        if not audio_bytes: return ""
        
        with tempfile.NamedTemporaryFile(delete=False, suffix=".webm") as temp_file:
            temp_file.write(audio_bytes)
            temp_file_path = temp_file.name

        try:
            logging.info("🚀 [Speech] Trying Groq Whisper...")
            text = await asyncio.to_thread(self._transcribe_with_groq, temp_file_path)
            logging.info(f"✅ [Speech] Groq Result: '{text}'")
            return text

        except Exception as e:
            logging.warning(f"⚠️ [Speech] Groq failed ({e}). Switching to Local Whisper...")
            try:
                text = await asyncio.to_thread(self._transcribe_with_local, temp_file_path)
                logging.info(f"✅ [Speech] Local Result: '{text}'")
                return text
            except Exception as local_e:
                logging.error(f"❌ [Speech] All STT methods failed: {local_e}")
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
        print(f"🗣️  [TTS] Synthesizing speech for: '{clean_text[:30]}...'")
        
        # ลองใช้ edge_tts ก่อน
        try:
            logging.info("🚀 [TTS] Trying edge-tts...")
            communicate = edge_tts.Communicate(clean_text, settings.TTS_VOICE, rate="-10%")
            
            mp3_buffer = io.BytesIO()
            async for chunk in communicate.stream():
                if chunk["type"] == "audio":
                    mp3_buffer.write(chunk["data"])
            
            mp3_buffer.seek(0)
            audio_data = mp3_buffer.read()
            
            # ตรวจสอบว่ามีข้อมูลเสียงจริง
            if len(audio_data) > 0:
                logging.info(f"✅ [TTS] edge-tts success ({len(audio_data)} bytes)")
                return audio_data
            else:
                raise Exception("edge-tts returned empty audio")

        except Exception as e:
            logging.warning(f"⚠️ [TTS] edge-tts failed ({e}). Switching to gTTS fallback...")
            
            # Fallback: ใช้ gTTS + speed up 1.25x
            try:
                tts = gTTS(text=clean_text, lang='th', slow=False)
                mp3_buffer = io.BytesIO()
                tts.write_to_fp(mp3_buffer)
                mp3_buffer.seek(0)
                
                # Speed up เสียง 1.25x ด้วย pydub
                audio = AudioSegment.from_mp3(mp3_buffer)
                faster_audio = audio.speedup(playback_speed=1.25)
                
                output_buffer = io.BytesIO()
                faster_audio.export(output_buffer, format='mp3')
                output_buffer.seek(0)
                audio_data = output_buffer.read()
                
                logging.info(f"✅ [TTS] gTTS fallback success (1.25x speed, {len(audio_data)} bytes)")
                return audio_data
                
            except Exception as gtts_error:
                logging.error(f"❌ [TTS] All TTS methods failed: {gtts_error}", exc_info=True)
                raise RuntimeError("Failed to synthesize speech with both edge-tts and gTTS.")

speech_handler_instance = SpeechHandler()