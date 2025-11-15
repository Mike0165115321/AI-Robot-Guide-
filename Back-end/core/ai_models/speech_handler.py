# /core/ai_models/speech_handler.py (ฉบับย้อนกลับ: ใช้ openai-whisper + GPU)

import whisper
import io
import re
import asyncio
import logging
from pydub import AudioSegment
import edge_tts
import tempfile
import numpy as np
from core.config import settings 

def sanitize_text_for_speech(text: str) -> str:
    text = re.sub(r'^\s*#+\s*', '', text, flags=re.MULTILINE)
    text = re.sub(r'\*\*(.*?)\*\*', r'\1', text)
    text = re.sub(r'\*(.*?)\*', r'\1', text)
    text = text.replace('💡', '')
    text = text.replace('...', '. ')
    text = re.sub(r'\s+', ' ', text).strip()
    return text

class SpeechHandler:
    def __init__(self, model_size="base"):
        device_to_use = settings.DEVICE
        print(f"🔄 [Speech] Loading OpenAI-Whisper model '{model_size}' onto device '{device_to_use}'...")
        
        self.stt_model = whisper.load_model(model_size, device=device_to_use)
        
        print(f"✅ [Speech] OpenAI-Whisper model '{model_size}' loaded successfully on '{device_to_use}'.")

    def _transcribe_sync(self, audio_bytes: bytes) -> str:
        if not self.stt_model:
            raise RuntimeError("Whisper model is not loaded.")
        
        try:
            webm_file_like_object = io.BytesIO(audio_bytes)
            audio_segment = AudioSegment.from_file(webm_file_like_object, format="webm")
            wav_segment = audio_segment.set_channels(1).set_frame_rate(16000)

            with tempfile.NamedTemporaryFile(delete=True, suffix=".wav") as temp_wav_file:
                wav_segment.export(temp_wav_file.name, format="wav")
                
                print(f"🎤 [STT] Transcribing audio with OpenAI-Whisper...")

                result = self.stt_model.transcribe(temp_wav_file.name, language="th", fp16=False)
            
            transcribed_text = result.get('text', '').strip()
            print(f"📝 [STT] OpenAI-Whisper result: '{transcribed_text}'")
            return transcribed_text

        except Exception as e:
            logging.error(f"❌ [STT] Error transcribing audio data: {e}", exc_info=True)
            return ""

    async def transcribe_audio_bytes(self, audio_bytes: bytes) -> str:
        return await asyncio.to_thread(self._transcribe_sync, audio_bytes)


    async def synthesize_speech_to_bytes(self, text: str) -> bytes:
        # (ฟังก์ชัน TTS - เหมือนเดิม ไม่ต้องแก้)
        if not text.strip():
            raise ValueError("Input text for TTS cannot be empty.")

        clean_text = sanitize_text_for_speech(text)
        print(f"🗣️  [TTS] Synthesizing speech for: '{clean_text[:30]}...'")
        
        try:
            communicate = edge_tts.Communicate(clean_text, "th-TH-PremwadeeNeural", rate="-10%")
            
            mp3_buffer = io.BytesIO()
            async for chunk in communicate.stream():
                if chunk["type"] == "audio":
                    mp3_buffer.write(chunk["data"])
            
            mp3_buffer.seek(0)
            return mp3_buffer.read()

        except Exception as e:
            logging.error(f"❌ [TTS] Error during edge-tts synthesis: {e}", exc_info=True)
            raise RuntimeError("Failed to synthesize speech.")

speech_handler_instance = SpeechHandler(model_size="medium")