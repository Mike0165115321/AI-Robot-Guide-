# /core/ai_models/speech_handler.py (Final WebM Handling Version)

import whisper
import io
from gtts import gTTS
import tempfile
from pydub import AudioSegment # <-- ต้องใช้ pydub

class SpeechHandler:
    def __init__(self, model_size="base"):
        print(f"🔄 [Speech] Loading Whisper model '{model_size}'...")
        self.stt_model = whisper.load_model(model_size)
        print(f"✅ [Speech] Whisper model '{model_size}' loaded successfully.")

    def transcribe_audio_bytes(self, audio_bytes: bytes) -> str:
        """
        [สำคัญ] แปลงข้อมูลเสียงแบบ bytes (ซึ่งเป็นไฟล์ .webm) เป็นข้อความ
        """
        if not self.stt_model:
            raise RuntimeError("Whisper model is not loaded.")

        try:
            # 1. สร้าง file-like object ใน memory จาก bytes ที่ได้รับมา
            webm_file_like_object = io.BytesIO(audio_bytes)
            
            # 2. ให้ pydub โหลดเสียงจาก object นี้ และบอกมันว่าเป็น format 'webm'
            audio_segment = AudioSegment.from_file(webm_file_like_object, format="webm")
            
            # 3. แปลงเสียงเป็น Mono, 16kHz WAV ซึ่งเป็น format ที่ดีที่สุดสำหรับ Whisper
            wav_segment = audio_segment.set_channels(1).set_frame_rate(16000)

            # 4. สร้างไฟล์ WAV ชั่วคราวเพื่อให้ Whisper อ่าน
            with tempfile.NamedTemporaryFile(delete=True, suffix=".wav") as temp_wav_file:
                wav_segment.export(temp_wav_file.name, format="wav")
                
                print(f"🎤 [STT] Transcribing converted .wav audio...")
                result = self.stt_model.transcribe(
                    temp_wav_file.name,
                    language="th",
                    fp16=False
                )
                
                transcribed_text = result['text'].strip()
                print(f"📝 [STT] Transcription result: '{transcribed_text}'")
                return transcribed_text

        except Exception as e:
            print(f"❌ [STT] Error transcribing .webm data: {e}", exc_info=True)
            return ""

    def synthesize_speech_to_bytes(self, text: str) -> bytes:
        """
        [สำคัญ] แปลงข้อความเป็น MP3 bytes เพื่อส่งกลับไปให้เบราว์เซอร์
        """
        if not text.strip():
            raise ValueError("Input text for TTS cannot be empty.")
        
        print(f"🗣️  [TTS] Synthesizing speech to MP3 bytes for text: '{text[:30]}...'")
        
        try:
            # ใช้ gTTS สร้างเสียง MP3 ใน memory
            tts = gTTS(text=text, lang='th', slow=False)
            mp3_fp = io.BytesIO()
            tts.write_to_fp(mp3_fp)
            mp3_fp.seek(0)
            
            # คืนค่าเป็น MP3 bytes โดยตรง
            return mp3_fp.read()

        except Exception as e:
            print(f"❌ [TTS] Error during MP3 speech synthesis: {e}")
            raise RuntimeError("Failed to synthesize speech to MP3 bytes.")

# --- สร้าง instance กลางไว้ใช้งาน ---
speech_handler_instance = SpeechHandler(model_size="base")