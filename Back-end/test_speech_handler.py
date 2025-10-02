# /Back-end/test_speech_handler.py

import asyncio
from core.ai_models.speech_handler import SpeechHandler
import numpy as np # Library สำหรับจัดการข้อมูลตัวเลข, ใช้สร้างเสียงจำลอง
import wave # Library สำหรับทำงานกับไฟล์ WAV
import simpleaudio as sa # Library สำหรับเล่นเสียง (ติดตั้งด้วย pip install simpleaudio)

# --- ส่วนของการทดสอบ ---
async def main_test():
    print("--- Starting SpeechHandler Test ---")

    # 1. สร้าง Instance ของ SpeechHandler
    # (ขั้นตอนนี้อาจใช้เวลาสักครู่ในการโหลดโมเดล Whisper)
    handler = SpeechHandler(model_size="base")

    # --- Test Case 1: Text-to-Speech (TTS) ---
    print("\n--- Testing TTS: Text to Raw PCM Bytes ---")
    test_text = "สวัสดีครับ นี่คือการทดสอบระบบเสียงพูด"
    try:
        # เรียกใช้ฟังก์ชันที่เราต้องการทดสอบ
        pcm_audio_bytes = handler.synthesize_speech_to_bytes(test_text)
        
        if pcm_audio_bytes:
            print(f"✅ TTS Success! Generated {len(pcm_audio_bytes)} bytes of PCM data.")
            
            # --- เล่นเสียงที่ได้เพื่อฟังผลลัพธ์ ---
            print("🔊 Playing synthesized audio...")
            # simpleaudio ต้องการ sample_width, channels, frame_rate
            # 2 bytes = 16-bit, 1 channel = mono, 24000 = 24kHz
            wave_obj = sa.WaveObject(pcm_audio_bytes, 1, 2, 24000)
            play_obj = wave_obj.play()
            play_obj.wait_done() # รอให้เล่นจนจบ
            print("▶️ Playback finished.")
            
        else:
            print("❌ TTS Failed! No audio data returned.")

    except Exception as e:
        print(f"❌ An error occurred during TTS test: {e}")


    # --- Test Case 2: Speech-to-Text (STT) ---
    print("\n--- Testing STT: Raw PCM Bytes to Text ---")
    try:
        # สร้างเสียงจำลอง: เสียง sine wave ที่ความถี่ 440Hz (ตัวโน้ต A)
        # นี่ไม่ใช่เสียงพูดจริง แต่เป็นการทดสอบว่า Whisper สามารถประมวลผล
        # Raw PCM data ที่เราสร้างขึ้นได้หรือไม่
        sample_rate = 16000 # 16kHz
        duration = 2 # 2 วินาที
        frequency = 440
        t = np.linspace(0., duration, int(sample_rate * duration), endpoint=False)
        amplitude = np.iinfo(np.int16).max * 0.5
        data = amplitude * np.sin(2. * np.pi * frequency * t)
        
        # แปลง Float64 (จาก numpy) เป็น Int16 (2 bytes) ที่ VAD/Whisper ต้องการ
        simulated_pcm_bytes = data.astype(np.int16).tobytes()

        print(f"🔬 Created simulated PCM audio ({len(simulated_pcm_bytes)} bytes).")

        # เรียกใช้ฟังก์ชันที่เราต้องการทดสอบ
        transcribed_text = handler.transcribe_audio_bytes(simulated_pcm_bytes)
        
        # หมายเหตุ: Whisper อาจจะถอดเสียง sine wave ไม่ได้ หรือได้เป็นข้อความแปลกๆ
        # ซึ่งเป็นเรื่องปกติ! เป้าหมายของเราคือ "ดูว่ามันทำงานโดยไม่พัง"
        print(f"✅ STT Processed without error! Transcription result: '{transcribed_text}'")
        print("(Note: Empty or strange text is expected for a sine wave test)")

    except Exception as e:
        print(f"❌ An error occurred during STT test: {e}")

    print("\n--- Test Finished ---")

# --- รันการทดสอบ ---
if __name__ == "__main__":
    # ติดตั้ง library ที่จำเป็นก่อนรัน
    print("Please make sure you have installed 'numpy' and 'simpleaudio'.")
    print("Run: pip install numpy simpleaudio")
    
    # รัน main_test function
    asyncio.run(main_test())