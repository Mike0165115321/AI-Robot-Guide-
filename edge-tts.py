import asyncio
import edge_tts
import platform

TEXT = "สวัสดีครับ นี่คือการทดสอบเสียงภาษาไทยจาก Microsoft Edge TTS"
VOICE = "th-TH-PremwadeeNeural" 
OUTPUT_FILE = "thai_output.mp3"

async def generate_thai_speech() -> None:
    """Main function to generate speech."""
    try:
        communicate = edge_tts.Communicate(TEXT, VOICE)
        await communicate.save(OUTPUT_FILE)
        print(f"สร้างไฟล์เสียง '{OUTPUT_FILE}' เสร็จสิ้นแล้ว!")
    except Exception as e:
        print(f"เกิดข้อผิดพลาด: {e}")
        print("ตรวจสอบการเชื่อมต่ออินเทอร์เน็ต และชื่อ VOICE ที่ใช้อีกครั้ง")

# ใช้ asyncio.run() ซึ่งเป็นวิธีมาตรฐานและจัดการ Event Loop อัตโนมัติใน Python 3.7+
if __name__ == "__main__":
    asyncio.run(generate_thai_speech())
