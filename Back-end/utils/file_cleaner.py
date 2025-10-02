# /utils/file_cleaner.py

import os
import time
import schedule
import threading
from pathlib import Path

TEMP_AUDIO_DIR = Path(__file__).parent.parent / "temp_audio"

def setup_temp_audio_dir():
    """สร้างโฟลเดอร์ถ้ายังไม่มี"""
    TEMP_AUDIO_DIR.mkdir(exist_ok=True)
    print(f"✅ Temporary audio directory is ready at: {TEMP_AUDIO_DIR}")

def delete_old_files(directory: Path, max_age_minutes: int = 5):
    """ลบไฟล์ที่เก่ากว่าเวลาที่กำหนด"""
    print(f"🧹 Running cleanup job for '{directory}'...")
    cutoff_time = time.time() - (max_age_minutes * 60)
    files_deleted = 0
    for filepath in directory.iterdir():
        if filepath.is_file():
            try:
                file_mtime = filepath.stat().st_mtime
                if file_mtime < cutoff_time:
                    filepath.unlink()
                    files_deleted += 1
            except Exception as e:
                print(f"❌ Error deleting file {filepath}: {e}")
    if files_deleted > 0:
        print(f"🗑️  Deleted {files_deleted} old audio file(s).")

def run_cleanup_scheduler():
    """ตั้งเวลาให้ 'ภารโรง' ทำงานทุกๆ 5 นาที"""
    schedule.every(5).minutes.do(delete_old_files, directory=TEMP_AUDIO_DIR, max_age_minutes=5)
    
    print("🕰️  File cleanup scheduler started. Will run every 5 minutes.")
    while True:
        schedule.run_pending()
        time.sleep(1)

def start_background_cleanup():
    """เริ่มการทำงานของ 'ภารโรง' ใน Background Thread"""
    setup_temp_audio_dir()
    cleanup_thread = threading.Thread(target=run_cleanup_scheduler, daemon=True)
    cleanup_thread.start()