import asyncio
import logging
from pathlib import Path
import time

# [V5.1] ย้าย Path มาไว้ด้านบน
TEMP_AUDIO_DIR = Path(__file__).parent.parent / "temp_audio"
CLEANUP_INTERVAL_SECONDS = 300 # 5 นาที (300 วินาที)
MAX_AGE_MINUTES = 5            # ลบไฟล์ที่เก่ากว่า 5 นาที

def setup_temp_audio_dir():
    """สร้างโฟลเดอร์ถ้ายังไม่มี (ฟังก์ชันนี้ OK)"""
    try:
        TEMP_AUDIO_DIR.mkdir(exist_ok=True)
        logging.info(f"✅ Temporary audio directory is ready at: {TEMP_AUDIO_DIR}")
    except Exception as e:
        logging.error(f"❌ Failed to create temp audio dir: {e}")

# [V5.1] เราจะแยก Logic การลบไฟล์ (ที่เป็น Sync I/O) ออกมา
def _delete_old_files_sync() -> int:
    """
    (Sync Function) Logic การลบไฟล์
    ฟังก์ชันนี้จะถูกย้ายไปรันใน to_thread เพื่อไม่ให้บล็อก
    """
    logging.info(f"🧹 [CleanupTask] Running cleanup job for '{TEMP_AUDIO_DIR}'...")
    try:
        if not TEMP_AUDIO_DIR.is_dir():
            logging.warning(f"Cleanup skipped: Directory not found: {TEMP_AUDIO_DIR}")
            return 0
        
        cutoff_time = time.time() - (MAX_AGE_MINUTES * 60)
        files_deleted = 0
        
        # (Sync I/O: .iterdir(), .stat(), .unlink())
        for filepath in TEMP_AUDIO_DIR.iterdir():
            if filepath.is_file():
                try:
                    file_mtime = filepath.stat().st_mtime
                    if file_mtime < cutoff_time:
                        filepath.unlink()
                        files_deleted += 1
                except Exception as e:
                    logging.warning(f"❌ Error deleting file {filepath}: {e}")
                    
        if files_deleted > 0:
            logging.info(f"🗑️  [CleanupTask] Deleted {files_deleted} old audio file(s).")
        else:
            logging.info(f"🧹 [CleanupTask] No old files to delete.")
            
        return files_deleted
        
    except Exception as e:
        logging.error(f"❌ [CleanupTask] Error during file iteration: {e}")
        return 0

# [V5.1] นี่คือฟังก์ชัน "ตัวจริง" ที่ main.py จะเรียก
async def start_background_cleanup():
    """
    (Async Coroutine) งานเบื้องหลังที่จะรันตลอดเวลา
    """
    setup_temp_audio_dir()
    logging.info(f"🕰️  [CleanupTask] Async file cleanup scheduler started. Will run every {CLEANUP_INTERVAL_SECONDS} seconds.")
    
    while True:
        try:
            # 1. "พัก" งานแบบ Async (ไม่บล็อก Event Loop)
            await asyncio.sleep(CLEANUP_INTERVAL_SECONDS) 
            
            # 2. เรียก Logic การลบไฟล์ (ที่เป็น Sync I/O)
            #    โดยย้ายไปรันใน Thread แยกอัตโนมัติ
            await asyncio.to_thread(_delete_old_files_sync)

        except asyncio.CancelledError:
            # (จำเป็น!) เมื่อ Server สั่งปิด (ใน lifespan)
            logging.info("🛑 [CleanupTask] File cleanup task cancelled.")
            break
        except Exception as e:
            # (เผื่อกรณี Task พัง)
            logging.error(f"❌ [CleanupTask] Error in cleanup loop: {e}", exc_info=True)
            # รอสักพักก่อนเริ่มใหม่
            await asyncio.sleep(60)