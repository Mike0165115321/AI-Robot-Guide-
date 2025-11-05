import json
import os
import shutil
from pathlib import Path
import logging
import re 
import time 

# IMAGE_MAPPING และค่าคงที่อื่นๆ ยังคงเหมือนเดิม
IMAGE_MAPPING = {
    "วัดภูมินทร์": "wat-phumin-",
    "ดอยเสมอดาว": "doi-samoe-dao-",
    "อุทยานแห่งชาติดอยภูคา": "doi-phu-kha-park-",
    "ดอยภูคา": "doi-phu-kha-park-",
    "เสาดินนาน้อย": "sao-din-na-noi-",
    "วัดพระธาตุแช่แห้ง": "wat-phra-that-chae-haeng-",
    "หมู่บ้านสะปัน": "sapan-village-",
    "วัดพระธาตุเขาน้อย": "wat-phra-that-khao-noi-",
    "ถนนคนเดิน": "nan-walking-street-",
    "กาดข่วงเมือง": "nan-walking-street-",
    "อุทยานแห่งชาติขุนสถาน": "khun-sathan-national-park-",
    "พิพิธภัณฑสถานแห่งชาติน่าน": "nan-national-museum-",
    "หอคำ": "nan-national-museum-",
    "วัดพระธาตุช้างค้ำ": "wat-phra-that-chang-kham-",
    "บ่อเกลือ": "bo-kluea-salt-licks-",
    "ดอยสกาด": "doi-skat-",
    "อ้อมดาวริมน้ำ": "aom-dao-riverside-",
    "ร้านอ้อมดาว": "aom-dao-restaurant-",
    "ตูบนา": "toobna-homestay-",
    "ลำดวนผ้าทอ": "tailue-coffee-",
    "กาแฟไทลื้อ": "tailue-coffee-",
    "ป้านิ่ม": "pa-nim-dessert-",
    "เฮือนภูคา": "huen-phukha-restaurant-",
    "บ้านนาก๋างโต้ง": "baan-na-kang-tong-",
    "Nirvanan House": "nirvanan-house-",
    "Bitter Bar": "bitter-bar-nan-",
    "วัดอรัญญาวาส": "wat-aranyawat-",
    "วัดมิ่งเมือง": "wat-ming-mueang-",
    "ประวัติศาสตร์น่าน": "history-nan-",
    "ยุคใหม่": "history-modern-",
    "ชนเผ่า": "ethnic-group-",
    "วัฒนธรรม": "culture-nan-",
}

BACKEND_ROOT = Path(__file__).resolve().parent.parent
DATA_SOURCE_FOLDER = BACKEND_ROOT / "core" / "database" / "data"

def generate_safe_slug(text: str) -> str:
    if not text:
        timestamp_ms = int(time.time() * 1000)
        return f"item-{timestamp_ms}" 

    slug = text.lower().strip()
    slug = re.sub(r'[\s\(\)\[\]{}]+', '-', slug) 
    slug = re.sub(r'[^a-z0-9ก-ฮ-]', '', slug)    
    slug = re.sub(r'[^a-z0-9-]', '', slug)    
    slug = re.sub(r'[-]+', '-', slug)      
    slug = slug.strip('-')                 
    slug = slug[:50]
    if not slug:
        timestamp_ms = int(time.time() * 1000)
        return f"item-{timestamp_ms}"
    return slug

def process_all_jsonl_files():
    print(f"--- 🖼️  Starting to add 'slug' and 'image_prefix' (V7 - Title Only) in '{DATA_SOURCE_FOLDER}' ---")

    if not DATA_SOURCE_FOLDER.is_dir():
        print(f"❌ ERROR: Directory not found: {DATA_SOURCE_FOLDER}")
        return

    files_to_process = [f for f in os.listdir(DATA_SOURCE_FOLDER) if f.endswith(".jsonl")]

    if not files_to_process:
        print(" - 🟡 No .jsonl files found to process. Please ensure you have moved your data files from the '_processed' folder back into the 'data' folder.")
        return

    total_files_processed = 0

    for filename in files_to_process:
        file_path = DATA_SOURCE_FOLDER / filename
        temp_file_path = file_path.with_suffix(file_path.suffix + ".tmp")

        print(f"\n--- 🏭 Processing file: '{filename}' ---")
        lines_updated = 0
        slugs_generated_in_file = set()

        try:
            with open(file_path, 'r', encoding='utf-8') as infile, open(temp_file_path, 'w', encoding='utf-8') as outfile:
                for line_num, line in enumerate(infile, 1):
                    try: 
                        data = json.loads(line)
                        
                        title = data.get("title", "")
                        title_lower = title.lower()
                        
                        found_prefix = None
                        best_match_key = ""

                        if title_lower:
                            for place_name in IMAGE_MAPPING.keys():
                                if place_name.lower() in title_lower:
                                    if len(place_name) > len(best_match_key):
                                        best_match_key = place_name
                        
                        if best_match_key:
                            found_prefix = IMAGE_MAPPING[best_match_key]
                        
                        if "metadata" not in data or data["metadata"] is None:
                            data["metadata"] = {}

                        final_slug = ""
                        if found_prefix:
                            data["metadata"]["image_prefix"] = found_prefix
                            final_slug = found_prefix.rstrip('-') 
                            lines_updated += 1
                        else:
                            if title:
                                generated_slug = generate_safe_slug(title)

                                original_slug = generated_slug
                                counter = 1
                                while generated_slug in slugs_generated_in_file:
                                    suffix = f"_{counter}"
                                    generated_slug = f"{original_slug[:50-len(suffix)]}{suffix}"
                                    counter += 1

                                final_slug = generated_slug
                            else:
                                logging.error(f"Line {line_num} missing 'title', cannot generate slug. Skipping line.")
                                outfile.write(line)
                                continue 

                        if final_slug:
                            data["slug"] = final_slug
                            slugs_generated_in_file.add(final_slug)

                        outfile.write(json.dumps(data, ensure_ascii=False) + '\n')

                    except json.JSONDecodeError:
                        logging.warning(f"Skipping malformed JSON on line {line_num} in {filename}.")
                        outfile.write(line) 
                    except Exception as line_e:
                        logging.error(f"Error processing line {line_num} in {filename}: {line_e}", exc_info=False)
                        outfile.write(line) 

            shutil.move(temp_file_path, file_path)
            print(f"  - ✅ Finished. Updated/Checked {lines_updated} (prefix) lines in '{filename}'.")
            total_files_processed += 1
        except Exception as e:
            print(f"❌ An error occurred while processing {filename}: {e}")
            logging.error(f"Failed processing {filename}", exc_info=True)
            if os.path.exists(temp_file_path):
                try:
                    os.remove(temp_file_path)
                except OSError as rm_err:
                    logging.error(f"Failed to remove temporary file {temp_file_path}: {rm_err}")

    print(f"\n--- 🎉 Successfully processed {total_files_processed} file(s). ---")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    process_all_jsonl_files()