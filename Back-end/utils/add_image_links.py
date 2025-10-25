import json
import os
import shutil
from pathlib import Path
import logging
import re 
import time 

IMAGE_MAPPING = {
    "วัดภูมินทร์": "wat_phumin_",
    "ดอยเสมอดาว": "doi_samoe_dao_",
    "อุทยานแห่งชาติดอยภูคา": "doi_phu_kha_park_",
    "ดอยภูคา": "doi_phu_kha_park_",
    "เสาดินนาน้อย": "sao_din_na_noi_",
    "วัดพระธาตุแช่แห้ง": "wat_phra_that_chae_haeng_",
    "หมู่บ้านสะปัน": "sapan_village_",
    "วัดพระธาตุเขาน้อย": "wat_phra_that_khao_noi_",
    "ถนนคนเดิน": "nan_walking_street_",
    "กาดข่วงเมือง": "nan_walking_street_",
    "อุทยานแห่งชาติขุนสถาน": "khun_sathan_national_park_",
    "พิพิธภัณฑสถานแห่งชาติน่าน": "nan_national_museum_",
    "หอคำ": "nan_national_museum_",
    "วัดพระธาตุช้างค้ำ": "wat_phra_that_chang_kham_",
    "บ่อเกลือ": "bo_kluea_salt_licks_",
    "ดอยสกาด": "doi_skat_",
    "อ้อมดาวริมน้ำ": "aom_dao_riverside_",
    "ร้านอ้อมดาว": "aom_dao_restaurant_",
    "ตูบนา": "toobna_homestay_",
    "ลำดวนผ้าทอ": "tailue_coffee_",
    "กาแฟไทลื้อ": "tailue_coffee_",
    "ป้านิ่ม": "pa_nim_dessert_",
    "เฮือนภูคา": "huen_phukha_restaurant_",
    "บ้านนาก๋างโต้ง": "baan_na_kang_tong_",
    "Nirvanan House": "nirvanan_house_",
    "Bitter Bar": "bitter_bar_nan_",
    "วัดอรัญญาวาส": "wat_aranyawat_",
    "วัดมิ่งเมือง": "wat_ming_mueang_",
    "ประวัติศาสตร์น่าน": "history_nan_",
    "ยุคใหม่": "history_modern_",
    "ชนเผ่า": "ethnic_group_",
    "วัฒนธรรม": "culture_nan_",
}

BACKEND_ROOT = Path(__file__).resolve().parent.parent
DATA_SOURCE_FOLDER = BACKEND_ROOT / "core" / "database" / "data"

def generate_safe_slug(text: str) -> str:
    """
    สร้าง Slug ที่ปลอดภัย (a-z, 0-9, _, -) จากข้อความใดๆ
    """
    if not text:
        timestamp_ms = int(time.time() * 1000)
        return f"item_{timestamp_ms}"

    slug = text.lower().strip()
    slug = re.sub(r'[\s\(\)\[\]{}]+', '_', slug)
    slug = re.sub(r'[^a-z0-9_-]', '', slug)
    slug = re.sub(r'[-_]+', '_', slug)
    slug = slug.strip('_-')
    slug = slug[:50]
    if not slug:
        timestamp_ms = int(time.time() * 1000)
        return f"item_{timestamp_ms}"
    return slug


def process_all_jsonl_files():
    print(f"--- 🖼️  Starting to add 'slug' and 'image_prefix' (V6 - Smart Priority) in '{DATA_SOURCE_FOLDER}' ---")

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
                        
                        # --- [ NEW V6 LOGIC START ] ---
                        title = data.get("title", "")
                        title_lower = title.lower()
                        
                        summary_text = (
                            data.get("summary", "") + " " +
                            str(data.get("details", []))
                        ).lower()

                        found_prefix = None
                        best_match_key = ""

                        # 1. Priority Search: ค้นหาใน TITLE ก่อน
                        #    (เราจะหา "key" ที่ยาวที่สุดที่ตรงกัน เพื่อแก้ปัญหา "วัดภูมินทร์" vs "วัดพระธาตุช้างค้ำ")
                        if title_lower:
                            for place_name in IMAGE_MAPPING.keys():
                                if place_name.lower() in title_lower:
                                    if len(place_name) > len(best_match_key):
                                        best_match_key = place_name
                        
                        if best_match_key:
                            found_prefix = IMAGE_MAPPING[best_match_key]
                        
                        # 2. Fallback Search: ถ้าไม่เจอใน title, ค่อยค้นหาใน summary + details
                        if not found_prefix and summary_text:
                            best_match_key_fallback = ""
                            for place_name in IMAGE_MAPPING.keys():
                                if place_name.lower() in summary_text:
                                    if len(place_name) > len(best_match_key_fallback):
                                        best_match_key_fallback = place_name
                            
                            if best_match_key_fallback:
                                found_prefix = IMAGE_MAPPING[best_match_key_fallback]
                                logging.warning(f"Line {line_num} ('{title}'): Prefix not in title. Found '{found_prefix}' in summary.")
                        # --- [ NEW V6 LOGIC END ] ---

                        # Ensure metadata exists
                        if "metadata" not in data or data["metadata"] is None:
                            data["metadata"] = {}

                        final_slug = ""
                        # Case 1: Prefix found in mapping
                        if found_prefix:
                            data["metadata"]["image_prefix"] = found_prefix
                            final_slug = found_prefix.rstrip('_')
                            lines_updated += 1
                        # Case 2: No prefix found, generate slug from title
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
                                logging.warning(f"Line {line_num} ('{title}') missing prefix mapping. Generated slug: '{final_slug}'")
                            else:
                                logging.error(f"Line {line_num} missing 'title', cannot generate slug. Skipping line.")
                                outfile.write(line)
                                continue 

                        # Assign the determined slug (either from prefix or generated)
                        if final_slug:
                            data["slug"] = final_slug
                            slugs_generated_in_file.add(final_slug)

                        outfile.write(json.dumps(data, ensure_ascii=False) + '\n')

                    except json.JSONDecodeError:
                        logging.warning(f"Skipping malformed JSON on line {line_num} in {filename}.")
                        outfile.write(line) # Write original line back
                    except Exception as line_e:
                        logging.error(f"Error processing line {line_num} in {filename}: {line_e}", exc_info=False)
                        outfile.write(line) # Write original line back


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