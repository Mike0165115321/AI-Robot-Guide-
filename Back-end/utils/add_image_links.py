# /utils/add_image_links.py (Upgraded Version)
import json
import os

# 1. กำหนด "Keyword" ของสถานที่ และ "Image Prefix" ที่ตรงกัน
#    Image Prefix คือส่วนหน้าของชื่อไฟล์รูปภาพ (ไม่มี -01, -02)
IMAGE_MAPPING = {
    "วัดภูมินทร์": "wat-phumin",
    "ดอยเสมอดาว": "doi-samoe-dao",
    "อุทยานแห่งชาติดอยภูคา": "doi-phu-kha-park",
    "ดอยภูคา": "doi-phu-kha-park",
    "เสาดินนาน้อย": "sao-din-na-noi",
    "วัดพระธาตุแช่แห้ง": "wat-phra-that-chae-haeng",
    "พิพิธภัณฑสถานแห่งชาติน่าน": "nan-national-museum",
    "ถนนคนเดินกาดข่วงเมืองน่าน": "nan-walking-street",
    # ... เพิ่มสถานที่อื่นๆ ...
}

# 2. กำหนด Path (เหมือนเดิม)
INPUT_JSONL_PATH = os.path.join('core', 'database', 'data', 'nan_guide_data.jsonl') # <-- แก้ชื่อไฟล์ให้ตรงกับของคุณ
OUTPUT_JSONL_PATH = os.path.join('core', 'database', 'data', 'nan_guide_data_with_images.jsonl')

def process_jsonl_file():
    print(f"กำลังอ่านไฟล์จาก: {INPUT_JSONL_PATH}")
    if not os.path.exists(INPUT_JSONL_PATH):
        print(f"Error: ไม่พบไฟล์ {INPUT_JSONL_PATH}")
        return

    lines_processed = 0
    lines_updated = 0
    
    with open(OUTPUT_JSONL_PATH, 'w', encoding='utf-8') as outfile:
        with open(INPUT_JSONL_PATH, 'r', encoding='utf-8') as infile:
            for line in infile:
                lines_processed += 1
                try:
                    data = json.loads(line)
                    content = data.get("content", "")
                    
                    found_prefix = None
                    for place_name, image_prefix in IMAGE_MAPPING.items():
                        if place_name in content:
                            found_prefix = image_prefix
                            break
                    
                    if found_prefix:
                        if "metadata" not in data:
                            data["metadata"] = {}
                        
                        data["metadata"]["image_prefix"] = found_prefix
                        lines_updated += 1
                    
                    outfile.write(json.dumps(data, ensure_ascii=False) + '\n')

                except json.JSONDecodeError:
                    print(f"Warning: ข้ามบรรทัดที่ {lines_processed} เนื่องจากไม่ใช่ JSON format ที่ถูกต้อง")

    print("\n--- สรุปผล ---")
    print(f"ประมวลผลทั้งหมด: {lines_processed} บรรทัด")
    print(f"อัปเดตข้อมูล Image Prefix: {lines_updated} บรรทัด")
    print(f"ไฟล์ผลลัพธ์ถูกบันทึกที่: {OUTPUT_JSONL_PATH}")
    print("ขั้นตอนต่อไป: โปรดใช้ไฟล์ใหม่นี้ในการ re-index ข้อมูลของคุณ")

if __name__ == "__main__":
    process_jsonl_file()