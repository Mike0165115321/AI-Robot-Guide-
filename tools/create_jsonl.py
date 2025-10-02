# /tools/create_jsonl.py
import json
import os

topic_name = "ข้อมูลสถานที่ที่เกี่ยวข้องกับประวัติศาสตร์"

entries = [
    
]

output_filename = f"{topic_name}.jsonl"
    
    # กำหนด path ปลายทางให้ไปอยู่ที่ Back-end/core/database/data/
output_path = os.path.join(os.path.dirname(__file__), '..', 'Back-end', 'core', 'database', 'data', output_filename)

os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
with open(output_path, "w", encoding="utf-8") as f:
    for entry in entries:
        json_line = json.dumps(entry, ensure_ascii=False)
        f.write(json_line + "\n")

print(f"✅ สร้างไฟล์ .jsonl สำเร็จ: {output_path}")