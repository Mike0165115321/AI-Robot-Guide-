import json
import os 

path_to_data = "/home/ratthanan/AI-Robot-Guide-/Back-end/core/database/data/_processed/superdata_filtered_attractions.jsonl"

# เปลี่ยนไปใช้ List of Dictionaries เพื่อเก็บข้อมูลที่เกี่ยวข้องกันไว้ด้วยกัน
data_with_coords = []

# ขั้นตอนที่ 1 และ 2: เปิดและอ่านทีละบรรทัด
with open(path_to_data, 'r', encoding='utf-8') as f:
    # ใช้ enumerate เพื่อบันทึกเลขบรรทัดหากเกิดข้อผิดพลาด (ตามคำแนะนำก่อนหน้า)
    for line_number, line in enumerate(f, start=1):
        try:
            # ขั้นตอนที่ 3: แปลง JSON String เป็น Python Dictionary
            data_object = json.loads(line)
            
            # ดึงค่าที่ต้องการ
            file_title = data_object.get("title")
            file_slug = data_object.get("slug")

            
            # ตรวจสอบว่ามีค่า 'title' และพิกัดหรือไม่
            if file_slug is not None:
                # จัดเก็บข้อมูลในรูปแบบ Dictionary
                data_with_coords.append({
                    "slug": file_slug,
                    "title": file_title
                })
                
        except json.JSONDecodeError:
            print(f"🚨 พบข้อผิดพลาดในการถอดรหัส JSON ที่บรรทัด **{line_number}**: {line.strip()}")
            
        # เพิ่มการจัดการ AttributeError เผื่อไว้ หากมีการอ่านข้อมูลที่ไม่ถูกต้อง
        except AttributeError:
             print(f"⚠️ พบ AttributeError ที่บรรทัด **{line_number}**: ข้อมูลไม่ใช่ Dictionary หรือมีปัญหา")


## แสดงผลลัพธ์

print("\n รายชื่อหัวข้อพร้อมพิกัดใน nan_data_with_coords.jsonl")
# ใช้ enumerate เพื่อเพิ่มเลขกำกับ
for item_number, item in enumerate(data_with_coords, start=1):
    print(f"{item_number}. {item['slug']}, {item['title']}")