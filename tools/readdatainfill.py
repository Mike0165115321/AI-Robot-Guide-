import json
import os
from pathlib import Path
import sys # นำเข้า sys เพื่อให้แสดงผล Error ได้ชัดเจนขึ้น

# ***************************************************************
# 1. การกำหนด PATH แบบยืดหยุ่น (Relative Path)
# ***************************************************************

# หาตำแหน่งของไฟล์ปัจจุบัน (readdatainfill.py)
current_dir = Path(__file__).resolve().parent

# เราอยู่ที่: /home/mikedev/AI Robot Guide จังหวัดน่าน/tools
# เราอยากไปที่: /home/mikedev/AI Robot Guide จังหวัดน่าน/Back-end/core/database/data/_processed/...

# current_dir.parent คือ /home/mikedev/AI Robot Guide จังหวัดน่าน/
# จึงต้องสร้าง Path จากจุดนี้
# **ปรับ Path ด้านล่างนี้ถ้าโครงสร้าง Project เปลี่ยนค่ะ**
# path_to_data = current_dir.parent /"Back-end/core/database/data/_processed/superdata_filtered_attractions.jsonl"

# print(f"✅ กำลังอ่านไฟล์จาก Path: {path_to_data}")

# Correct absolute paths
BASE_DIR = "/home/ratthanan/AI-Robot-Guide-"
path_to_data = os.path.join(BASE_DIR, "Back-end/core/database/data/_processed/superdata_filtered_attractions.jsonl")
path_to_images = os.path.join(BASE_DIR, "Back-end/static/images")

# Load all image filenames for efficient lookup
try:
    existing_images = set(os.listdir(path_to_images))
except FileNotFoundError:
    print(f"🚨 Image directory not found: {path_to_images}", file=sys.stderr)
    existing_images = set()

data_with_missing_images = []
data_with_images = []

print(f"Checking data from: {path_to_data}")
print(f"Checking images in: {path_to_images}")

# ***************************************************************
# 2. การอ่านและประมวลผลข้อมูลพร้อม Try/Except
# ***************************************************************
# data_with_coords = []
# processed_count = 0

try:
    with open(path_to_data, 'r', encoding='utf-8') as f:
        for line_number, line in enumerate(f, start=1):
            try:
                data_object = json.loads(line)
                
                file_title = data_object.get("title")
                file_slug = data_object.get("slug")
                
                # ตรวจสอบว่ามีค่า 'slug' หรือไม่ (เพราะต้องการใช้เป็น ID)
                if file_slug:
                    # Check if any image starts with the slug
                    # This assumes image names are like "slug-01.jpg", "slug.jpg", etc.
                    has_image = any(img.startswith(file_slug) for img in existing_images)
                    
                    if has_image:
                        data_with_images.append({"slug": file_slug, "title": file_title})
                    else:
                        data_with_missing_images.append({"slug": file_slug, "title": file_title})
                    
            except json.JSONDecodeError:
                print(f"🚨 JSON Decode Error at line {line_number}", file=sys.stderr)
            except AttributeError:
                 print(f"⚠️ AttributeError at line {line_number}", file=sys.stderr)
    
except FileNotFoundError:
    print(f"❌ ERROR: ไม่พบไฟล์ที่ {path_to_data}", file=sys.stderr)
    sys.exit(1) # จบโปรแกรมทันทีถ้าหาไฟล์ไม่พบ

print(f"\n✅ Found {len(data_with_images)} attractions with images.")
print(f"❌ Found {len(data_with_missing_images)} attractions WITHOUT images.")

if data_with_missing_images:
    print("\n--- Attractions Missing Images ---")
    for item in data_with_missing_images:
        print(f"Slug: {item['slug']} | Title: {item['title']}")
else:
    print("\n🎉 All attractions have at least one image!")