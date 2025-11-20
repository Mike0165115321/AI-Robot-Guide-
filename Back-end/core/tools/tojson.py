import json
from pathlib import Path

# --- กำหนดชื่อไฟล์ ---
# ไฟล์ต้นฉบับ (JSON Lines)
input_file = Path("/home/ratthanan/AI-Robot-Guide-/Back-end/core/database/data/_processed/superdata_filtered_attractions.jsonl")
# ไฟล์เป้าหมาย (JSON Array)
output_file = Path("/home/ratthanan/AI-Robot-Guide-/Back-end/core/database/data/_processed/superdata_filtered_attractions.json")

def convert_jsonl_to_json(input_path: Path, output_path: Path):
    """
    อ่านไฟล์ JSONL และรวมข้อมูลทั้งหมดเข้าเป็น JSON Array แล้วบันทึกในไฟล์ JSON ใหม่
    """
    
    # 1. ตรวจสอบว่าไฟล์ต้นฉบับมีอยู่จริงหรือไม่
    if not input_path.exists():
        print(f"❌ ข้อผิดพลาด: ไม่พบไฟล์ต้นฉบับ: {input_path}")
        print("โปรดตรวจสอบว่าไฟล์ nan_data_with_coords.jsonl อยู่ในโฟลเดอร์เดียวกันหรือไม่")
        return

    data_list = []
    processed_lines = 0
    
    # 2. อ่านไฟล์ JSONL ทีละบรรทัด
    try:
        with input_path.open('r', encoding='utf-8') as infile:
            print(f"กำลังอ่านไฟล์ JSONL: {input_path}...")
            
            for line in infile:
                line = line.strip()
                if line:
                    try:
                        # แปลง JSON string เป็น Python Dictionary
                        data = json.loads(line)
                        data_list.append(data)
                        processed_lines += 1
                        
                    except json.JSONDecodeError as e:
                        print(f"⚠️ คำเตือน: ข้ามบรรทัดเนื่องจากรูปแบบ JSON ผิดพลาด: {e} ที่บรรทัด {processed_lines + 1}")

    except Exception as e:
        print(f"❌ เกิดข้อผิดพลาดขณะอ่านไฟล์: {e}")
        return

    # 3. บันทึกข้อมูลทั้งหมดเป็นไฟล์ JSON ใหม่
    if data_list:
        try:
            with output_path.open('w', encoding='utf-8') as outfile:
                # ใช้ json.dump เพื่อเขียน Python List (data_list) ลงในไฟล์
                # indent=4 เพื่อให้อ่านง่าย
                # ensure_ascii=False เพื่อให้ภาษาไทยไม่ถูกแปลงเป็นรหัส \uXXXX
                json.dump(data_list, outfile, ensure_ascii=False, indent=4)
            
            print(f"\n✅ การแปลงเสร็จสมบูรณ์!")
            print(f"จำนวนรายการที่ถูกประมวลผล: {processed_lines}")
            print(f"ไฟล์ JSON เป้าหมายถูกบันทึกที่: {output_path.resolve()}")
            
        except Exception as e:
            print(f"❌ เกิดข้อผิดพลาดขณะเขียนไฟล์ JSON: {e}")
    else:
        print("⚠️ ไฟล์ต้นฉบับไม่มีข้อมูลที่ถูกต้องให้ประมวลผล")

# --- รันฟังก์ชัน ---
convert_jsonl_to_json(input_file, output_file)