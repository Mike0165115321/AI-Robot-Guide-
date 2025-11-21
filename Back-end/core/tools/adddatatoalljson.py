import json

# กำหนดชื่อไฟล์อินพุตและเอาท์พุต
INPUT_FILE = "superdata.json"
OUTPUT_FILE = "superdata_with_statusmaps.json"

# กำหนดฟิลด์และค่าที่ต้องการเพิ่ม
NEW_FIELD = "statusmaps"
NEW_VALUE = "นำทางได้"

print(f"กำลังดำเนินการอ่านไฟล์: {INPUT_FILE} ...")

updated_data = []
record_count = 0

try:
    with open(INPUT_FILE, 'r', encoding='utf-8') as f:
        # สมมติว่าไฟล์เป็นรูปแบบ JSON Array ([{...}, {...}])
        # หากไฟล์เป็น JSONL (แต่ละบรรทัดคือ 1 อ็อบเจกต์) ต้องปรับการอ่านเล็กน้อย
        
        # สำหรับไฟล์ที่เป็น JSON Array ธรรมดา (มาตรฐาน):
        all_data = json.load(f)
        
        for item in all_data:
            # เพิ่มฟิลด์ใหม่เข้าไปในอ็อบเจกต์
            item[NEW_FIELD] = NEW_VALUE
            updated_data.append(item)
            record_count += 1
            
    print(f"อ่านข้อมูลและเพิ่มฟิลด์เรียบร้อยแล้ว {record_count} รายการ")

    # บันทึกข้อมูลที่แก้ไขแล้วลงในไฟล์ใหม่ (แนะนำให้บันทึกไฟล์ใหม่เพื่อความปลอดภัย)
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
        # ใช้ ensure_ascii=False เพื่อให้แสดงภาษาไทยได้อย่างถูกต้อง
        json.dump(updated_data, f, indent=4, ensure_ascii=False) 
        
    print(f"✅ บันทึกข้อมูลที่แก้ไขแล้วลงในไฟล์: {OUTPUT_FILE} เรียบร้อยแล้ว")
    
except FileNotFoundError:
    print(f"❌ เกิดข้อผิดพลาด: ไม่พบไฟล์ชื่อ '{INPUT_FILE}'")
except json.JSONDecodeError:
    print("❌ เกิดข้อผิดพลาด: โครงสร้างไฟล์ JSON ไม่ถูกต้อง กรุณาตรวจสอบว่าไฟล์อยู่ในรูปแบบ JSON Array [{}, {}] หรือไม่")
except Exception as e:
    print(f"❌ เกิดข้อผิดพลาดที่ไม่คาดคิด: {e}")