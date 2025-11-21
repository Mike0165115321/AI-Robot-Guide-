import json

def convert_json_to_jsonl(input_file_path, output_file_path):
    """
    อ่านไฟล์ JSON ที่มีข้อมูลเป็น Array of Objects และแปลงเป็นไฟล์ JSON Lines (JSONL)

    Args:
        input_file_path (str): ชื่อหรือพาธของไฟล์ JSON ต้นฉบับ
        output_file_path (str): ชื่อหรือพาธของไฟล์ JSONL ที่ต้องการสร้าง
    """
    try:
        # 1. อ่านข้อมูลจากไฟล์ JSON ต้นฉบับ (Array of Objects)
        with open(input_file_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        if not isinstance(data, list):
            print(f"❌ ข้อผิดพลาด: ข้อมูลในไฟล์ '{input_file_path}' ไม่อยู่ในรูปแบบ Array of Objects")
            return

        # 2. เขียนข้อมูลลงในไฟล์ JSONL (Object ต่อหนึ่งบรรทัด)
        with open(output_file_path, 'w', encoding='utf-8') as outfile:
            for record in data:
                # ใช้ json.dumps เพื่อแปลง Object เป็นสตริง JSON
                # และเขียนลงในไฟล์ ตามด้วย '\n' เพื่อให้แต่ละ Object อยู่คนละบรรทัด
                json_line = json.dumps(record, ensure_ascii=False)
                outfile.write(json_line + '\n')

        print(f"✅ แปลงข้อมูลสำเร็จ: ได้สร้างไฟล์ JSON Lines ที่ '{output_file_path}' เรียบร้อยแล้ว")
        print(f"   จำนวนรายการที่ถูกแปลง: {len(data)} รายการ")

    except FileNotFoundError:
        print(f"❌ ข้อผิดพลาด: ไม่พบไฟล์ต้นฉบับที่ '{input_file_path}'")
    except json.JSONDecodeError:
        print(f"❌ ข้อผิดพลาด: รูปแบบไฟล์ JSON ที่ '{input_file_path}' ไม่ถูกต้อง")
    except Exception as e:
        print(f"❌ ข้อผิดพลาดที่ไม่คาดคิด: {e}")

# --- ตัวอย่างการใช้งาน ---

# กำหนดชื่อไฟล์ต้นฉบับและไฟล์ปลายทาง
INPUT_FILE = '/home/ratthanan/AI-Robot-Guide-/Back-end/core/database/data/_processed/superdata_filtered_attractions.json'
OUTPUT_FILE = '/home/ratthanan/AI-Robot-Guide-/Back-end/core/database/data/_processed/superdata_filtered_attractions.jsonl'

# รันฟังก์ชัน
convert_json_to_jsonl(INPUT_FILE, OUTPUT_FILE)