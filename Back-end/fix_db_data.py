
from pymongo import MongoClient
from core.config import settings

def fix_data():
    client = MongoClient(settings.MONGO_URI)
    db = client[settings.MONGO_DATABASE_NAME]
    col = db["nan_locations"]
    
    # Target the document with slug 'wat-phumin'
    target_slug = "wat-phumin"
    
    # Correct Data for Wat Phumin
    new_data = {
        "title": "วัดภูมินทร์ (Wat Phumin)",
        "category": "ข้อมูลสถานที่ท่องเที่ยว",
        "topic": "วัดและสถานที่สำคัญทางศาสนา",
        "summary": "วัดภูมินทร์ เป็นวัดหลวงเก่าแก่ที่มีชื่อเสียงที่สุดแห่งหนึ่งของจังหวัดน่าน โดดเด่นด้วยสถาปัตยกรรมทรงจตุรมุขหนึ่งเดียวในประเทศไทย และภาพจิตรกรรมฝาผนัง 'ปู่ม่านย่าม่าน' หรือ 'กระซิบรักบันลือโลก' อันเลื่องชื่อ ตั้งอยู่ใจกลางเมืองน่าน ใกล้กับพิพิธภัณฑสถานแห่งชาติน่าน",
        "keywords": ["วัดภูมินทร์", "กระซิบรัก", "ปู่ม่านย่าม่าน", "เที่ยวน่าน", "วัดสวย"],
        "details": [
            {
                "heading": "ไฮไลท์ห้ามพลาด",
                "content": "**ภาพจิตรกรรมปู่ม่านย่าม่าน:** ภาพกระซิบรักบันลือโลก ผลงานของหนานบัวผัน จิตรกรพื้นถิ่นเชื้อสายไทลื้อ, **พระอุโบสถจตุรมุข:** สถาปัตยกรรมที่ผสมผสานระหว่างวิหาร อุโบสถ และเจดีย์เข้าไว้ด้วยกัน, **พระประธานจตุรทิศ:** พระพุทธรูปขนาดใหญ่ 4 องค์ หันพระพักตร์ออกไปทางประตูทั้ง 4 ทิศ"
            },
            {
                "heading": "ข้อมูลการเยี่ยมชม",
                "content": "**เวลาทำการ:** เปิดทุกวัน 06:00 - 18:00 น., **ค่าเข้าชม:** ฟรี (มีกล่องรับบริจาคบำรุงวัด), **การเดินทาง:** ตั้งอยู่ที่ถนนผากอง ตำบลในเวียง อำเภอเมืองน่าน"
            }
        ]
    }
    
    print(f"Updating document with slug: {target_slug}...")
    result = col.update_one({"slug": target_slug}, {"$set": new_data})
    
    if result.modified_count > 0:
        print("✅ Successfully updated Wat Phumin data.")
    else:
        print("⚠️ No document matched or data was already correct.")

    # Verify
    updated = col.find_one({"slug": target_slug})
    print(f"Current Title: {updated.get('title')}")
    print(f"Current Category: {updated.get('category')}")

if __name__ == "__main__":
    fix_data()
