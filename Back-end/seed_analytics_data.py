#!/usr/bin/env python3
"""
🧪 Seed Sample Data for Analytics Dashboard
สร้างข้อมูลจำลองสำหรับทดสอบ Dashboard

Usage:
    cd Back-end
    python seed_analytics_data.py
"""

import asyncio
import random
from datetime import datetime, timezone, timedelta
from pymongo import MongoClient
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# MongoDB Connection
MONGO_URI = os.getenv("MONGODB_URI", "mongodb://localhost:27017")
DB_NAME = "nan_guide_ai"


# Sample data pools
SAMPLE_QUERIES = [
    "วัดภูมินทร์เปิดกี่โมง",
    "ร้านอาหารอร่อยในเมืองน่าน",
    "ที่พักใกล้วัดพระธาตุแช่แห้ง",
    "การเดินทางไปน่านยังไง",
    "ถนนคนเดินน่านวันไหน",
    "ของฝากน่านมีอะไรบ้าง",
    "วัดพระธาตุแช่แห้งประวัติ",
    "ที่เที่ยวน่านยอดฮิต",
    "กาแฟน่านร้านไหนดี",
    "ภูมิอากาศน่านเป็นยังไง",
    "เทศกาลน่านมีอะไรบ้าง",
    "ล่องเรือแม่น้ำน่าน",
    "บ่อเกลือน่านอยู่ตรงไหน",
    "สะพุดน่านคืออะไร",
    "ผ้าทอน่านซื้อที่ไหน",
    "หมู่บ้านไทลื้อน่าน",
    "ดอยภูคาเปิดเที่ยวไหม",
    "น้ำตกน่านมีที่ไหนบ้าง",
    "ร้านข้าวซอยน่าน",
    "ประวัติเมืองน่าน",
    "สถานที่ถ่ายรูปสวยน่าน",
    "วิวสวยน่านตรงไหน",
    "ตลาดเช้าน่านอยู่ไหน",
    "ขนมน่านมีอะไรบ้าง",
    "ค่าเข้าวัดภูมินทร์เท่าไหร่",
]

SAMPLE_LOCATIONS = [
    "วัดภูมินทร์",
    "วัดพระธาตุแช่แห้ง",
    "วัดช้างค้ำวรวิหาร",
    "ถนนคนเดิน",
    "ตลาดเช้าน่าน",
    "ดอยภูคา",
    "บ่อเกลือ",
    "ปัว",
    "เมืองน่าน",
    "ท่าวังผา",
]

SAMPLE_TOPICS = [
    "สถานที่ท่องเที่ยว",
    "วัด",
    "ร้านอาหาร",
    "ที่พัก",
    "การเดินทาง",
    "วัฒนธรรม",
    "อาหาร",
    "กิจกรรม",
    "ธรรมชาติ",
    "ของฝาก",
]

SAMPLE_ORIGINS = [
    "Thai", "Thai", "Thai", "Thai", "Thai",  # Weight Thai higher
    "China", "Japan", "Korea", "USA", "France",
    "Germany", "UK", "Australia", "Singapore", "Malaysia"
]

SAMPLE_PROVINCES = [
    "กรุงเทพมหานคร", "กรุงเทพมหานคร", "กรุงเทพมหานคร",  # Weight Bangkok
    "เชียงใหม่", "เชียงราย", "ลำปาง", "ลำพูน",
    "พะเยา", "แพร่", "อุตรดิตถ์", "นครสวรรค์",
    "ขอนแก่น", "อุดรธานี", "ชลบุรี", "ภูเก็ต",
]

SAMPLE_INTENTS = [
    "INFORMATIONAL", "INFORMATIONAL", "INFORMATIONAL",  # Weight higher
    "NAVIGATION", "MUSIC", "SMALL_TALK", "FAQ"
]

SAMPLE_UNANSWERED = [
    "ค่าเข้าวัดภูมินทร์เท่าไหร่",
    "รถเมล์ไปน่านมีไหม",
    "โรงพยาบาลน่านอยู่ตรงไหน",
    "ATM ใกล้วัดภูมินทร์",
    "ร้านซักผ้าในเมืองน่าน",
    "สนามบินน่านอยู่ไกลไหม",
    "รถตู้ไปเชียงรายจากน่าน",
    "ปั๊มน้ำมันใกล้ดอยภูคา",
    "7-11 ที่ปัวมีไหม",
    "ร้านขายยาในน่าน",
    "รถเช่าน่านราคาเท่าไหร่",
    "วัดศรีพันต้นเปิดกี่โมง",
    "หอศิลป์ริมน่านเข้าฟรีไหม",
    "ตลาดนัดน่านวันอะไร",
    "ร้านซ่อมรถน่าน",
]


def generate_analytics_logs(count: int = 500):
    """Generate sample analytics logs"""
    logs = []
    now = datetime.now(timezone.utc)
    
    for i in range(count):
        # Random timestamp within last 30 days
        days_ago = random.randint(0, 30)
        hours_ago = random.randint(0, 23)
        timestamp = now - timedelta(days=days_ago, hours=hours_ago)
        
        query = random.choice(SAMPLE_QUERIES)
        
        log = {
            "session_id": f"session_{random.randint(1000, 9999)}_{i}",
            "timestamp": timestamp,
            "hour": timestamp.hour,
            "day_of_week": timestamp.weekday(),
            "user_query": query,
            "ai_response": f"ตอบคำถาม: {query[:20]}...",
            "interest_topic": random.choice(SAMPLE_TOPICS),
            "location_title": random.choice(SAMPLE_LOCATIONS) if random.random() > 0.3 else None,
            "user_origin": random.choice(SAMPLE_ORIGINS) if random.random() > 0.2 else None,
            "user_province": random.choice(SAMPLE_PROVINCES) if random.random() > 0.3 else None,
            "sentiment": random.choice(["Positive", "Neutral", "Negative"]),
            "response_time_ms": random.randint(100, 2000),
            "confidence_score": random.uniform(0.3, 0.95),
            "intent": random.choice(SAMPLE_INTENTS),
            "ai_mode": random.choice(["fast", "quality"]),
            "meta": {
                "query_length": len(query),
                "response_length": random.randint(50, 500)
            }
        }
        logs.append(log)
    
    return logs


def generate_feedback_logs(count: int = 100):
    """Generate sample feedback logs"""
    logs = []
    now = datetime.now(timezone.utc)
    
    # 70% like, 30% dislike
    for i in range(count):
        days_ago = random.randint(0, 30)
        timestamp = now - timedelta(days=days_ago)
        
        feedback_type = "like" if random.random() > 0.3 else "dislike"
        query = random.choice(SAMPLE_QUERIES)
        
        log = {
            "session_id": f"session_{random.randint(1000, 9999)}_{i}",
            "timestamp": timestamp,
            "user_query": query,
            "ai_response": f"ตอบคำถาม: {query[:30]}...",
            "feedback_type": feedback_type,
            "reason": "ตอบไม่ตรงคำถาม" if feedback_type == "dislike" else None
        }
        logs.append(log)
    
    return logs


def generate_knowledge_gaps(count: int = 15):
    """Generate sample unanswered questions"""
    gaps = []
    now = datetime.now(timezone.utc)
    
    for i, query in enumerate(SAMPLE_UNANSWERED[:count]):
        days_ago = random.randint(1, 14)
        first_asked = now - timedelta(days=days_ago)
        last_asked = now - timedelta(days=random.randint(0, days_ago))
        
        gap = {
            "query": query,
            "normalized_query": query.lower().strip(),
            "max_score": random.uniform(0.2, 0.44),
            "count": random.randint(1, 20),
            "first_asked": first_asked,
            "last_asked": last_asked,
            "status": "PENDING",
            "sessions": [f"session_{random.randint(1000, 9999)}" for _ in range(min(5, random.randint(1, 10)))],
            "context": None,
            "resolved_answer": None,
            "resolved_by": None,
            "resolved_at": None,
            "dismiss_reason": None
        }
        gaps.append(gap)
    
    return gaps


def main():
    print("🧪 เริ่มสร้างข้อมูลจำลอง...")
    
    # Connect to MongoDB
    client = MongoClient(MONGO_URI)
    db = client[DB_NAME]
    
    # Generate and insert analytics logs
    print("\n📊 กำลังสร้าง Analytics Logs...")
    analytics_logs = generate_analytics_logs(500)
    analytics_collection = db["analytics_logs"]
    # Insert new data (don't delete existing)
    result = analytics_collection.insert_many(analytics_logs)
    print(f"   ✅ เพิ่ม {len(result.inserted_ids)} analytics logs")
    
    # Generate and insert feedback logs
    print("\n👍 กำลังสร้าง Feedback Logs...")
    feedback_logs = generate_feedback_logs(100)
    feedback_collection = db["feedback_logs"]
    result = feedback_collection.insert_many(feedback_logs)
    print(f"   ✅ เพิ่ม {len(result.inserted_ids)} feedback logs")
    
    # Generate and insert knowledge gaps
    print("\n🧠 กำลังสร้าง Knowledge Gaps...")
    gaps = generate_knowledge_gaps(15)
    gaps_collection = db["unanswered_questions"]
    # Clear existing gaps first for clean demo
    gaps_collection.delete_many({})
    result = gaps_collection.insert_many(gaps)
    print(f"   ✅ เพิ่ม {len(result.inserted_ids)} knowledge gaps")
    
    # Print summary
    print("\n" + "="*50)
    print("📋 สรุปข้อมูลใน MongoDB:")
    print(f"   - analytics_logs: {analytics_collection.count_documents({})} รายการ")
    print(f"   - feedback_logs: {feedback_collection.count_documents({})} รายการ")
    print(f"   - unanswered_questions: {gaps_collection.count_documents({})} รายการ")
    print("="*50)
    
    print("\n✅ เสร็จสิ้น! ลองเปิด Dashboard เพื่อดูข้อมูลได้เลย")
    print("   📊 http://localhost:8014/admin/dashboard.html")
    print("   🧠 http://localhost:8014/admin/knowledge-gaps.html")
    
    client.close()


if __name__ == "__main__":
    main()
