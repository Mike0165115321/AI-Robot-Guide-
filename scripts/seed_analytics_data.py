
import asyncio
import sys
import os
import random
from datetime import datetime, timedelta, timezone

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Back-end")))

from core.database.mongodb_manager import MongoDBManager

# Mock Data Pools
TOPICS = ["Culture", "Food", "Nature", "Accommodation", "Travel", "History", "Activities"]
LOCATIONS = [
    "วัดภูมินทร์", "ดอยเสมอดาว", "บ่อเกลือสินเธาว์", "พิพิธภัณฑสถานแห่งชาติน่าน", 
    "วัดพระธาตุแช่แห้ง", "ถนนคนเดินน่าน", "อุทยานแห่งชาติขุนสถาน", "เสาดินนาน้อย",
    "ร้านกาแฟบ้านไทลื้อ", "วัดมิ่งเมือง"
]
PROVINCES = ["กรุงเทพมหานคร", "เชียงใหม่", "น่าน", "ชลบุรี", "ขอนแก่น", "ภูเก็ต", "นครราชสีมา", "สงขลา"]
ORIGINS = ["Thailand", "Thailand", "Thailand", "Thailand", "China", "United States", "Japan", "France", "United Kingdom"]
QUERIES = [
    "แนะนำวัดสวยๆ หน่อย", "ร้านอาหารอร่อยในเมืองน่าน", "ที่พักดอยเสมอดาวว่างไหม", 
    "ประวัติวัดภูมินทร์", "การเดินทางไปบ่อเกลือ", "ของฝากน่านมีอะไรบ้าง", 
    "ร้านกาแฟวิวสวย", "เช่ารถมอเตอร์ไซค์ที่ไหน", "สภาพอากาศช่วงนี้", "วัดเปิดกี่โมง"
]

FEEDBACK_TYPES = ["like", "like", "like", "like", "dislike"] # 80% Like

async def seed_data():
    print("🌱 Starting Analytics Data Seeding...")
    mongo = MongoDBManager()
    analytics_col = mongo.get_collection("analytics_logs")
    feedback_col = mongo.get_collection("feedback_logs")

    if analytics_col is None or feedback_col is None:
        print("❌ Failed to access collections.")
        return

    # Configuration
    NUM_LOGS = 150  # Total conversation logs
    NUM_FEEDBACK = 80 # Total feedback logs
    DAYS_BACK = 30

    new_analytics = []
    new_feedback = []

    # Generate Analytics Logs
    for i in range(NUM_LOGS):
        timestamp = datetime.now(timezone.utc) - timedelta(
            days=random.randint(0, DAYS_BACK),
            hours=random.randint(0, 23),
            minutes=random.randint(0, 59)
        )
        
        entry = {
            "session_id": f"seed_session_{i}",
            "timestamp": timestamp,
            "user_query": random.choice(QUERIES),
            "ai_response": "This is a mock response.",
            "interest_topic": random.choice(TOPICS),
            "location_title": random.choice(LOCATIONS),
            "user_origin": random.choice(ORIGINS),
            "user_province": random.choice(PROVINCES) if random.random() > 0.3 else None,
            "meta": {"is_mock": True}
        }
        new_analytics.append(entry)

    # Generate Feedback Logs
    for i in range(NUM_FEEDBACK):
        timestamp = datetime.now(timezone.utc) - timedelta(
            days=random.randint(0, DAYS_BACK),
            hours=random.randint(0, 23)
        )
        
        entry = {
            "session_id": f"seed_session_{random.randint(0, NUM_LOGS)}",
            "timestamp": timestamp,
            "user_query": "Mock query for feedback",
            "feedback_type": random.choice(FEEDBACK_TYPES),
            "reason": None,
            "meta": {"is_mock": True}
        }
        new_feedback.append(entry)

    # Bulk Insert
    if new_analytics:
        # Use asyncio.to_thread for blocking PyMongo calls
        await asyncio.to_thread(analytics_col.insert_many, new_analytics)
        print(f"✅ Inserted {len(new_analytics)} analytics logs.")

    if new_feedback:
        await asyncio.to_thread(feedback_col.insert_many, new_feedback)
        print(f"✅ Inserted {len(new_feedback)} feedback logs.")

    print("\n🎉 Seeding Complete! Please refresh your dashboard.")

if __name__ == "__main__":
    asyncio.run(seed_data())
