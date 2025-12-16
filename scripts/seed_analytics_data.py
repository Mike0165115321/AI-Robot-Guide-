
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

# Will be populated from DB
LOCATIONS = [] 

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

    # Fetch real locations from DB to ensure correlation
    nan_col = mongo.get_collection("nan_locations")
    if nan_col is not None:
        print("   🔍 Fetching real location titles from DB...")
        # Use aggregation to pick 20 random titles
        real_locs = list(nan_col.aggregate([
            {"$match": {"title": {"$exists": True, "$ne": ""}}},
            {"$sample": {"size": 20}}, 
            {"$project": {"title": 1, "_id": 0}}
        ]))
        for l in real_locs:
            if l.get('title'):
                LOCATIONS.append(l.get('title'))
        print(f"   ✅ Loaded {len(LOCATIONS)} real locations.")
    
    if not LOCATIONS:
        print("   ⚠️ No locations found in DB. Using mock data.")
        LOCATIONS.extend(["Mock Location A", "Mock Location B", "Mock Location C"])

    new_analytics = []
    new_feedback = []

    # Clear old data for clean test state
    print("   🧹 Clearing old analytics data...")
    analytics_col.delete_many({})
    feedback_col.delete_many({})

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
