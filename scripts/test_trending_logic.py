import asyncio
import sys
import os
import logging

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Back-end")))

from core.database.mongodb_manager import MongoDBManager
from core.services.analytics_service import AnalyticsService

# Configure logging
logging.basicConfig(level=logging.INFO)

async def test_trending():
    print("\n🧪 Testing Smart Trending Recommendations Logic...")
    
    mongo_manager = MongoDBManager()
    analytics_service = AnalyticsService(mongo_manager)
    
    # Test 1: Get Trending Titles
    print("\n🔹 [Test 1] Fetching Top 5 Trending Locations...")
    trending_titles = await analytics_service.get_trending_locations(limit=5)
    print(f"   🔥 Result: {trending_titles}")
    
    if not trending_titles:
        print("   ❌ Failed to fetch trending titles (or seeding failed?)")
        return

    # DEBUG: Check DB content
    col = mongo_manager.get_collection("nan_locations")
    print(f"\nExample doc in DB: {col.find_one({}, {'title':1, '_id':0})}")
    
    # DEBUG: Try finding ONE title manually
    test_title = trending_titles[0]
    print(f"   🔍 Trying manual find for: {repr(test_title)}")
    
    # Check exact match
    found = col.find_one({"title": test_title})
    if found:
        print(f"      ✅ Manual find succeeded for: {found.get('title')}")
    else:
        print(f"      ❌ Manual find FAILED.")
        # Try Regex
        print("      🔍 Attempting Regex search...")
        found_regex = list(col.find({"title": {"$regex": test_title[:10]}})) # Match first 10 chars
        for f in found_regex:
            print(f"      ⚠️ Found similar: {repr(f.get('title'))}")

    # Test 2: Verify MongoDB Fetch by Titles
    print("\n🔹 [Test 2] Fetching Full Docs by Titles...")
    docs = mongo_manager.get_locations_by_titles(trending_titles)
    print(f"   📚 Found {len(docs)} documents.")
    for d in docs:
        print(f"       - {d.get('title')}")
        
    if len(docs) > 0:
        print("   ✅ Full doc fetch successful.")
    else:
        print(f"   ⚠️ Mismatch: Asked for {len(trending_titles)}, got {len(docs)}")

if __name__ == "__main__":
    asyncio.run(test_trending())
