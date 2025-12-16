
import asyncio
import sys
import os
import argparse

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Back-end")))

from core.database.mongodb_manager import MongoDBManager

async def find_location(keyword):
    print(f"🔍 Searching for keyword: '{keyword}'...")
    mongo = MongoDBManager()
    col = mongo.get_collection("nan_locations")
    
    if col is None:
        print("❌ Cannot access nan_locations collection.")
        return

    # Use Regex for flexible partial match
    results = list(col.find({"title": {"$regex": keyword, "$options": "i"}}))
    
    if results:
        print(f"✅ Found {len(results)} matches:")
        for doc in results:
            print(f"   - ID: {doc.get('_id')}")
            print(f"     Title: {doc.get('title')}")
            print(f"     Slug: {doc.get('slug')}")
            print(f"     Category: {doc.get('category')}")
            print("     ---")
    else:
        print("❌ No matches found.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python find_title.py <keyword>")
    else:
        keyword = sys.argv[1]
        asyncio.run(find_location(keyword))
