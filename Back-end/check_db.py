
import os
from pymongo import MongoClient
from core.config import settings

def check_db():
    print("Connecting to Mongo:", settings.MONGO_URI)
    client = MongoClient(settings.MONGO_URI)
    db = client[settings.MONGO_DATABASE_NAME]
    col = db["nan_locations"]
    
    # 1. Search for Wat Phumin (try different variations)
    print("\n--- Searching for 'วัดภูมินทร์' ---")
    queries = [
        {"title": {"$regex": "วัดภูมินทร์"}},
        {"title": {"$regex": "Phumin"}},
        {"slug": {"$regex": "wat-phumin"}}
    ]
    
    found_any = False
    for q in queries:
        cursor = col.find(q)
        found = list(cursor)
        if found:
            found_any = True
            print(f"\nQuery {q} matched {len(found)} documents:")
            for doc in found:
                print(f"Title: {doc.get('title')}")
                print(f"Slug: {doc.get('slug')}")
                print(f"Doc Type: {doc.get('doc_type')}")
                print(f"Image Prefix: {doc.get('metadata', {}).get('image_prefix')}")
                print("-" * 20)
    
    if not found_any:
        print("No documents found for Wat Phumin.")

    # 2. Check total locations
    total = col.count_documents({"doc_type": "Location"})
    print(f"\nTotal 'Location' documents: {total}")
    
    # 3. Check all doc types
    pipeline = [{"$group": {"_id": "$doc_type", "count": {"$sum": 1}}}]
    types = list(col.aggregate(pipeline))
    print("\nDocument Types Distribution:")
    for t in types:
        print(f"{t['_id']}: {t['count']}")

if __name__ == "__main__":
    check_db()
