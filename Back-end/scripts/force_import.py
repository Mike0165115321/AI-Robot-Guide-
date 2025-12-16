#!/usr/bin/env python3
"""
Force Import Script - Import all data directly into MongoDB and Qdrant
This script bypasses the "already processed" check and imports everything
"""
import sys
import os
import json
import asyncio

# Add backend to path
current_dir = os.path.dirname(os.path.abspath(__file__))
backend_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.insert(0, backend_dir)

from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from utils.helper_functions import create_synthetic_document

async def force_import():
    print("\n" + "="*60)
    print("🚀 FORCE IMPORT - Restoring MongoDB + Qdrant Data")
    print("="*60)
    
    # Initialize managers
    mongo = MongoDBManager()
    qdrant = QdrantManager()
    await qdrant.initialize()
    
    # Path to JSON data
    data_file = os.path.join(backend_dir, 'core', 'database', 'data', 'superdata_filtered_attractions.json')
    
    if not os.path.exists(data_file):
        print(f"❌ Data file not found: {data_file}")
        return
    
    # Load data
    print(f"📂 Loading data from: {data_file}")
    with open(data_file, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    print(f"📊 Found {len(data)} records to import")
    
    # Clear existing data (optional - comment out if you want to keep existing)
    collection = mongo.get_collection("nan_locations")
    if collection is not None:
        existing_count = collection.count_documents({})
        print(f"🗑️  Clearing {existing_count} existing records...")
        collection.delete_many({})
    
    # Import each record
    success_count = 0
    error_count = 0
    
    for i, item in enumerate(data):
        try:
            slug = item.get("slug", f"unknown-{i}")
            
            # Generate embedding text
            embedding_text = create_synthetic_document(item)
            
            # Insert into MongoDB
            mongo_id = mongo.add_location(item)
            
            if mongo_id:
                # Insert into Qdrant
                qdrant_metadata = {
                    "title": item.get("title"),
                    "slug": item.get("slug"),
                    "category": item.get("category"),
                    "district": (item.get("location_data") or {}).get("district"),
                    "sub_district": (item.get("location_data") or {}).get("sub_district")
                }
                await qdrant.upsert_location(mongo_id, embedding_text, metadata=qdrant_metadata)
                success_count += 1
                if (i + 1) % 10 == 0:
                    print(f"  ✅ Processed {i + 1}/{len(data)} records...")
        except Exception as e:
            error_count += 1
            print(f"  ❌ Error on record {i} ({slug}): {e}")
    
    # Close connections
    await qdrant.close()
    
    print("\n" + "="*60)
    print(f"✅ Import Complete!")
    print(f"   - Successfully imported: {success_count}")
    print(f"   - Errors: {error_count}")
    print("="*60)

if __name__ == "__main__":
    asyncio.run(force_import())
