
import asyncio
import sys
import logging
from pathlib import Path

# Add project root to path
sys.path.append(str(Path(__file__).resolve().parent.parent / "Back-end"))

import sys
from unittest.mock import MagicMock
# Mock dependencies before import
sys.modules["gspread"] = MagicMock()
sys.modules["requests"] = MagicMock()
sys.modules["pymongo"] = MagicMock()
sys.modules["qdrant_client"] = MagicMock()
sys.modules["sentence_transformers"] = MagicMock()
sys.modules["numpy"] = MagicMock()
sys.modules["bson"] = MagicMock()
sys.modules["bson.objectid"] = MagicMock()

from core.services.google_sheets_service import GoogleSheetsService
# We don't strictly need real managers if we mock them, 
# but we import checking if they are used in type hints which might trigger imports.
# Since we mocked their deps, they should import fine or we can skip importing them 
# and just use the mocks defined below.
# from core.database.mongodb_manager import MongoDBManager 
# from core.database.qdrant_manager import QdrantManager

# Mock classes for testing without real connections
class MockMongoManager:
    def __init__(self):
        self.locations = {}
    
    def add_location(self, data):
        _id = str(len(self.locations) + 1)
        data["_id"] = _id
        self.locations[data['slug']] = data
        logging.info(f"[MockMongo] Added: {data['slug']}")
        return _id
        
    def update_location_by_slug(self, slug, data):
        if slug in self.locations:
            self.locations[slug].update(data)
            logging.info(f"[MockMongo] Updated: {slug}")
        else:
            logging.error(f"[MockMongo] Slug not found: {slug}")

    def get_all_locations(self):
        return list(self.locations.values())

    def get_location_by_slug(self, slug):
        return self.locations.get(slug)

class MockQdrantManager:
    def __init__(self):
        self.upserted_count = 0
    
    async def upsert_location(self, mongo_id, description, metadata):
        self.upserted_count += 1
        logging.info(f"[MockQdrant] ✅ Upserted vector for ID: {mongo_id} | Desc: {description[:30]}...")
        return True

async def run_verification():
    """
    Simulate a sync process to verify logic flow
    """
    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    print("\n🕵️‍♂️ --- Starting Sync Integrity Verification ---")
    
    # 1. Setup Mocks
    mock_mongo = MockMongoManager()
    mock_qdrant = MockQdrantManager()
    
    service = GoogleSheetsService(mongo_manager=mock_mongo, qdrant_manager=mock_qdrant)
    
    # 2. Simulate Detect Changes
    print("\n🧪 Test Case 1: Create New Location")
    sheet_data = [{
        "slug": "test-location",
        "title": "Test Location",
        "category": "Test",
        "topic": "Testing",
        "summary": "This is a test summary for vector sync."
    }]
    db_data = [] # Empty DB
    
    changes = service.detect_changes(sheet_data, db_data)
    print(f"   Changes detected: Create={len(changes['to_create'])}, Update={len(changes['to_update'])}")
    
    # 3. Simulate Async Sync
    print("   Apply changes to Mongo + Qdrant...")
    result = await service.sync_to_mongodb(changes)
    
    # 4. Assertions
    if result.created == 1 and mock_qdrant.upserted_count == 1:
        print("✅ PASS: New location created in Mongo AND upserted to Qdrant.")
    else:
        print(f"❌ FAIL: Created={result.created}, Qdrant Upserts={mock_qdrant.upserted_count}")

    print("\n🧪 Test Case 2: Update Existing Location")
    # Modify data
    sheet_data[0]["summary"] = "Updated summary content."
    # Simulate DB state (populated from step 1)
    db_data = mock_mongo.get_all_locations()
    
    changes = service.detect_changes(sheet_data, db_data)
    print(f"   Changes detected: Create={len(changes['to_create'])}, Update={len(changes['to_update'])}")
    
    # Reset counts
    mock_qdrant.upserted_count = 0
    result = await service.sync_to_mongodb(changes)
    
    if result.updated == 1 and mock_qdrant.upserted_count == 1:
        print("✅ PASS: Location updated in Mongo AND re-upserted to Qdrant.")
    else:
        print(f"❌ FAIL: Updated={result.updated}, Qdrant Upserts={mock_qdrant.upserted_count}")

    print("\n✨ Verification Complete.")

if __name__ == "__main__":
    asyncio.run(run_verification())
