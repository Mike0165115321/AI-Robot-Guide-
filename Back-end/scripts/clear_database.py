import sys
import os
import asyncio  # [V5.1] เพิ่ม

# --- Setup path ---
current_script_dir = os.path.dirname(os.path.abspath(__file__))
backend_dir = os.path.abspath(os.path.join(current_script_dir, '..'))
sys.path.insert(0, backend_dir)

from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from qdrant_client import models
from core.config import settings # [V5.1] เพิ่ม

# [V5.1] เปลี่ยนเป็น async def
async def clear_all_data_async():
    print("="*60)
    print("--- ⚠️  WARNING: This will delete all data in the collections! ⚠️ ---")
    print("="*60)
    
    confirm = input("Are you sure you want to proceed? (yes/no): ")
    if confirm.lower() != 'yes':
        print("Aborted by user.")
        return

    # --- 1. Clear MongoDB Collection ---
    print("\n--- Clearing MongoDB Collection ---")
    mongo = MongoDBManager()
    collection = mongo.get_collection("nan_locations")
    if collection is not None:
        delete_result = collection.delete_many({})
        print(f"✅ Deleted {delete_result.deleted_count} documents from MongoDB collection 'nan_locations'.")
    else:
        print("❌ Could not connect to MongoDB.")

    # --- 2. Clear Qdrant Collection (Async) ---
    print("\n--- Clearing Qdrant Collection ---")
    qdrant = None
    try:
        qdrant = QdrantManager()
        
        # [V5.1] ต้องเรียก initialize() ก่อน
        await qdrant.initialize()
        
        print(f"Recreating Qdrant collection '{qdrant.collection_name}'...")
        
        # [V5.1] ต้อง await การเรียก
        await qdrant.client.recreate_collection(
            collection_name=qdrant.collection_name,
            vectors_config=models.VectorParams(
                size=qdrant.embedding_model.get_sentence_embedding_dimension(),
                distance=models.Distance.COSINE
            )
        )
        print("✅ Qdrant collection has been cleared and recreated.")
        
    except Exception as e:
        print(f"❌ An error occurred while clearing Qdrant collection: {e}")
    finally:
        # [V5.1] ต้อง close() client
        if qdrant:
            await qdrant.close()

    print("\n--- ✨ Database clearing process finished. ---")
    print("You can now run 'scripts/build_vectors.py' to re-index all data.")

if __name__ == "__main__":
    # [V5.1] รันฟังก์ชัน async
    asyncio.run(clear_all_data_async())