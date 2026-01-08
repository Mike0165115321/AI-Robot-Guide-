import asyncio
import sys
import os
import logging

# Add backend to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from core.database.qdrant_manager import QdrantManager
from core.config import settings

# Configure logging
logging.basicConfig(level=logging.INFO)

async def debug_qdrant():
    print("🔍 Initializing Qdrant Manager...")
    qdrant = QdrantManager()
    
    # We don't need to full initialize if we just want to check data, 
    # but creating the client is enough.
    
    try:
        collection_info = await qdrant.client.get_collection(qdrant.collection_name)
        print(f"✅ Collection found: {collection_info.status}")
        print(f"📊 Points count: {collection_info.points_count}")
        
        if collection_info.points_count == 0:
            print("⚠️ Collection is empty!")
            return

        # Scroll 5 points to inspect payload
        print("\n🔍 Inspecting first 5 points:")
        scroll_result = await qdrant.client.scroll(
            collection_name=qdrant.collection_name,
            limit=5,
            with_payload=True,
            with_vectors=False
        )
        
        points, _ = scroll_result
        for p in points:
            print(f"ID: {p.id}")
            print(f"Payload: {p.payload}")
            print("-" * 40)
            
        # Test specific filter
        print("\n🧪 Testing specific filter: category='attraction'")
        from qdrant_client import models
        
        # Construct filter manually as in qdrant_manager
        conditions = [models.FieldCondition(
            key="category", 
            match=models.MatchValue(value="attraction")
        )]
        
        q_filter = models.Filter(must=conditions)
        
        # Count points with this filter using scroll (lazy count)
        count_result = await qdrant.client.count(
            collection_name=qdrant.collection_name,
            count_filter=q_filter
        )
        
        print(f"🔢 Count with category='attraction': {count_result.count}")
        
        # Check distinct categories in the first batch
        categories = set(p.payload.get("category") for p in points if p.payload)
        print(f"\n🏷️ Sample categories found in DB: {categories}")
        
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        await qdrant.close()

if __name__ == "__main__":
    asyncio.run(debug_qdrant())
