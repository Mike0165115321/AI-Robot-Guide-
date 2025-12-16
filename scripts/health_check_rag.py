import asyncio
import sys
import os
import requests
import logging

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Back-end")))

from core.database.mongodb_manager import MongoDBManager
from core.ai_models.rag_orchestrator import RAGOrchestrator
from core.config import settings

# Configure logging
logging.basicConfig(level=logging.ERROR) # Only show errors by default

async def check_health():
    print("\n🏥 RAG System Health Check")
    print("----------------------------")
    
    # 1. MongoDB Check
    print("🔹 Checking MongoDB Connection...")
    try:
        mongo = MongoDBManager()
        col = mongo.get_collection("nan_locations")
        count = col.count_documents({})
        print(f"   ✅ MongoDB Connected. Found {count} locations.")
    except Exception as e:
        print(f"   ❌ MongoDB Failed: {e}")
        return

    # 2. Qdrant Check
    print("\n🔹 Checking Qdrant Connection...")
    try:
        url = f"http://{settings.QDRANT_HOST}:{settings.QDRANT_PORT}/collections" # Add http://
        response = requests.get(url)
        if response.status_code == 200:
            print(f"   ✅ Qdrant Accessible. Code: {response.status_code}")
        else:
            print(f"   ⚠️ Qdrant Status: {response.status_code}")
    except Exception as e:
        print(f"   ❌ Qdrant Failed: {e}")

    # 3. RAG Orchestrator Logic Check
    print("\n🔹 Checking RAG Orchestrator Logic...")
    
    # Initialize Dependencies
    from core.database.qdrant_manager import QdrantManager
    from core.ai_models.query_interpreter import QueryInterpreter
    from core.ai_models.services.prompt_engine import PromptEngine
    
    qdrant = QdrantManager()
    interpreter = QueryInterpreter()
    prompt_engine = PromptEngine()
    
    # Note: RAGOrchestrator might take different args depending on latest refactor.
    # Checking __init__ signature... it typically needs mongo, qdrant, interpreter.
    rag = RAGOrchestrator(
        mongo_manager=mongo, 
        qdrant_manager=qdrant, 
        query_interpreter=interpreter
    )
    
    # Check 3.1: Specific Query (Semantic Search)
    specific_query = "วัดภูมินทร์"
    print(f"   🔍 Testing Specific Query: '{specific_query}' from KB...")
    try:
        start_time = asyncio.get_event_loop().time()
        result_specific = await rag.answer_query(
            session_id="health_check_session",
            query=specific_query,
            mode="text", ai_mode="fast"
        )
        duration = asyncio.get_event_loop().time() - start_time
        
        if result_specific.get('sources'):
            print(f"      ✅ RAG Retrieval Works. Found {len(result_specific['sources'])} sources in {duration:.2f}s.")
            print(f"      📝 Answer Preview: {result_specific['answer'][:50]}...")
        else:
            print(f"      ⚠️ RAG returned no sources for '{specific_query}'. Answer: {result_specific['answer'][:50]}...")
            
    except Exception as e:
        print(f"      ❌ RAG Specific Query Error: {e}")

    # Check 3.2: Broad Query (Trending Functionality)
    broad_query = "แนะนำที่เที่ยวหน่อย"
    print(f"\n   🔥 Testing Broad Query (Trending): '{broad_query}'...")
    try:
        start_time = asyncio.get_event_loop().time()
        result_broad = await rag.answer_query(
            session_id="health_check_session",
            query=broad_query,
            mode="text", ai_mode="fast"
        )
        duration = asyncio.get_event_loop().time() - start_time
        
        has_trending = False
        # To verify trending, we can check logs or infer from answer if it mentions popular spots,
        # but better yet, let's check if sources contain expected trending items (from our test logic).
        # However, generate_answer returns a final payload, not internal context.
        # We can look for the fire emoji likely injected in titles if configured.
        
        sources = result_broad.get('sources', [])
        found_fire = any("🔥" in s.get('title', '') for s in sources)
        
        if found_fire:
             print(f"      ✅ Trending Injection Detected (Found '🔥' in titles).")
        else:
             print(f"      ⚠️ No Trending Indicators in sources. (Maybe prompt stripped them or trending logic didn't trigger).")
             
        if sources:
            print(f"      ✅ Broad Query Returned {len(sources)} sources in {duration:.2f}s.")
        else:
             print(f"      ⚠️ Broad Query returned NO sources.")

    except Exception as e:
        print(f"      ❌ RAG Broad Query Error: {e}")

    print("\n✅ Health Check Complete.")

if __name__ == "__main__":
    asyncio.run(check_health())
