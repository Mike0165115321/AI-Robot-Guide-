
import asyncio
import sys
import os
import logging

# Add project root to path (Priority 0 to override installed packages)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Back-end")))

# Configure logging
logging.basicConfig(level=logging.INFO)

from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from core.ai_models.query_interpreter import QueryInterpreter
from core.ai_models.rag_orchestrator_FIX import RAGOrchestrator

async def test_typo():
    print("\n🔍 Testing Typo Query: 'ดอสเสมอดาว' (Should find ดอยเสมอดาว)")
    
    # Initialize components
    mongo = MongoDBManager()
    qdrant = QdrantManager()
    interpreter = QueryInterpreter()
    rag = RAGOrchestrator(mongo, qdrant, interpreter)
    
    query = "รู้จักดอสเสมอดาวมั้ยครับ" # Typo intended
    print(f"👉 Query: '{query}'")

    # 1. Test Interpreter extraction & Correction
    print("\n1️⃣  Testing Query Interpreter (Does it auto-correct?)...")
    interpretation = await interpreter.interpret_and_route(query)
    print(f"   Corrected Query: {interpretation.get('corrected_query')}")
    print(f"   Intent: {interpretation.get('intent')}")
    print(f"   Entity: {interpretation.get('entity')}")
    
    # 3. Search via RAG (Using internal method for testing) with debugging enabled
    print("\nDEBUG: Calling rag.answer_query (Gemini Mode)...")
    result = await rag.answer_query(query=query, mode="text", ai_mode="detailed")
    
    print("\n📝 Result Answer Snippet:")
    print(f"   {result.get('answer')[:200]}...")
    
    print("\n📚 Result Sources:")
    for src in result.get('sources', []):
        print(f"   - Title: {src.get('title')}")
        print(f"     Is Direct Match: {src.get('is_direct_match')}")
        print(f"     Origin: {src.get('origin', 'Standard Search')}")

if __name__ == "__main__":
    print(f"DEBUG: RAGOrchestrator loaded from: {RAGOrchestrator.__module__}")
    try:
        import inspect
        print(f"DEBUG: RAGOrchestrator file: {inspect.getfile(RAGOrchestrator)}")
    except:
        print("DEBUG: Could not inspect file")

    asyncio.run(test_typo())
