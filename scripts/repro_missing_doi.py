
import asyncio
import sys
import os
import logging

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Back-end")))

# Configure logging
logging.basicConfig(level=logging.INFO)

from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from core.ai_models.query_interpreter import QueryInterpreter
from core.ai_models.rag_orchestrator import RAGOrchestrator
from core.ai_models.services.prompt_engine import PromptEngine

async def reproduce_issue():
    print("\n🔍 Reproducing 'Missing Doi Samer Dao' Issue")
    
    # Initialize components
    mongo = MongoDBManager()
    qdrant = QdrantManager()
    interpreter = QueryInterpreter()
    prompt_engine = PromptEngine()
    rag = RAGOrchestrator(mongo, qdrant, interpreter)
    
    query = "รู้จักดอยเสมอดาวมั้ยครับ"
    print(f"👉 Query: '{query}'")

    # 1. Test Interpreter extraction
    print("\n1️⃣  Testing Query Interpreter...")
    interpretation = await interpreter.interpret_and_route(query)
    print(f"   Intent: {interpretation.get('intent')}")
    print(f"   Entity: {interpretation.get('entity')}")
    print(f"   Category: {interpretation.get('category')}")

    # 2. Test Full RAG Flow
    print("\n2️⃣  Testing RAG Flow...")
    result = await rag.answer_query(query=query, mode="text", ai_mode="fast")
    
    print("\n📝 Result Answer Snippet:")
    print(f"   {result.get('answer')[:200]}...")
    
    print("\n🖼️  Result Images:")
    for img in result.get('image_gallery', []):
        print(f"   - {img}")
    
    print("\n📚 Result Sources:")
    for src in result.get('sources', []):
        print(f"   - Title: {src.get('title')}")
        print(f"     Is Trending: {src.get('is_trending')}")
        print(f"     Is Direct Match: {src.get('is_direct_match')}") # Check for new flag
        print(f"     Origin: {src.get('origin', 'Standard Search')}")

if __name__ == "__main__":
    asyncio.run(reproduce_issue())
