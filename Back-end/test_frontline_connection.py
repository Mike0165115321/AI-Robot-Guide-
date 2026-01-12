import os
import asyncio
import sys
import logging

# Set env var for protobuf fix
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
# Ensure we look for creds in current dir
os.environ["GOOGLE_ASSISTANT_CREDENTIALS"] = "assistant_credentials.json" 

# Add current directory to path
sys.path.append(os.getcwd())

# Configure basic logging to see handler output
logging.basicConfig(level=logging.ERROR) 

try:
    from core.ai_models.frontline_handler import frontline_handler
except ImportError as e:
    print(f"Import Error: {e}")
    sys.exit(1)

async def main():
    print("========================================")
    print("🧪 Testing Google Assistant Connection")
    print("========================================")
    
    # Test 1: Greeting
    query = "Hello"
    print(f"\n🔹 Query 1: '{query}'")
    print("   Sending...")
    
    try:
        result = await frontline_handler.process_query(query)
        print(f"   ► Result: {result}")
        
        if result["intent"] != "RAG_QUERY" and result.get("reply"):
            print("   ✅ PASS: Assistant replied successfully.")
        else:
            print("   ❌ FAIL: Routed to RAG or no reply.")
            
    except Exception as e:
        print(f"   ❌ ERROR: {e}")

    # Test 2: Specific Question (Expect RAG Fallback)
    query = "วัดภูมินทร์สร้างพ.ศ.อะไร"
    print(f"\n🔹 Query 2: '{query}' (Deep Knowledge)")
    print("   Sending...")
    
    try:
        result = await frontline_handler.process_query(query)
        print(f"   ► Result: {result}")
        
        if result["intent"] == "RAG_QUERY":
            print("   ✅ PASS: Correctly routed to RAG.")
        else:
            print(f"   ⚠️ NOTE: Assistant attempted to answer: {result.get('reply')}")
            
    except Exception as e:
        print(f"   ❌ ERROR: {e}")

    print("\n========================================")

if __name__ == "__main__":
    asyncio.run(main())
