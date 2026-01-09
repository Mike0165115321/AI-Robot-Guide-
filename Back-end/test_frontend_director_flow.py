import requests
import json
import time

BASE_URL = "http://localhost:8014"

def print_header(title):
    print(f"\n{'='*50}")
    print(f"🎬 TESTING: {title}")
    print(f"{'='*50}")

def mock_frontend_director(query):
    print(f"\n👤 USER SAYS: '{query}'")
    
    # --- STEP 0: Local Logic (Simulation) ---
    print("   [Step 0] Checking Local Logic...")
    local_keywords = ["เต้น", "dance", "หัวเราะ", "laugh"]
    if any(k in query.lower() for k in local_keywords):
        print(f"   ✅ DIRECTOR: Handled locally! (Found keyword)")
        return
    else:
        print("   ❌ DIRECTOR: No local match. Proceeding...")

    # --- STEP 1: Ask Google Assistant Proxy ---
    print("   [Step 1] Asking Google Assistant (Proxy)...")
    try:
        start_t = time.time()
        response = requests.post(f"{BASE_URL}/api/assistant/query", json={"text": query})
        process_time = time.time() - start_t
        
        if response.status_code != 200:
            print(f"   ❌ ERROR: API failed {response.status_code}")
            return

        data = response.json()
        if not data["success"]:
             print(f"   ❌ ERROR: Backend returned failure: {data.get('error')}")
             return

        result = data["data"]
        intent = result.get("intent")
        reply = result.get("reply")
        
        print(f"      ---> Intent: {intent}")
        print(f"      ---> Reply:  {reply}")
        print(f"      ---> Time:   {process_time:.2f}s")
        
        # Decision Logic (Similar to app.js _isAssistantUseful)
        is_useful = True
        if intent == "RAG_QUERY": is_useful = False
        if not reply: is_useful = False
        
        if is_useful:
            print("   ✅ DIRECTOR: Google Answer Accepted! (Stopping here)")
            return
        else:
            print("   ⚠️ DIRECTOR: Google doesn't know. Falling back to RAG...")

    except Exception as e:
        print(f"   ❌ Network Error: {e}")
        return

    # --- STEP 2: Ask RAG (Simulation) ---
    print("   [Step 2] Asking RAG System...")
    try:
        # We assume RAG endpoint is /api/chat with plain text
        # (The actual app.js calls chatService which wraps this)
        rag_payload = {
            "query": query,
            "session_id": "test_director_session",
            "ai_mode": "fast"
        }
        res_rag = requests.post(f"{BASE_URL}/api/chat/", json=rag_payload)
        
        if res_rag.status_code == 200:
            rag_data = res_rag.json()
            print(f"   ✅ DIRECTOR: RAG Answered!")
            print(f"      ---> Answer: {rag_data.get('answer')[:100]}...") # truncate
        else:
            print(f"   ❌ RAG Error: {res_rag.status_code}")

    except Exception as e:
        print(f"   ❌ RAG Network Error: {e}")

if __name__ == "__main__":
    # Test 1: Local Logic
    mock_frontend_director("เต้นให้ดูหน่อย")
    
    # Test 2: Google Assistant (Should succeed)
    mock_frontend_director("Hello")
    
    # Test 3: RAG Fallback (Should fail Google -> Go to RAG)
    mock_frontend_director("วัดภูมินทร์สร้างปีไหน") # Deep question
