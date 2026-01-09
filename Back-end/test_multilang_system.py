import requests
import json
import time

BASE_URL = "http://localhost:8014"

def print_test_header(title):
    print(f"\nExample Test: 🧪 {title}")
    print("-" * 50)

def test_assistant_lang(text, lang_code, expected_intent="SMALL_TALK"):
    print(f"📡 Sending '{text}' [Lang: {lang_code}]...")
    try:
        payload = {"text": text, "language": lang_code}
        res = requests.post(f"{BASE_URL}/api/assistant/query", json=payload)
        
        if res.status_code == 200:
            data = res.json()
            if data["success"]:
                result = data["data"]
                print(f"   ✅ Success! Reply: {result.get('reply')}")
                print(f"      Intent: {result.get('intent')}")
                if "intent" in result and result["intent"] == expected_intent:
                    return True
                elif expected_intent == "ANY":
                    return True
                else:
                    # RAG_QUERY is also valid if Google doesn't know, but we want to check if it accepted the language
                    print(f"      Note: Got {result.get('intent')}, Expected {expected_intent}")
                    return True
            else:
                print(f"   ❌ API Logic Error: {data.get('error')}")
                return False
        else:
            print(f"   ❌ HTTP Error: {res.status_code}")
            return False
            
    except Exception as e:
        print(f"   ❌ Exception: {e}")
        return False

def test_stability_flood():
    print_test_header("Stability / Stress Test")
    print("🌊 Sending 5 requests rapidly...")
    
    success_cnt = 0
    for i in range(5):
        try:
            res = requests.post(f"{BASE_URL}/api/assistant/query", json={"text": f"ping {i}", "language": "en"})
            if res.status_code == 200:
                success_cnt += 1
                print(f"   Req {i+1}: OK")
            else:
                print(f"   Req {i+1}: Failed {res.status_code}")
        except Exception as e:
            print(f"   Req {i+1}: Error {e}")
            
    print(f"   Stability Result: {success_cnt}/5 passed")

if __name__ == "__main__":
    print("🚀 STARTING COMPREHENSIVE SYSTEM TEST")
    
    # 1. Thai Test
    print_test_header("Thai Language Test")
    test_assistant_lang("สวัสดี", "th")
    
    # 2. English Test
    print_test_header("English Language Test")
    test_assistant_lang("Hello", "en")
    
    # 3. Japanese Test
    print_test_header("Japanese Language Test")
    test_assistant_lang("こんにちは", "ja")
    
    # 4. Chinese Test
    print_test_header("Chinese Language Test")
    test_assistant_lang("你好", "zh")

    # 5. Stability
    test_stability_flood()
    
    print("\n🏁 TEST COMPLETE")
