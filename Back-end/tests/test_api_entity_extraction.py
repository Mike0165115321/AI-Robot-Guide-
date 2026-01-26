"""
🚀 E2E Test: Entity Extraction via API
ทดสอบโดยการยิง Request จริงไปที่ Server (http://localhost:8014)
ไม่ต้อง Mock! ทดสอบระบบจริงทั้งระบบ

วิธีรัน:
    python Back-end/tests/test_api_entity_extraction.py
"""

import requests
import json
import time

API_URL = "http://localhost:8014/api/chat/text"

# Test Cases
TEST_QUERY = "อยากไปวัดพระธาตุเขาน้อย รู้จักมั้ยครับ"
EXPECTED_ENTITY = "วัดพระธาตุเขาน้อย"

def test_api():
    print(f"📡 Sending Request to {API_URL}...")
    print(f"❓ Query: '{TEST_QUERY}'")
    
    payload = {
        "message": TEST_QUERY,
        "session_id": "test_script_session_001",
        "mode": "detailed"
    }
    
    try:
        start_time = time.time()
        response = requests.post(API_URL, json=payload)
        duration = time.time() - start_time
        
        if response.status_code != 200:
            print(f"❌ API Error: {response.status_code}")
            print(response.text)
            return
            
        data = response.json()
        print(f"✅ Response Received in {duration:.2f}s")
        
        # Analyze Response
        action = data.get("action")
        print(f"🎯 Action: {action}")
        
        if action == "SHOW_MAP_EMBED":
            payload = data.get("action_payload", {})
            dest = payload.get("destination_name")
            print(f"📍 Destination: '{dest}'")
            
            if EXPECTED_ENTITY in dest:
                 print("\n🎉 SUCCESS! ระบบเข้าใจและนำทางถูกต้อง")
            else:
                 print(f"\n❌ FAILED! นำทางผิดที่ (Expected: {EXPECTED_ENTITY}, Got: {dest})")
        else:
            print("\n❌ FAILED! ระบบไม่ได้ตอบสนองด้วยการนำทาง (SHOW_MAP_EMBED)")
            print(f"Answer: {data.get('answer')}")

    except Exception as e:
        print(f"❌ Connection Error: {e}")
        print("💡 ตรวจสอบว่า Server รันอยู่หรือไม่ (./start_web.sh)")

if __name__ == "__main__":
    test_api()
