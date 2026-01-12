import json
import urllib.request
import urllib.error
import time
import sys

BASE_URL = "http://localhost:8014"

def run_test(name, func):
    print(f"🧪 Testing {name}...", end=" ", flush=True)
    try:
        func()
        print("✅ PASS")
        return True
    except Exception as e:
        print(f"❌ FAIL: {e}")
        return False

def test_health():
    with urllib.request.urlopen(f"{BASE_URL}/health") as response:
        if response.status != 200:
            raise Exception(f"Status code: {response.status}")
        data = json.loads(response.read().decode())
        if data.get("status") != "healthy":
            raise Exception(f"Unhealthy status: {data}")

def test_send_text():
    payload = {
        "query": "สวัสดีครับ ลองทดสอบระบบ",
        "session_id": "test_session_123",
        "ai_mode": "fast"
    }
    req = urllib.request.Request(
        f"{BASE_URL}/api/chat/",
        data=json.dumps(payload).encode(),
        headers={'Content-Type': 'application/json'},
        method='POST'
    )
    with urllib.request.urlopen(req) as response:
        if response.status != 200:
            raise Exception(f"Status code: {response.status}")
        data = json.loads(response.read().decode())
        if "answer" not in data:
            raise Exception("No 'answer' in response")

def test_tts():
    test_cases = [
        ("th", "ทดสอบเสียงภาษาไทย"),
        ("en", "Testing English TTS capability"),
        ("ja", "こんにちは"),       # Japanese
        ("zh", "你好"),             # Chinese
        ("hi", "नमस्ते"),           # Hindi
        ("ms", "Selamat pagi"),    # Malay
        ("ru", "Привет")            # Russian
    ]
    
    for lang, text in test_cases:
        print(f"\n    🗣️  Language [{lang}]: {text} ...", end=" ")
        payload = {
            "text": text,
            "language": lang
        }
        req = urllib.request.Request(
            f"{BASE_URL}/api/chat/tts",
            data=json.dumps(payload).encode(),
            headers={'Content-Type': 'application/json'},
            method='POST'
        )
        with urllib.request.urlopen(req) as response:
            if response.status != 200:
                raise Exception(f"Status code: {response.status}")
            if response.headers.get("Content-Type") not in ["audio/mpeg", "audio/wav", "audio/webm"]:
                 pass
            content = response.read()
            if len(content) < 100:
                 raise Exception(f"Audio content too small for {lang}")
            print("✅ OK", end="")
    print() # Newline after loop

def test_alerts():
    req = urllib.request.Request(f"{BASE_URL}/api/alerts/recent", method='GET')
    with urllib.request.urlopen(req) as response:
        if response.status != 200: raise Exception(f"Status: {response.status}")
        data = json.loads(response.read().decode())
        # Response structure: {"success": True, "count": ..., "alerts": []}
        if not data.get("success"): raise Exception("Alert API failed")
        if "alerts" not in data: raise Exception("No alerts field")

def test_google_assistant_proxy():
    # Test if endpoint exists (422 expected for empty body, 200 if valid)
    req = urllib.request.Request(
        f"{BASE_URL}/api/assistant/query", 
        data=b"{}", 
        headers={'Content-Type': 'application/json'},
        method='POST'
    )
    try:
        with urllib.request.urlopen(req) as response:
            pass 
    except urllib.error.HTTPError as e:
        if e.code not in [422, 400, 200]:
            raise Exception(f"Unexpected status: {e.code}")

def test_analytics_status():
    req = urllib.request.Request(f"{BASE_URL}/api/analytics/status", method='GET')
    with urllib.request.urlopen(req) as response:
        if response.status != 200: raise Exception(f"Status: {response.status}")

if __name__ == "__main__":
    print(f"🚀 Starting Backend Tests on {BASE_URL}")
    print("=" * 40)
    
    # Wait for server if needed (simple check)
    try:
        urllib.request.urlopen(f"{BASE_URL}/health", timeout=2)
    except:
        print(f"⚠️ Could not connect to {BASE_URL}. Is the server running?")
        sys.exit(1)

    tests = [
        ("Health Check", test_health),
        ("Send Text API", test_send_text),
        ("TTS API (Multi-lang)", test_tts),
        ("Alerts API", test_alerts),
        ("Google Assistant Proxy", test_google_assistant_proxy),
        ("Analytics Status", test_analytics_status)
    ]

    passed = 0
    for name, func in tests:
        if run_test(name, func):
            passed += 1

    print("=" * 40)
    print(f"📊 Result: {passed}/{len(tests)} Passed")
    
    if passed == len(tests):
        sys.exit(0)
    else:
        sys.exit(1)
