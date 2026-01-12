import asyncio
import websockets
import json
import sys

# Install websockets if not present: pip install websockets
# But we will try to use standard library or fallback if possible, 
# strictly speaking 'websockets' is not standard.
# If user environment doesn't have it, we might fail.
# Let's check if we can use a simple handshake manually or just assume dependency exists?
# The user's repo likely has 'websockets' or similar for FastAPI.
# Let's try to verify via 'pip freeze' first? No, just try to import.

async def test_websocket():
    uri = "ws://localhost:8014/api/chat/ws"
    print(f"🔌 Connecting to {uri}...")
    
    try:
        async with websockets.connect(uri) as websocket:
            print("✅ Connected!")
            
            # Test 1: Handshake / Hello
            payload = {
                "text": json.dumps({
                    "type": "ping", 
                    "query": "",
                    "action": "ping" # Some logic skips empty query, let's see
                })
            }
            # Sending a real query to get a response
            query = "สวัสดี"
            print(f"📤 Sending query: '{query}'")
            await websocket.send(json.dumps({
                "text": json.dumps({
                    "query": query,
                    "id": "test_ws_1",
                    "ai_mode": "fast"
                })
            }))
            
            print("📥 Waiting for response...")
            try:
                response = await asyncio.wait_for(websocket.recv(), timeout=30.0)
                data = json.loads(response)
                print(f"✅ Received: {str(data)[:100]}...")
                
                if "answer" in data or "avatar_mood" in data:
                    print("✅ Response structure valid")
                else:
                    raise Exception("Invalid response structure")
                    
            except asyncio.TimeoutError:
                raise Exception("Timeout waiting for response")
                
    except Exception as e:
        print(f"❌ WebSocket Test Failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    try:
        asyncio.run(test_websocket())
        print("🎉 WebSocket Test Passed")
        sys.exit(0)
    except ImportError:
        print("⚠️ 'websockets' library not found. Skipping WebSocket test.")
        print("Run: pip install websockets")
        sys.exit(0) # Soft fail
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
