import asyncio
import logging
import sys
import os

# Add project root to path (Targeting 'Back-end' folder)
backend_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'Back-end'))
sys.path.append(backend_path)

from core.ai_models.query_interpreter import QueryInterpreter

# Configure Logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

async def main():
    print("----------------------------------------------------------------")
    print("🔍 Testing Query Interpreter (Language Analysis System)")
    print("----------------------------------------------------------------")
    
    interpreter = QueryInterpreter()
    
    test_queries = [
        "ร้านกาแฟ แถวสันติสุข มีไรบ้าง",
        "ขอที่พัก ในปัว ดีๆ",
        "วัดภูมินทร์ ไปยังไง",
        "สวัสดีครับ",
        "อยากฟังเพลง น่านเนิบๆ"
    ]
    
    for q in test_queries:
        print(f"\n📝 Input: '{q}'")
        try:
            result = await interpreter.interpret_and_route(q)
            print("✅ Output:")
            print(result)
            
            # Check for Fallback
            if result.get("location_filter") == {} and result.get("category") is None and q != "สวัสดีครับ":
                 print("⚠️  WARNING: Result looks like Fallback (No filters found)")
            else:
                 print("✨  Success: Filters/Intents detected")
                 
        except Exception as e:
            print(f"❌ Error: {e}")

    await interpreter.close()

if __name__ == "__main__":
    asyncio.run(main())
