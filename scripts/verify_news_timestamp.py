import asyncio
import sys
import os

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'Back-end')))

# MOCK dependencies that might handle missing env vars or packages
from unittest.mock import MagicMock
sys.modules['dotenv'] = MagicMock()
sys.modules['core.config'] = MagicMock()
sys.modules['core.config'].settings = MagicMock()
sys.modules['ddgs'] = MagicMock() # Mock ddgs if needed
sys.modules['duckduckgo_search'] = MagicMock() # Mock duckduckgo_search if needed
sys.modules['gnews'] = MagicMock() # Mock gnews if needed

# Now import the agent
from core.ai_models.news_analyzer_agent import NewsAnalyzerAgent

async def test_timestamp_preservation():
    agent = NewsAnalyzerAgent()
    
    # Mock news item with a specific past date
    mock_date = "2023-10-27T10:00:00Z"
    news_item = {
        "title": "Test News",
        "body": "This is a test news body.",
        "source": "Test Source",
        "date": mock_date,
        "url": "http://example.com"
    }
    
    # Test analyze (single)
    print("Testing single analyze...")
    # Mock LLM response to avoid API cost and ensure deterministic result
    # We cheat a bit by subclassing or mocking internal method if possible, 
    # but here we rely on the agent calling its internal methods. 
    # Since we can't easily mock the LLM call without proper mocking lib setup in this env,
    # we will try to monkeypatch _call_llm.
    
    async def mock_call_llm(prompt, system_prompt="", max_tokens=1024):
        return """
        ```json
        {
            "is_relevant": true,
            "category": "general",
            "severity_score": 1,
            "summary": "สรุปข่าวทดสอบ",
            "location_name": "น่าน",
            "valid_hours": 24,
            "action_recommendation": "info_only"
        }
        ```
        """
    
    agent._call_llm = mock_call_llm
    
    result = await agent.analyze(news_item)
    
    if result:
        print(f"Result keys: {result.keys()}")
        if "timestamp" in result:
             print(f"Timestamp found: {result['timestamp']}")
             if result['timestamp'] == mock_date:
                 print("✅ Single Analysis: Timestamp preserved correctly.")
             else:
                 print(f"❌ Single Analysis: Timestamp mismatch. Expected {mock_date}, got {result['timestamp']}")
        else:
            print("❌ Single Analysis: Timestamp MISSING in result.")
    else:
        print("❌ Single Analysis: No result returned.")

    # Test analyze_batch
    print("\nTesting batch analyze...")
    
    async def mock_call_llm_batch(prompt, system_prompt="", max_tokens=4096):
        return """
        ```json
        [
            {
                "news_index": 0,
                "is_relevant": true,
                "category": "general",
                "severity_score": 1,
                "summary": "สรุปข่าวทดสอบแบบกลุ่ม",
                "location_name": "น่าน"
            }
        ]
        ```
        """
    
    agent._call_llm = mock_call_llm_batch
    
    batch_results = await agent.analyze_batch([news_item])
    
    if batch_results and len(batch_results) > 0:
        res = batch_results[0]
        print(f"Batch Result keys: {res.keys()}")
        if "timestamp" in res:
             print(f"Timestamp found: {res['timestamp']}")
             if res['timestamp'] == mock_date:
                 print("✅ Batch Analysis: Timestamp preserved correctly.")
             else:
                 print(f"❌ Batch Analysis: Timestamp mismatch. Expected {mock_date}, got {res['timestamp']}")
        else:
             print("❌ Batch Analysis: Timestamp MISSING in result.")
    else:
        print("❌ Batch Analysis: No result returned.")

if __name__ == "__main__":
    asyncio.run(test_timestamp_preservation())
