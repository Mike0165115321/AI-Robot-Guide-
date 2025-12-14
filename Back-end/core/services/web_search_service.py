# /core/services/web_search_service.py
"""
Web Search Service: ค้นหาข้อมูลจาก Google Custom Search API
"""

import aiohttp
from typing import List, Dict, Optional
from core.config import settings


class WebSearchService:
    """Service สำหรับค้นหาข้อมูลจาก Google Custom Search API"""
    
    SEARCH_URL = "https://www.googleapis.com/customsearch/v1"
    
    def __init__(self):
        self.api_key = settings.GOOGLE_API_KEY
        self.cse_id = settings.GOOGLE_CSE_ID
        
    async def search(self, query: str, num_results: int = 5) -> List[Dict]:
        """
        ค้นหาข้อมูลจาก Google
        
        Args:
            query: คำค้นหา
            num_results: จำนวนผลลัพธ์ที่ต้องการ
            
        Returns:
            List of search results with title, snippet, link
        """
        if not self.api_key or not self.cse_id:
            print("❌ [WebSearch] ขาด GOOGLE_API_KEY หรือ GOOGLE_CSE_ID")
            return []
        
        params = {
            "key": self.api_key,
            "cx": self.cse_id,
            "q": query,
            "num": min(num_results, 10),
            "lr": "lang_th",  # Thai language
            "gl": "th"  # Thailand
        }
        
        try:
            async with aiohttp.ClientSession() as session:
                async with session.get(self.SEARCH_URL, params=params) as response:
                    if response.status != 200:
                        error_text = await response.text()
                        print(f"❌ [WebSearch] API error: {response.status} - {error_text[:200]}")
                        return []
                    
                    data = await response.json()
                    
                    results = []
                    for item in data.get("items", []):
                        results.append({
                            "title": item.get("title", ""),
                            "snippet": item.get("snippet", ""),
                            "link": item.get("link", "")
                        })
                    
                    print(f"✅ [WebSearch] พบ {len(results)} ผลลัพธ์สำหรับ: {query}")
                    return results
                    
        except Exception as e:
            print(f"❌ [WebSearch] ข้อผิดพลาด: {e}")
            return []
    
    async def search_and_summarize(self, query: str) -> str:
        """
        ค้นหาและรวมผลลัพธ์เป็น text สำหรับ AI วิเคราะห์
        """
        results = await self.search(query, num_results=5)
        
        if not results:
            return ""
        
        summary_parts = [f"ผลการค้นหา: {query}\n"]
        for i, r in enumerate(results, 1):
            summary_parts.append(f"\n--- ผลลัพธ์ {i} ---")
            summary_parts.append(f"หัวข้อ: {r['title']}")
            summary_parts.append(f"เนื้อหา: {r['snippet']}")
            summary_parts.append(f"ที่มา: {r['link']}")
        
        return "\n".join(summary_parts)


# Singleton instance
web_search_service = WebSearchService()
