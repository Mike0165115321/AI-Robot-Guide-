# /core/services/news_monitor_service.py
"""
News Monitor Service: ดึงข่าวจาก GNews และ DuckDuckGo
"""

import asyncio
import logging
from typing import List, Dict, Optional
from datetime import datetime, timezone
import aiohttp

logger = logging.getLogger(__name__)


class NewsMonitorService:
    """Service สำหรับดึงข่าวเกี่ยวกับจังหวัดน่าน"""
    
    # คีย์เวิร์ดสำหรับค้นหาข่าว
    KEYWORDS = [
        "น่าน",
        "น้ำป่า น่าน",
        "ไฟป่า น่าน", 
        "ถนนปิด น่าน",
        "ดินถล่ม น่าน",
        "พายุ น่าน",
        "อุบัติเหตุ น่าน"
    ]
    
    def __init__(self):
        self.gnews_enabled = True
        self.ddg_enabled = True
        
    async def fetch_duckduckgo(self, keyword: str, max_results: int = 5) -> List[Dict]:
        """
        ดึงข่าวจาก DuckDuckGo Search
        
        Args:
            keyword: คำค้นหา
            max_results: จำนวนผลลัพธ์สูงสุด
            
        Returns:
            List of news items
        """
        try:
            from ddgs import DDGS
            
            results = []
            with DDGS() as ddgs:
                # ค้นหาข่าว (news)
                news_results = list(ddgs.news(
                    keyword,
                    region="th-th",
                    max_results=max_results
                ))
                
                for item in news_results:
                    results.append({
                        "source": "duckduckgo",
                        "title": item.get("title", ""),
                        "body": item.get("body", ""),
                        "url": item.get("url", ""),
                        "date": item.get("date", ""),
                        "image": item.get("image", ""),
                        "keyword": keyword,
                        "fetched_at": datetime.now(timezone.utc).isoformat()
                    })
                    
            logger.info(f"✅ [DDG] พบ {len(results)} ข่าวสำหรับ: {keyword}")
            return results
            
        except ImportError:
            logger.error("❌ [DDG] กรุณาติดตั้ง: pip install duckduckgo-search")
            return []
        except Exception as e:
            logger.error(f"❌ [DDG] ข้อผิดพลาด: {e}")
            return []
    
    async def fetch_gnews(self, keyword: str, max_results: int = 5) -> List[Dict]:
        """
        ดึงข่าวจาก GNews
        
        Args:
            keyword: คำค้นหา
            max_results: จำนวนผลลัพธ์สูงสุด
            
        Returns:
            List of news items
        """
        try:
            from gnews import GNews
            
            google_news = GNews(
                language='th',
                country='TH',
                max_results=max_results
            )
            
            news_results = google_news.get_news(keyword)
            results = []
            
            for item in news_results or []:
                results.append({
                    "source": "gnews",
                    "title": item.get("title", ""),
                    "body": item.get("description", ""),
                    "url": item.get("url", ""),
                    "date": item.get("published date", ""),
                    "publisher": item.get("publisher", {}).get("title", ""),
                    "keyword": keyword,
                    "fetched_at": datetime.now(timezone.utc).isoformat()
                })
                
            logger.info(f"✅ [GNews] พบ {len(results)} ข่าวสำหรับ: {keyword}")
            return results
            
        except ImportError:
            logger.error("❌ [GNews] กรุณาติดตั้ง: pip install gnews")
            return []
        except Exception as e:
            logger.error(f"❌ [GNews] ข้อผิดพลาด: {e}")
            return []
    
    async def aggregate_news(self, keywords: List[str] = None) -> List[Dict]:
        """
        รวมข่าวจากทุกแหล่ง
        
        Args:
            keywords: รายการคีย์เวิร์ด (ถ้าไม่ระบุใช้ค่าเริ่มต้น)
            
        Returns:
            List of all news items (deduplicated by URL)
        """
        if keywords is None:
            keywords = self.KEYWORDS
            
        all_news = []
        seen_urls = set()
        
        for keyword in keywords:
            # ดึงจาก DuckDuckGo
            if self.ddg_enabled:
                ddg_results = await self.fetch_duckduckgo(keyword, max_results=3)
                for item in ddg_results:
                    if item["url"] not in seen_urls:
                        all_news.append(item)
                        seen_urls.add(item["url"])
            
            # ดึงจาก GNews
            if self.gnews_enabled:
                gnews_results = await self.fetch_gnews(keyword, max_results=3)
                for item in gnews_results:
                    if item["url"] not in seen_urls:
                        all_news.append(item)
                        seen_urls.add(item["url"])
            
            # หน่วงเวลาเล็กน้อยเพื่อไม่ให้โดน rate limit
            await asyncio.sleep(0.5)
        
        logger.info(f"📰 [NewsMonitor] รวมข่าวทั้งหมด: {len(all_news)} รายการ")
        return all_news


# Singleton instance
news_monitor_service = NewsMonitorService()
