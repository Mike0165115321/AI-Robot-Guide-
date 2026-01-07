# /core/services/news_monitor_service.py
"""
News Monitor Service: ดึงข่าวจาก GNews และ DuckDuckGo
"""

import asyncio
import logging
from typing import List, Dict, Optional
from datetime import datetime, timezone

import aiohttp
import nltk

try:
    nltk.data.find('tokenizers/punkt')
    nltk.data.find('tokenizers/punkt_tab')
except LookupError:
    nltk.download('punkt', quiet=True)
    nltk.download('punkt_tab', quiet=True)

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
        
    async def _scrape_content(self, url: str) -> str:
        """
        Scrape full content from URL (Simple version)
        """
        if not url:
            return ""
            
        try:
            async with aiohttp.ClientSession() as session:
                async with session.get(url, timeout=10, headers={"User-Agent": "Mozilla/5.0"}) as response:
                    if response.status != 200:
                        return ""
                    html = await response.text()
            
            from bs4 import BeautifulSoup
            soup = BeautifulSoup(html, 'lxml')
            
            # Remove scripts and styles
            for script in soup(["script", "style", "nav", "footer", "header", "aside"]):
                script.extract()
                
            # Get text
            text = soup.get_text(separator='\n')
            
            # Clean text
            lines = (line.strip() for line in text.splitlines())
            chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
            text = '\n'.join(chunk for chunk in chunks if chunk)
            
            return text[:2000] # Limit to 2000 chars
            
        except Exception:
            return ""

    async def fetch_duckduckgo(self, keyword: str, max_results: int = 5) -> List[Dict]:
        """
        ดึงข่าวจาก DuckDuckGo Search
        """
        try:
            # Update to use new package name if available
            try:
                from ddgs import DDGS
            except ImportError:
                from duckduckgo_search import DDGS
            
            results = []
            # DDGS might raise Ratelimit exceptions
            try:
                # เพิ่ม timeout เป็น 30 วินาที
                with DDGS(timeout=30) as ddgs:
                    news_results = list(ddgs.news(
                        keyword,
                        region="th-th",
                        max_results=max_results
                    ))
                    
                    for item in news_results:
                        url = item.get("url", "")
                        body = item.get("body", "")
                        
                        # Try to scrape content (DDG links are usually direct)
                        # Only scrape if body is short
                        if len(body) < 200:
                            try:
                                full_content = await self._scrape_content(url)
                                if len(full_content) > len(body):
                                    body = full_content
                            except Exception as e:
                                logger.warning(f"⚠️ [DDG] Scrape failed for {url}: {e}")
                                
                        results.append({
                            "source": "duckduckgo",
                            "title": item.get("title", ""),
                            "body": body,
                            "url": url,
                            "date": item.get("date", ""),
                            "image": item.get("image", ""),
                            "keyword": keyword,
                            "fetched_at": datetime.now(timezone.utc).isoformat()
                        })
                        
                logger.info(f"✅ [DDG] พบ {len(results)} ข่าวสำหรับ: {keyword}")
                return results
                
            except Exception as e:
                # Handle Ratelimit strictly
                if "Ratelimit" in str(e):
                    logger.warning(f"⚠️ [DDG] Rate Limit Hit for {keyword}. Skipping.")
                    return []
                raise e

        except ImportError:
            logger.error("❌ [DDG] กรุณาติดตั้ง: pip install ddgs")
            return []
        except Exception as e:
            error_msg = str(e).lower()
            if "time" in error_msg and "out" in error_msg:
                 logger.warning(f"⚠️ [DDG] Connection Timed Out: {e}")
            else:
                 logger.error(f"❌ [DDG] ข้อผิดพลาด: {e}")
            return []
    
    async def fetch_gnews(self, keyword: str, max_results: int = 5) -> List[Dict]:
        """
        ดึงข่าวจาก GNews
        """
        try:
            from gnews import GNews
            
            google_news = GNews(
                language='th',
                country='TH',
                max_results=max_results
            )
            
            # Run blocking call in executor
            loop = asyncio.get_event_loop()
            news_results = await loop.run_in_executor(None, lambda: google_news.get_news(keyword))
            
            results = []
            
            for item in news_results or []:
                url = item.get("url", "")
                body = item.get("description", "")
                
                # Note: GNews URLs are Google Redirects which are hard to scrape without Selenium.
                # We skip deep scraping for GNews to avoid errors/empty content.
                # We rely on DuckDuckGo for deep content.

                results.append({
                    "source": "gnews",
                    "title": item.get("title", ""),
                    "body": body,
                    "url": url,
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
