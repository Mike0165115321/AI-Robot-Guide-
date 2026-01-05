
import asyncio
import logging
from core.services.news_monitor_service import news_monitor_service

# Setup logging
logging.basicConfig(level=logging.INFO)





async def test_scrape():
    # Test DuckDuckGo
    print("Fetching DDG for 'น่าน'...")
    try:
        # Note: fetch_duckduckgo now uses 'ddgs' package internally
        results = await news_monitor_service.fetch_duckduckgo("น่าน")
    except Exception as e:
        print(f"DDG Fetch Error: {e}")
        return

    import aiohttp
    from bs4 import BeautifulSoup
    
    async with aiohttp.ClientSession() as session:
        for item in results[:3]: # Test first 3
            url = item['url']
            print(f"\n--- Testing URL: {url} ---")
            
            # Use scraping logic similar to _scrape_content
            try:
                headers = {"User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0"}
                async with session.get(url, headers=headers, timeout=10) as response:
                    print(f"Status: {response.status}")
                    print(f"Final URL: {response.url}")
                    
                    if response.status == 200:
                        html = await response.text()
                        soup = BeautifulSoup(html, 'lxml')
                        
                        # Cleanup
                        for script in soup(["script", "style", "nav", "footer", "header", "aside"]):
                            script.extract()
                        
                        text = soup.get_text(separator='\n')
                        clean_text = '\n'.join([line.strip() for line in text.splitlines() if line.strip()])
                        
                        print(f"Title: {item['title']}")
                        print(f"Extracted Length: {len(clean_text)}")
                        print(f"Preview: {clean_text[:200]}...")
                        
                        if len(clean_text) > 200:
                             print("✅ SUCCESS: Found meaningful content!")
                        else:
                             print("⚠️ WARNING: Content short.")
            except Exception as e:
                print(f"Scrape Error: {e}")




if __name__ == "__main__":
    asyncio.run(test_scrape())
