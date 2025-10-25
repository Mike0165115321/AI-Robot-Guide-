# /core/tools/google_search.py

import logging
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError
from typing import List, Optional

# (!!) Important: Get these from your settings/environment variables
from core.config import settings

class GoogleSearchResult:
    """Simple class to hold search result data."""
    def __init__(self, title: str, link: str, snippet: str, image_url: Optional[str] = None):
        self.title = title
        self.link = link
        self.snippet = snippet
        self.image_url = image_url

class GoogleSearchTool:
    def __init__(self):
        self.api_key = settings.GOOGLE_API_KEY
        self.cse_id = settings.GOOGLE_CSE_ID
        
        if not self.api_key or not self.cse_id:
            logging.error("🚨 CRITICAL: Google API Key or CSE ID is missing in settings.")
            self.service = None
        else:
            try:
                # Build the service object
                self.service = build("customsearch", "v1", developerKey=self.api_key)
                logging.info("🛠️ Google Search Tool initialized.")
            except Exception as e:
                logging.error(f"❌ Failed to initialize Google Search service: {e}", exc_info=True)
                self.service = None

    def _execute_search(self, query: str, search_type: str, num_results: int) -> Optional[dict]:
        """Internal helper to execute the search query."""
        if not self.service:
            logging.error("Google Search service is not initialized.")
            return None
        try:
            result = self.service.cse().list(
                q=query,
                cx=self.cse_id,
                searchType=search_type, # 'image' or None for web
                num=num_results,
                safe='high' # Optional: filter explicit content
            ).execute()
            return result
        except HttpError as e:
            logging.error(f"❌ Google Search API HttpError: {e.content}", exc_info=True)
            return None
        except Exception as e:
            logging.error(f"❌ An unexpected error occurred during Google Search: {e}", exc_info=True)
            return None

    def search_web(self, query: str, max_results: int = 3) -> List[GoogleSearchResult]:
        """Performs a web search."""
        logging.info(f"🌐 [GoogleSearch] Searching web for: '{query}'")
        raw_results = self._execute_search(query, search_type=None, num_results=max_results)
        
        if not raw_results or 'items' not in raw_results:
            logging.warning("[GoogleSearch] No web results found or API error.")
            return []

        search_results = []
        for item in raw_results['items']:
            search_results.append(GoogleSearchResult(
                title=item.get('title'),
                link=item.get('link'),
                snippet=item.get('snippet')
            ))
        logging.info(f"✅ [GoogleSearch] Found {len(search_results)} web results.")
        return search_results

    def search_images(self, query: str, max_results: int = 5) -> List[GoogleSearchResult]:
        """Performs an image search."""
        logging.info(f"🖼️ [GoogleSearch] Searching images for: '{query}'")
        # Google Image Search API returns max 10 results per request
        num_to_request = min(max_results, 10) 
        
        raw_results = self._execute_search(query, search_type='image', num_results=num_to_request)

        if not raw_results or 'items' not in raw_results:
            logging.warning("[GoogleSearch] No image results found or API error.")
            return []

        search_results = []
        for item in raw_results['items']:
            # For images, the main URL is often in 'link', 
            # and a direct image URL might be in 'pagemap' -> 'cse_image' -> 'src'
            # or directly in the item's 'link' if it's a direct image link
            image_url = item.get('link') 
            # Try to find a better thumbnail/source if available
            if item.get('pagemap') and item['pagemap'].get('cse_image'):
                 image_url = item['pagemap']['cse_image'][0].get('src', image_url)
                 
            search_results.append(GoogleSearchResult(
                title=item.get('title'),
                link=item.get('image', {}).get('contextLink'), # Link to the page containing the image
                snippet=item.get('snippet'),
                image_url=image_url # The direct URL to the image
            ))
            
        logging.info(f"✅ [GoogleSearch] Found {len(search_results)} image results.")
        # Return only the number requested, even if API gave more
        return search_results[:max_results] 

# Create a singleton instance for other modules to import
google_search_instance = GoogleSearchTool()