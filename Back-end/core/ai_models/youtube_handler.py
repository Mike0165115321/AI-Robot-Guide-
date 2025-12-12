# /core/ai_models/youtube_handler.py (Final Upgraded Version)

import asyncio
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError
from core.config import settings
import yt_dlp 

class YouTubeHandler:
    def __init__(self):
        self.api_key = settings.YOUTUBE_API_KEY
        if not self.api_key:
            print("⚠️ [YouTube] API Key not found. YouTube feature will be disabled.")
            self.youtube_service = None
        else:
            try:
                self.youtube_service = build('youtube', 'v3', developerKey=self.api_key)
                print("✅ [YouTube] Service initialized successfully.")
            except Exception as e:
                print(f"❌ [YouTube] Failed to initialize service: {e}")
                self.youtube_service = None
        
        self.ydl_opts = {
            'format': 'bestaudio/best', 'noplaylist': True, 'quiet': True,
            'default_search': 'auto', 'source_address': '0.0.0.0'
        }

    async def search_music(self, query: str, max_results: int = 7) -> list[dict] | None:
        if not self.youtube_service: return None

        try:
            search_query = f"{query} เพลง" if "เพลง" not in query else query
            print(f"🎬 [YouTube] Searching for: '{search_query}'")
            
            def _search_videos():
                request = self.youtube_service.search().list(
                    part="snippet", q=search_query, type="video",
                    videoCategoryId="10", maxResults=max_results, relevanceLanguage="th"
                )
                return request.execute()

            search_response = await asyncio.to_thread(_search_videos)
            video_ids = [item["id"]["videoId"] for item in search_response.get("items", [])]
            if not video_ids: return []

            # [NEW] Check embeddable status
            def _check_status():
                video_request = self.youtube_service.videos().list(
                    part="snippet,status", id=",".join(video_ids)
                )
                return video_request.execute()

            video_response = await asyncio.to_thread(_check_status)
            
            embeddable_videos = []
            for item in video_response.get("items", []):
                if item.get("status", {}).get("embeddable"):
                    # ดึง thumbnail URL - ใช้ high quality ถ้ามี
                    thumbnails = item["snippet"].get("thumbnails", {})
                    thumbnail_url = (
                        thumbnails.get("high", {}).get("url") or
                        thumbnails.get("medium", {}).get("url") or
                        thumbnails.get("default", {}).get("url") or
                        f"https://img.youtube.com/vi/{item['id']}/hqdefault.jpg"
                    )
                    
                    embeddable_videos.append({
                        "video_id": item["id"],
                        "title": item["snippet"]["title"],
                        "channel": item["snippet"]["channelTitle"],
                        "url": f"https://www.youtube.com/watch?v={item['id']}",
                        "thumbnail": thumbnail_url  # 🆕 เพิ่ม thumbnail
                    })
            
            print(f"✅ [YouTube] Found {len(embeddable_videos)} embeddable videos.")
            return embeddable_videos[:5] # คืนค่าสูงสุด 5 รายการ

        except HttpError as e:
            print(f"❌ [YouTube] An HTTP error occurred: {e}")
            return None
        except Exception as e:
            print(f"❌ [YouTube] An unexpected error occurred: {e}")
            return None

    async def get_audio_stream_url(self, video_url: str) -> str | None:
        """
        ใช้ yt-dlp เพื่อดึง URL ของ audio stream โดยตรงจาก YouTube URL
        """
        loop = asyncio.get_event_loop()
        try:
            def blocking_ydl_call():
                with yt_dlp.YoutubeDL(self.ydl_opts) as ydl:
                    info = ydl.extract_info(video_url, download=False)
                    return info['url']

            audio_url = await loop.run_in_executor(None, blocking_ydl_call)
            print(f"🎧 [yt-dlp] Extracted audio stream URL.")
            return audio_url
        except Exception as e:
            print(f"❌ [yt-dlp] Failed to extract audio stream: {e}")
            return None

youtube_handler_instance = YouTubeHandler()
