"""
LINE Message Builder Utility

This module provides helper functions to build various LINE message types
including text, images, carousels, location, and flex messages.

Uses line-bot-sdk v3 compatible classes.
"""

import os
from typing import List, Dict, Any, Optional
from linebot.models import (
    TextSendMessage,
    ImageSendMessage,
    LocationSendMessage,
    TemplateSendMessage,
    ImageCarouselTemplate,
    ImageCarouselColumn,
    URIAction,
    FlexSendMessage,
    QuickReply,
    QuickReplyButton,
    MessageAction,
    PostbackAction,
)


class LineMessageBuilder:
    """Utility class to build LINE messages from RAG Orchestrator responses."""

    # Maximum messages per reply (LINE API limit)
    MAX_MESSAGES_PER_REPLY = 5

    # Base URL for static images (will be set from env or config)
    BASE_URL = os.getenv("LINE_STATIC_BASE_URL", "")

    @staticmethod
    def text_message(text: str, max_length: int = 5000) -> TextSendMessage:
        """
        Create a simple text message.
        LINE has a 5000 character limit per text message.
        """
        if len(text) > max_length:
            text = text[:max_length - 3] + "..."
        return TextSendMessage(text=text)

    @staticmethod
    def image_message(image_url: str, preview_url: str = None) -> Optional[ImageSendMessage]:
        """
        Create an image message.
        Both URLs must be HTTPS.
        """
        if not image_url:
            return None

        # Ensure HTTPS
        if image_url.startswith("http://"):
            image_url = image_url.replace("http://", "https://")

        if not preview_url:
            preview_url = image_url

        if preview_url.startswith("http://"):
            preview_url = preview_url.replace("http://", "https://")

        return ImageSendMessage(
            original_content_url=image_url,
            preview_image_url=preview_url
        )

    @staticmethod
    def image_carousel(image_urls: List[str], max_images: int = 10) -> Optional[TemplateSendMessage]:
        """
        Create an image carousel from a list of image URLs.
        LINE supports up to 10 images in a carousel.
        """
        if not image_urls:
            return None

        columns = []
        for url in image_urls[:max_images]:
            if not url:
                continue

            # Ensure HTTPS
            if url.startswith("http://"):
                url = url.replace("http://", "https://")

            columns.append(
                ImageCarouselColumn(
                    image_url=url,
                    action=URIAction(label="ดูรูปขยาย", uri=url)
                )
            )

        if not columns:
            return None

        return TemplateSendMessage(
            alt_text="รูปภาพสถานที่",
            template=ImageCarouselTemplate(columns=columns)
        )

    @staticmethod
    def location_message(
        title: str,
        address: str,
        latitude: float,
        longitude: float
    ) -> LocationSendMessage:
        """
        Create a location message with map pin.
        """
        return LocationSendMessage(
            title=title[:100],  # LINE limit
            address=address[:100] if address else title[:100],
            latitude=latitude,
            longitude=longitude
        )

    @staticmethod
    def google_maps_link(title: str, latitude: float, longitude: float) -> TextSendMessage:
        """
        Create a text message with Google Maps link.
        """
        maps_url = f"https://www.google.com/maps/search/?api=1&query={latitude},{longitude}"
        text = f"📍 แผนที่ไปยัง {title}\n{maps_url}"
        return TextSendMessage(text=text)

    @staticmethod
    def youtube_flex_card(songs: List[Dict[str, Any]]) -> FlexSendMessage:
        """
        Create a Flex Message carousel for YouTube song choices.
        
        Expected song dict format:
        {
            "title": "Song Title",
            "video_id": "dQw4w9WgXcQ",
            "thumbnail_url": "https://..."
        }
        """
        bubbles = []
        
        for song in songs[:10]:  # Max 10 bubbles
            video_id = song.get("video_id", "")
            thumbnail = song.get("thumbnail_url", f"https://img.youtube.com/vi/{video_id}/hqdefault.jpg")
            title = song.get("title", "Unknown")[:40]
            youtube_url = f"https://www.youtube.com/watch?v={video_id}"

            bubble = {
                "type": "bubble",
                "size": "micro",
                "hero": {
                    "type": "image",
                    "url": thumbnail,
                    "size": "full",
                    "aspectRatio": "16:9",
                    "aspectMode": "cover",
                    "action": {
                        "type": "uri",
                        "uri": youtube_url
                    }
                },
                "body": {
                    "type": "box",
                    "layout": "vertical",
                    "contents": [
                        {
                            "type": "text",
                            "text": title,
                            "weight": "bold",
                            "size": "sm",
                            "wrap": True,
                            "maxLines": 2
                        }
                    ],
                    "action": {
                        "type": "uri",
                        "uri": youtube_url
                    }
                }
            }
            bubbles.append(bubble)

        if not bubbles:
            return LineMessageBuilder.text_message("ไม่พบเพลงที่ค้นหา")

        carousel = {
            "type": "carousel",
            "contents": bubbles
        }

        return FlexSendMessage(
            alt_text="🎵 เพลงที่พบ",
            contents=carousel
        )

    @staticmethod
    def source_cards(sources: List[Dict[str, Any]], base_url: str = "") -> Optional[FlexSendMessage]:
        """
        Create Flex Message cards for source references.
        
        Expected source dict format:
        {
            "title": "Location Title",
            "summary": "Brief description...",
            "image_urls": ["https://..."]
        }
        """
        if not sources:
            return None

        bubbles = []
        
        for source in sources[:5]:  # Max 5 sources
            title = source.get("title", "ไม่ระบุชื่อ")[:40]
            summary = source.get("summary", "")[:100] + "..." if len(source.get("summary", "")) > 100 else source.get("summary", "")
            images = source.get("image_urls", [])
            image_url = images[0] if images else "https://via.placeholder.com/300x200?text=No+Image"

            # Ensure HTTPS
            if image_url.startswith("http://"):
                image_url = image_url.replace("http://", "https://")

            bubble = {
                "type": "bubble",
                "size": "kilo",
                "hero": {
                    "type": "image",
                    "url": image_url,
                    "size": "full",
                    "aspectRatio": "3:2",
                    "aspectMode": "cover"
                },
                "body": {
                    "type": "box",
                    "layout": "vertical",
                    "contents": [
                        {
                            "type": "text",
                            "text": title,
                            "weight": "bold",
                            "size": "md",
                            "wrap": True
                        },
                        {
                            "type": "text",
                            "text": summary,
                            "size": "xs",
                            "color": "#888888",
                            "wrap": True,
                            "margin": "md"
                        }
                    ]
                }
            }
            bubbles.append(bubble)

        if not bubbles:
            return None

        carousel = {
            "type": "carousel",
            "contents": bubbles
        }

        return FlexSendMessage(
            alt_text="📚 แหล่งข้อมูลอ้างอิง",
            contents=carousel
        )

    @staticmethod
    def default_quick_reply() -> QuickReply:
        """
        Create default Quick Reply buttons that appear after messages.
        These buttons provide quick access to common actions.
        LINE simplified: 🎵 ฟังเพลง | 🧮 เครื่องคิดเลข | ❓ ถามน้องน่าน
        """
        return QuickReply(items=[
            QuickReplyButton(
                action=MessageAction(label="🎵 ฟังเพลง", text="อยากฟังเพลง")
            ),
            QuickReplyButton(
                action=MessageAction(label="🧮 เครื่องคิดเลข", text="คำนวณ 15% ของ 850")
            ),
            QuickReplyButton(
                action=MessageAction(label="❓ ถามน้องน่าน", text="แนะนำที่เที่ยวน่าน")
            ),
        ])

    @classmethod
    def build_response_messages(
        cls,
        response: Dict[str, Any],
        base_url: str = "",
        add_quick_reply: bool = True
    ) -> List:
        """
        Build a list of LINE messages from a RAG Orchestrator response.
        
        This is the main method that handles all response types.
        Returns a list of message objects ready to send via LINE API.
        
        Response structure:
        {
            "answer": str,
            "action": str | None,
            "action_payload": dict | list | None,
            "image_gallery": list[str],
            "sources": list[dict]
        }
        """
        messages = []
        
        # 1. Handle text answer (always present)
        answer = response.get("answer", "")
        if answer:
            text_msg = cls.text_message(answer)
            # Attach Quick Reply buttons to the first message
            if add_quick_reply:
                text_msg.quick_reply = cls.default_quick_reply()
            messages.append(text_msg)

        # 2. Handle special actions
        action = response.get("action")
        action_payload = response.get("action_payload")

        if action == "SHOW_SONG_CHOICES" and action_payload:
            # YouTube song choices
            flex_msg = cls.youtube_flex_card(action_payload)
            if flex_msg:
                messages.append(flex_msg)

        # Navigation (SHOW_MAP) removed from LINE to reduce complexity
        # Users can ask navigation questions on main chat instead

        # 3. Handle image gallery - Use base_url (ngrok) for local images
        image_gallery = response.get("image_gallery", [])
        if image_gallery and base_url:
            # Convert local URLs to public URLs using base_url (ngrok)
            public_images = []
            for url in image_gallery[:10]:
                if not url:
                    continue
                if url.startswith("http://") or url.startswith("https://"):
                    # Already public URL
                    public_images.append(url.replace("http://", "https://"))
                elif url.startswith("/"):
                    # Local path, prepend base_url
                    public_images.append(f"{base_url}{url}")
            
            if len(public_images) == 1:
                img_msg = cls.image_message(public_images[0])
                if img_msg:
                    messages.append(img_msg)
            elif len(public_images) > 1:
                carousel_msg = cls.image_carousel(public_images)
                if carousel_msg:
                    messages.append(carousel_msg)

        # 4. Source cards disabled (images may not be public)
        # TODO: Enable when images are hosted on public HTTPS server

        # Limit to maximum allowed
        return messages[:cls.MAX_MESSAGES_PER_REPLY]

