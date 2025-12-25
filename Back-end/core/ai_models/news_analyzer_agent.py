# /core/ai_models/news_analyzer_agent.py
"""
News Analyzer Agent: วิเคราะห์ข่าวด้วย LLM (JSON Mode)
"""

import asyncio
import logging
import json
from typing import Dict, Optional, List
from datetime import datetime, timezone, timedelta

from core.config import settings

logger = logging.getLogger(__name__)


class NewsAnalyzerAgent:
    """Agent สำหรับวิเคราะห์ข่าวและให้คะแนนความสำคัญ"""
    
    # Prompt สำหรับวิเคราะห์ข่าวเดี่ยว (แบบเก่า)
    ANALYSIS_PROMPT = """คุณเป็นนักวิเคราะห์ข่าวด้านการท่องเที่ยวและความปลอดภัยของจังหวัดน่าน
วิเคราะห์ข่าวต่อไปนี้แล้วตอบกลับเป็น JSON เท่านั้น (ห้ามมีข้อความอื่น):

ข่าว:
หัวข้อ: {title}
เนื้อหา: {body}
แหล่งที่มา: {source}
วันที่: {date}

ตอบเป็น JSON format ดังนี้:
{{
    "is_relevant": true/false,
    "category": "disaster|traffic|weather|event|health|general",
    "severity_score": 1-5,
    "summary": "สรุปข่าวสั้นๆ ภาษาไทย",
    "location_name": "ชื่อสถานที่ที่เกี่ยวข้อง หรือ null",
    "valid_hours": 24,
    "action_recommendation": "avoid_route|caution|monitor|info_only"
}}

เกณฑ์ severity_score:
1 = ข่าวทั่วไป/FYI
2 = ข่าวน่าสนใจ
3 = ข่าวสำคัญ ควรทราบ
4 = ข่าวเร่งด่วน ควรระวัง
5 = ข่าวฉุกเฉิน/วิกฤต

หมายเหตุ: ถ้าข่าวไม่เกี่ยวกับน่าน ให้ is_relevant = false"""

    # Prompt สำหรับวิเคราะห์ข่าวหลายรายการพร้อมกัน (Batch Mode - 5 ข่าว/ครั้ง)
    BATCH_ANALYSIS_PROMPT = """คุณเป็นนักวิเคราะห์ข่าวด้านการท่องเที่ยวและความปลอดภัยของจังหวัดน่าน
วิเคราะห์ข่าวทั้งหมดต่อไปนี้ แล้วตอบกลับเป็น JSON Array เท่านั้น (ห้ามมีข้อความอื่นนอกเหนือจาก JSON):

{news_list}

ตอบเป็น JSON Array (ตอบทุกข่าว):
[
  {{"news_index": 0, "is_relevant": true, "category": "event", "severity_score": 2, "summary": "สรุปข่าวสั้นๆ", "location_name": "อ.ปัว"}}
]

หมวดหมู่: disaster, traffic, weather, event, health, general
severity_score: 1=ทั่วไป, 2=น่าสนใจ, 3=สำคัญ, 4=เร่งด่วน, 5=วิกฤต
ถ้าข่าวไม่เกี่ยวกับน่าน ให้ is_relevant = false"""

    def __init__(self):
        self.llm_handler = None
        
    async def _get_llm_handler(self):
        """Lazy load LLM handler"""
        # ใช้ function-based approach แทน class
        return True  # Return truthy to indicate ready
    
    async def analyze(self, news_item: Dict) -> Optional[Dict]:
        """
        วิเคราะห์ข่าวหนึ่งรายการ
        
        Args:
            news_item: Dict with title, body, source, date
            
        Returns:
            Analysis result dict or None
        """
        try:
            llm = await self._get_llm_handler()
            if not llm:
                return None
                
            # สร้าง prompt
            prompt = self.ANALYSIS_PROMPT.format(
                title=news_item.get("title", ""),
                body=news_item.get("body", "")[:500],  # จำกัดความยาว
                source=news_item.get("source", ""),
                date=news_item.get("date", "")
            )
            
            # เรียก LLM
            response = await self._call_llm(prompt)
            
            # Parse JSON
            result = self._parse_json_response(response)
            
            if result and result.get("is_relevant", False):
                # เพิ่มข้อมูลต้นฉบับ
                result["original_title"] = news_item.get("title", "")
                result["original_url"] = news_item.get("url", "")
                result["original_source"] = news_item.get("source", "")
                result["analyzed_at"] = datetime.now(timezone.utc).isoformat()
                
                # คำนวณ valid_until
                valid_hours = result.get("valid_hours", 24)
                result["valid_until"] = (
                    datetime.now(timezone.utc) + timedelta(hours=valid_hours)
                ).isoformat()
                
                logger.info(f"✅ [NewsAnalyzer] วิเคราะห์สำเร็จ: {result.get('summary', '')[:50]}...")
                return result
            else:
                logger.debug(f"⏭️ [NewsAnalyzer] ข่าวไม่เกี่ยวข้อง: {news_item.get('title', '')[:30]}...")
                return None
                
        except Exception as e:
            logger.error(f"❌ [NewsAnalyzer] ข้อผิดพลาด: {e}")
            return None
    
    def _parse_json_response(self, response: str) -> Optional[Dict]:
        """Parse JSON จาก LLM response with cleanup"""
        try:
            # ลองหา JSON block
            if "```json" in response:
                start = response.find("```json") + 7
                end = response.find("```", start)
                json_str = response[start:end].strip()
            elif "```" in response:
                start = response.find("```") + 3
                end = response.find("```", start)
                json_str = response[start:end].strip()
            else:
                # ลองหา [ ] หรือ { } โดยตรง
                if "[" in response and "]" in response:
                    start = response.find("[")
                    end = response.rfind("]") + 1
                    json_str = response[start:end]
                elif "{" in response:
                    start = response.find("{")
                    end = response.rfind("}") + 1
                    if start >= 0 and end > start:
                        json_str = response[start:end]
                    else:
                        return None
                else:
                    return None
            
            # Cleanup JSON common issues
            json_str = self._cleanup_json(json_str)
            
            return json.loads(json_str)
            
        except json.JSONDecodeError as e:
            logger.error(f"❌ [NewsAnalyzer] JSON parse error: {e}")
            return None
    
    def _cleanup_json(self, json_str: str) -> str:
        """Clean up common JSON issues from LLM output"""
        import re
        
        # ลบ trailing commas ก่อน ] หรือ }
        json_str = re.sub(r',\s*([\]}])', r'\1', json_str)
        
        # ลบ newlines และ extra spaces ใน strings ถ้ามี
        json_str = json_str.replace('\n', ' ').replace('\r', '')
        
        # ลบ ... (ellipsis) ที่ LLM บางทีใส่มา
        json_str = json_str.replace('...', '')
        
        return json_str.strip()
    
    async def analyze_batch(self, news_items: List[Dict]) -> List[Dict]:
        """
        วิเคราะห์ข่าวหลายรายการพร้อมกัน (ใช้ Gemini 1 ครั้ง)
        
        Args:
            news_items: List of news items (max 10)
            
        Returns:
            List of relevant analyzed items
        """
        if not news_items:
            return []
            
        # จำกัด 5 ข่าว เพื่อลด API calls และป้องกัน rate limit
        items_to_analyze = news_items[:5]
        
        # สร้างรายการข่าวสำหรับ prompt (จำกัด body 150 ตัวอักษร)
        news_list_str = ""
        for i, item in enumerate(items_to_analyze):
            title = item.get("title", "")[:200]
            body = item.get("body", "")[:150]
            news_list_str += f"\n[ข่าว {i}]\nหัวข้อ: {title}\nเนื้อหา: {body}\n"
        
        # สร้าง prompt
        prompt = self.BATCH_ANALYSIS_PROMPT.format(news_list=news_list_str)
        
        try:
            # เรียก Gemini 1 ครั้ง
            response = await self._call_llm(prompt, max_tokens=2048)
            
            # Parse JSON Array
            parsed = self._parse_json_response(response)
            
            if not parsed or not isinstance(parsed, list):
                logger.error("❌ [NewsAnalyzer] Batch response ไม่ใช่ JSON Array")
                return []
            
            # Map results กลับไปยังข่าวต้นฉบับ
            results = []
            now = datetime.now(timezone.utc)
            
            for item in parsed:
                if not item.get("is_relevant", False):
                    continue
                    
                idx = item.get("news_index", 0)
                if idx < 0 or idx >= len(items_to_analyze):
                    continue
                    
                original = items_to_analyze[idx]
                
                result = {
                    **item,
                    "original_title": original.get("title", ""),
                    "original_url": original.get("url", ""),
                    "original_source": original.get("source", ""),
                    "analyzed_at": now.isoformat(),
                    "valid_hours": item.get("valid_hours", 24),
                    "valid_until": (now + timedelta(hours=item.get("valid_hours", 24))).isoformat(),
                    "action_recommendation": item.get("action_recommendation", "info_only")
                }
                results.append(result)
            
            logger.info(f"📊 [NewsAnalyzer] วิเคราะห์ {len(items_to_analyze)} ข่าว -> {len(results)} ข่าวที่เกี่ยวข้อง (1 Gemini call)")
            return results
            
        except Exception as e:
            logger.error(f"❌ [NewsAnalyzer] Batch analyze error: {e}")
            return []
    
    async def _call_llm(self, prompt: str, system_prompt: str = "", max_tokens: int = 1024) -> str:
        """Call LLM with prompt"""
        try:
            from core.ai_models.gemini_handler import get_gemini_response
            return await get_gemini_response(
                user_query=prompt,
                system_prompt=system_prompt or "คุณเป็น JSON generator ตอบแค่ JSON เท่านั้น ห้ามมีข้อความอื่น",
                max_tokens=max_tokens
            )
        except Exception as e:
            logger.error(f"❌ [NewsAnalyzer] LLM call failed: {e}")
            return ""
    
    async def analyze_weather(self, weather_data: Dict) -> Optional[Dict]:
        """วิเคราะห์สภาพอากาศและสร้าง alert ถ้าจำเป็น"""
        if not weather_data:
            return None
            
        temp = weather_data.get("temperature", 25)
        wind_speed = weather_data.get("wind_speed", 0)
        description = weather_data.get("description", "")
        
        severity = 1
        summary = ""
        
        # วิเคราะห์อุณหภูมิ
        if temp > 40:
            severity = max(severity, 4)
            summary = f"🌡️ อุณหภูมิสูงมาก {temp}°C - ระวังโรคลมแดด"
        elif temp > 38:
            severity = max(severity, 3)
            summary = f"🌡️ อุณหภูมิสูง {temp}°C"
        
        # วิเคราะห์ลม
        if wind_speed > 20:
            severity = max(severity, 5)
            summary = f"💨 ลมแรงมาก {wind_speed} m/s - อันตราย"
        elif wind_speed > 15:
            severity = max(severity, 4)
            summary = f"💨 ลมแรง {wind_speed} m/s - ระวังต้นไม้ล้ม"
        
        if severity >= 3:
            return {
                "is_relevant": True,
                "category": "weather",
                "severity_score": severity,
                "summary": summary or f"สภาพอากาศ: {description}",
                "location_name": "จังหวัดน่าน",
                "valid_hours": 6,
                "valid_until": (datetime.now(timezone.utc) + timedelta(hours=6)).isoformat(),
                "action_recommendation": "caution" if severity < 5 else "avoid_route",
                "original_source": weather_data.get("source", "weather_api"),
                "analyzed_at": datetime.now(timezone.utc).isoformat()
            }
        
        return None
    
    async def analyze_air_quality(self, pm25_data: Dict) -> Optional[Dict]:
        """วิเคราะห์คุณภาพอากาศและสร้าง alert ถ้าจำเป็น"""
        if not pm25_data:
            return None
            
        severity = pm25_data.get("severity", 1)
        pm25 = pm25_data.get("pm25", 0)
        level = pm25_data.get("aqi_level_th", "ปกติ")
        
        if severity >= 3:
            action = "monitor" if severity < 4 else "caution"
            summary = f"🌫️ PM2.5: {pm25} µg/m³ ({level})"
            
            if severity >= 4:
                summary += " - แนะนำสวมหน้ากาก N95"
            
            return {
                "is_relevant": True,
                "category": "health",
                "severity_score": severity,
                "summary": summary,
                "location_name": pm25_data.get("station_name", "จังหวัดน่าน"),
                "valid_hours": 3,
                "valid_until": (datetime.now(timezone.utc) + timedelta(hours=3)).isoformat(),
                "action_recommendation": action,
                "original_source": "openaq",
                "analyzed_at": datetime.now(timezone.utc).isoformat()
            }
        
        return None


# Singleton instance
news_analyzer_agent = NewsAnalyzerAgent()
