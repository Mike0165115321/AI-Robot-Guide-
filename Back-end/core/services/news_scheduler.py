# /core/services/news_scheduler.py
"""
News Scheduler: Background job ที่รันทุก 10 นาที
"""

import asyncio
import logging
from typing import Optional, Callable, List, Dict
from datetime import datetime, timezone

logger = logging.getLogger(__name__)


class NewsScheduler:
    """
    Scheduler สำหรับดึงข่าวและวิเคราะห์เป็นระยะ
    ใช้ asyncio task แทน APScheduler เพื่อความเบาและเข้ากับ FastAPI
    """
    
    def __init__(self, interval_minutes: int = 10):
        self.interval_minutes = interval_minutes
        self.running = False
        self._task: Optional[asyncio.Task] = None
        self._alert_callback: Optional[Callable] = None
        
        # Services (lazy loaded)
        self._news_service = None
        self._weather_service = None
        self._air_quality_service = None
        self._news_analyzer = None
        self._geocoding_service = None
        
    async def _load_services(self):
        """Lazy load services"""
        if self._news_service is None:
            from core.services.news_monitor_service import news_monitor_service
            from core.services.weather_service import weather_service
            from core.services.air_quality_service import air_quality_service
            from core.services.geocoding_service import geocoding_service
            from core.ai_models.news_analyzer_agent import news_analyzer_agent
            
            self._news_service = news_monitor_service
            self._weather_service = weather_service
            self._air_quality_service = air_quality_service
            self._news_analyzer = news_analyzer_agent
            self._geocoding_service = geocoding_service
    
    def set_alert_callback(self, callback: Callable):
        """ตั้ง callback สำหรับส่ง alert ไป WebSocket"""
        self._alert_callback = callback
        
    def start(self):
        """เริ่ม scheduler"""
        if self.running:
            logger.warning("⚠️ [NewsScheduler] กำลังทำงานอยู่แล้ว")
            return
            
        self.running = True
        self._task = asyncio.create_task(self._run_loop())
        logger.info(f"✅ [NewsScheduler] เริ่มทำงาน - polling ทุก {self.interval_minutes} นาที")
        
    def stop(self):
        """หยุด scheduler"""
        self.running = False
        if self._task:
            self._task.cancel()
            self._task = None
        logger.info("🛑 [NewsScheduler] หยุดทำงาน")
    
    async def _run_loop(self):
        """Main loop"""
        # รอ 30 วินาทีหลังเริ่ม server ก่อนทำงานครั้งแรก
        await asyncio.sleep(30)
        
        while self.running:
            try:
                await self.poll_and_analyze()
            except Exception as e:
                logger.error(f"❌ [NewsScheduler] Error in poll loop: {e}")
            
            # รอ interval ก่อนรอบถัดไป
            await asyncio.sleep(self.interval_minutes * 60)
    
    async def poll_and_analyze(self) -> List[Dict]:
        """
        ดึงข่าว วิเคราะห์ และส่ง alert
        Returns: List of alerts generated
        """
        await self._load_services()
        
        all_alerts = []
        
        logger.info("📰 [NewsScheduler] เริ่มดึงข้อมูล...")
        
        try:
            # 1. ดึงข่าว
            news_items = await self._news_service.aggregate_news()
            logger.info(f"📰 พบข่าว {len(news_items)} รายการ")
            
            # 2. วิเคราะห์ข่าว
            if news_items:
                analyzed = await self._news_analyzer.analyze_batch(news_items[:10])  # จำกัด 10 ข่าว/รอบ
                for item in analyzed:
                    # Geocode สถานที่
                    if item.get("location_name"):
                        geo = await self._geocoding_service.geocode(item["location_name"])
                        if geo:
                            item["lat"] = geo["lat"]
                            item["lon"] = geo["lon"]
                    
                    all_alerts.append(item)
            
            # 3. ดึงและวิเคราะห์สภาพอากาศ
            weather = await self._weather_service.get_current_weather()
            if weather:
                weather_alert = await self._news_analyzer.analyze_weather(weather)
                if weather_alert:
                    all_alerts.append(weather_alert)
            
            # 4. ดึงและวิเคราะห์ PM2.5 (WAQI API)
            pm25 = await self._air_quality_service.get_pm25()
            if pm25:
                pm25_alert = await self._news_analyzer.analyze_air_quality(pm25)
                if pm25_alert:
                    all_alerts.append(pm25_alert)
            
            # 5. ส่ง alerts ที่ severity >= 4 ไป WebSocket
            high_priority_alerts = [a for a in all_alerts if a.get("severity_score", 0) >= 4]
            
            if high_priority_alerts and self._alert_callback:
                for alert in high_priority_alerts:
                    try:
                        await self._alert_callback(alert)
                        logger.info(f"🚨 [NewsScheduler] ส่ง alert: {alert.get('summary', '')[:50]}...")
                    except Exception as e:
                        logger.error(f"❌ [NewsScheduler] ส่ง alert ล้มเหลว: {e}")
            
            # 6. บันทึก alerts ทั้งหมดลง storage (สำหรับ UI)
            await self._store_alerts(all_alerts)
            
            logger.info(f"✅ [NewsScheduler] เสร็จสิ้น: {len(all_alerts)} alerts, {len(high_priority_alerts)} high priority")
            return all_alerts
            
        except Exception as e:
            logger.error(f"❌ [NewsScheduler] poll_and_analyze error: {e}")
            return []
    
    async def _store_alerts(self, alerts: List[Dict]):
        """บันทึก alerts ลง MongoDB"""
        if not alerts:
            return
            
        try:
            from core.services.alert_storage_service import alert_storage_service
            saved = await alert_storage_service.save_alerts_batch(alerts)
            logger.info(f"💾 [NewsScheduler] บันทึก {saved}/{len(alerts)} alerts ลง MongoDB")
        except Exception as e:
            logger.error(f"❌ [NewsScheduler] บันทึก alerts ล้มเหลว: {e}")
    
    async def manual_poll(self) -> List[Dict]:
        """สำหรับเรียกใช้ manual (API endpoint)"""
        return await self.poll_and_analyze()


# Singleton instance
news_scheduler = NewsScheduler()
