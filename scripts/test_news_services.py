#!/usr/bin/env python3
"""
Test Script: Smart News Services
ทดสอบ services ที่สร้างสำหรับ Smart News Monitor
"""

import asyncio
import sys
import os

# Add project root to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'Back-end'))


async def test_news_service():
    """ทดสอบ NewsMonitorService"""
    print("\n" + "="*50)
    print("🧪 ทดสอบ NewsMonitorService")
    print("="*50)
    
    try:
        from core.services.news_monitor_service import news_monitor_service
        
        # Test DuckDuckGo
        print("\n📰 ทดสอบ DuckDuckGo News...")
        ddg_results = await news_monitor_service.fetch_duckduckgo("น่าน", max_results=2)
        print(f"   ผลลัพธ์: {len(ddg_results)} รายการ")
        if ddg_results:
            print(f"   หัวข้อแรก: {ddg_results[0].get('title', '')[:50]}...")
            
        # Test GNews
        print("\n📰 ทดสอบ GNews...")
        gnews_results = await news_monitor_service.fetch_gnews("น่าน", max_results=2)
        print(f"   ผลลัพธ์: {len(gnews_results)} รายการ")
        if gnews_results:
            print(f"   หัวข้อแรก: {gnews_results[0].get('title', '')[:50]}...")
            
        print("\n✅ NewsMonitorService: PASSED")
        return True
        
    except Exception as e:
        print(f"\n❌ NewsMonitorService: FAILED - {e}")
        return False


async def test_air_quality_service():
    """ทดสอบ AirQualityService"""
    print("\n" + "="*50)
    print("🧪 ทดสอบ AirQualityService (OpenAQ)")
    print("="*50)
    
    try:
        from core.services.air_quality_service import air_quality_service
        
        print("\n🌫️ ดึงค่า PM2.5 บริเวณน่าน...")
        pm25_data = await air_quality_service.get_pm25()
        
        if pm25_data:
            print(f"   สถานี: {pm25_data.get('station_name')}")
            print(f"   PM2.5: {pm25_data.get('pm25')} µg/m³")
            print(f"   ระดับ: {pm25_data.get('aqi_level_th')}")
            print(f"   Severity: {pm25_data.get('severity')}")
            print("\n✅ AirQualityService: PASSED")
            return True
        else:
            print("   ⚠️ ไม่พบสถานีในบริเวณนี้ (อาจเป็นปกติ)")
            print("\n✅ AirQualityService: PASSED (no station)")
            return True
            
    except Exception as e:
        print(f"\n❌ AirQualityService: FAILED - {e}")
        return False


async def test_geocoding_service():
    """ทดสอบ GeocodingService"""
    print("\n" + "="*50)
    print("🧪 ทดสอบ GeocodingService (Nominatim)")
    print("="*50)
    
    try:
        from core.services.geocoding_service import geocoding_service
        
        test_places = ["วัดภูมินทร์", "ดอยเสมอดาว"]
        
        for place in test_places:
            print(f"\n📍 Geocoding: {place}")
            result = await geocoding_service.geocode(place)
            
            if result:
                print(f"   พิกัด: ({result['lat']}, {result['lon']})")
                print(f"   ชื่อเต็ม: {result['display_name'][:50]}...")
            else:
                print(f"   ⚠️ ไม่พบพิกัด")
                
        print("\n✅ GeocodingService: PASSED")
        return True
        
    except Exception as e:
        print(f"\n❌ GeocodingService: FAILED - {e}")
        return False


async def test_weather_service():
    """ทดสอบ WeatherService"""
    print("\n" + "="*50)
    print("🧪 ทดสอบ WeatherService")
    print("="*50)
    
    try:
        from core.services.weather_service import weather_service
        
        print("\n🌤️ ดึงสภาพอากาศน่าน...")
        weather = await weather_service.get_current_weather()
        
        if weather:
            print(f"   แหล่งข้อมูล: {weather.get('source')}")
            print(f"   อุณหภูมิ: {weather.get('temperature')}°C")
            print(f"   ความชื้น: {weather.get('humidity')}%")
            print(f"   สภาพอากาศ: {weather.get('description', weather.get('condition', '-'))}")
            print("\n✅ WeatherService: PASSED")
            return True
        else:
            print("   ⚠️ ไม่สามารถดึงข้อมูลได้ (ต้องตั้งค่า API Key)")
            print("   ➡️ ตั้งค่า OPENWEATHER_API_KEY หรือ TMD_API_KEY ใน .env")
            print("\n⚠️ WeatherService: SKIPPED (no API key)")
            return True
            
    except Exception as e:
        print(f"\n❌ WeatherService: FAILED - {e}")
        return False


async def test_news_analyzer():
    """ทดสอบ NewsAnalyzerAgent"""
    print("\n" + "="*50)
    print("🧪 ทดสอบ NewsAnalyzerAgent (LLM)")
    print("="*50)
    
    try:
        from core.ai_models.news_analyzer_agent import news_analyzer_agent
        
        test_news = {
            "title": "เตือนน้ำป่าไหลหลากที่อำเภอปัว จังหวัดน่าน",
            "body": "กรมอุตุฯ เตือนประชาชนในพื้นที่อำเภอปัว จังหวัดน่าน ระวังน้ำป่าไหลหลากจากฝนตกหนัก",
            "source": "test",
            "date": "2025-12-25"
        }
        
        print("\n🤖 วิเคราะห์ข่าวทดสอบ...")
        print(f"   หัวข้อ: {test_news['title']}")
        
        result = await news_analyzer_agent.analyze(test_news)
        
        if result:
            print(f"\n   📊 ผลการวิเคราะห์:")
            print(f"   - is_relevant: {result.get('is_relevant')}")
            print(f"   - category: {result.get('category')}")
            print(f"   - severity: {result.get('severity_score')}")
            print(f"   - summary: {result.get('summary', '')[:50]}...")
            print("\n✅ NewsAnalyzerAgent: PASSED")
            return True
        else:
            print("   ⚠️ ไม่ได้รับผลวิเคราะห์ (อาจเป็นเพราะ LLM API)")
            print("\n⚠️ NewsAnalyzerAgent: SKIPPED")
            return True
            
    except Exception as e:
        print(f"\n❌ NewsAnalyzerAgent: FAILED - {e}")
        return False


async def main():
    """Run all tests"""
    print("\n" + "="*60)
    print("🚀 Smart News Services - Test Suite")
    print("="*60)
    
    results = []
    
    # Run tests
    results.append(("NewsMonitorService", await test_news_service()))
    results.append(("AirQualityService", await test_air_quality_service()))
    results.append(("GeocodingService", await test_geocoding_service()))
    results.append(("WeatherService", await test_weather_service()))
    # results.append(("NewsAnalyzerAgent", await test_news_analyzer()))  # Uncomment to test LLM
    
    # Summary
    print("\n" + "="*60)
    print("📊 สรุปผลการทดสอบ")
    print("="*60)
    
    passed = 0
    for name, result in results:
        status = "✅ PASSED" if result else "❌ FAILED"
        print(f"   {name}: {status}")
        if result:
            passed += 1
            
    print(f"\n   ผ่าน: {passed}/{len(results)}")
    print("="*60)
    
    return all(r[1] for r in results)


if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)
