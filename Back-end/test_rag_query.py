#!/usr/bin/env python3
"""
🧪 RAG Query Test Script
ทดสอบการค้นหาข้อมูลทั้งกรณีระบุและไม่ระบุอำเภอ
"""

import asyncio
import sys
sys.path.insert(0, '/home/mikedev/AI Robot Guide จังหวัดน่าน/Back-end')

from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from core.ai_models.query_interpreter import QueryInterpreter
from core.ai_models.rag_orchestrator import RAGOrchestrator

# Test Cases
TEST_QUERIES = [
    # กลุ่ม 1: ไม่ระบุอำเภอ (Broad Query)
    {"query": "ที่เที่ยวน่านมีอะไรบ้าง", "expect_district": None, "expect_results": True},
    {"query": "แนะนำวัดในน่านหน่อย", "expect_district": None, "expect_results": True},
    {"query": "สถานที่ท่องเที่ยวยอดนิยม", "expect_district": None, "expect_results": True},
    
    # กลุ่ม 2: ระบุอำเภอชัดเจน
    {"query": "ที่เที่ยวอำเภอปัว", "expect_district": "ปัว", "expect_results": True},
    {"query": "วัดที่ท่าวังผา", "expect_district": "ท่าวังผา", "expect_results": True},
    {"query": "ร้านอาหารเวียงสา", "expect_district": "เวียงสา", "expect_results": True},
    {"query": "บ่อเกลือมีอะไรน่าสนใจ", "expect_district": "บ่อเกลือ", "expect_results": True},
    
    # กลุ่ม 3: คำถามเฉพาะเจาะจง (Entity)
    {"query": "วัดภูมินทร์", "expect_district": None, "expect_results": True},
    {"query": "ดอยเสมอดาว", "expect_district": None, "expect_results": True},
]

async def test_rag():
    print("=" * 60)
    print("🧪 RAG Query Test - กำลังโหลดระบบ...")
    print("=" * 60)
    
    # Initialize components
    mongo = MongoDBManager()
    qdrant = QdrantManager()
    interpreter = QueryInterpreter()
    rag = RAGOrchestrator(mongo, qdrant, interpreter)
    
    print("\n✅ โหลดระบบเสร็จสิ้น\n")
    
    passed = 0
    failed = 0
    
    for i, test in enumerate(TEST_QUERIES, 1):
        query = test["query"]
        expect_district = test["expect_district"]
        
        print(f"\n{'─' * 50}")
        print(f"📝 Test #{i}: \"{query}\"")
        print(f"   Expected District: {expect_district or '(any/none)'}")
        
        try:
            result = await rag.answer_query(
                query=query,
                mode="text",
                session_id=None,
                ai_mode="fast"
            )
            
            answer = result.get("answer", "")[:100] + "..."
            sources = result.get("sources", [])
            
            # Check results
            has_results = bool(sources) or ("ไม่เจอ" not in answer and "ไม่มี" not in answer)
            
            if has_results:
                print(f"   ✅ PASS - ได้ผลลัพธ์ {len(sources)} sources")
                print(f"   📄 Answer: {answer[:80]}...")
                passed += 1
            else:
                print(f"   ❌ FAIL - ไม่มีผลลัพธ์")
                print(f"   📄 Answer: {answer}")
                failed += 1
                
        except Exception as e:
            print(f"   ❌ ERROR: {e}")
            failed += 1
    
    # Summary
    print("\n" + "=" * 60)
    print(f"📊 SUMMARY: {passed}/{len(TEST_QUERIES)} passed")
    if failed:
        print(f"❌ {failed} tests failed")
    else:
        print("🎉 All tests passed!")
    print("=" * 60)

if __name__ == "__main__":
    asyncio.run(test_rag())
