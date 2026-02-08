#!/bin/bash
# 🧪 RAG Query API Test Script
# ใช้ curl เรียก API โดยตรงเพื่อทดสอบ

API_URL="http://localhost:8014/api/chat/"

echo "=============================================="
echo "🧪 RAG Query API Test"
echo "=============================================="

# Test 1: ไม่ระบุอำเภอ
echo ""
echo "📝 Test 1: ที่เที่ยวน่านมีอะไรบ้าง (Broad Query)"
curl -s -X POST "$API_URL" \
  -H "Content-Type: application/json" \
  -d '{"query": "ที่เที่ยวน่านมีอะไรบ้าง", "mode": "text", "ai_mode": "fast"}' | \
  python3 -c "import sys,json; d=json.load(sys.stdin); print('✅ Answer:', d.get('answer','')[:150]+'...' if d.get('answer') else '❌ No answer')"

echo ""
echo "──────────────────────────────────────────"

# Test 2: ระบุอำเภอปัว
echo ""
echo "📝 Test 2: ที่เที่ยวอำเภอปัว (With District)"
curl -s -X POST "$API_URL" \
  -H "Content-Type: application/json" \
  -d '{"query": "ที่เที่ยวอำเภอปัว", "mode": "text", "ai_mode": "fast"}' | \
  python3 -c "import sys,json; d=json.load(sys.stdin); print('✅ Answer:', d.get('answer','')[:150]+'...' if d.get('answer') else '❌ No answer')"

echo ""
echo "──────────────────────────────────────────"

# Test 3: คำถามเฉพาะ
echo ""
echo "📝 Test 3: วัดภูมินทร์ (Specific Entity)"
curl -s -X POST "$API_URL" \
  -H "Content-Type: application/json" \
  -d '{"query": "วัดภูมินทร์", "mode": "text", "ai_mode": "fast"}' | \
  python3 -c "import sys,json; d=json.load(sys.stdin); print('✅ Answer:', d.get('answer','')[:150]+'...' if d.get('answer') else '❌ No answer')"

echo ""
echo "──────────────────────────────────────────"

# Test 4: อำเภอบ่อเกลือ
echo ""
echo "📝 Test 4: บ่อเกลือมีอะไรน่าสนใจ (District: บ่อเกลือ)"
curl -s -X POST "$API_URL" \
  -H "Content-Type: application/json" \
  -d '{"query": "บ่อเกลือมีอะไรน่าสนใจ", "mode": "text", "ai_mode": "fast"}' | \
  python3 -c "import sys,json; d=json.load(sys.stdin); print('✅ Answer:', d.get('answer','')[:150]+'...' if d.get('answer') else '❌ No answer')"

echo ""
echo "=============================================="
echo "🏁 Test Complete!"
echo "=============================================="
