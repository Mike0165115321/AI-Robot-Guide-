# 🔑 API Keys Configuration Guide

> คู่มือการขอและตั้งค่า API Keys ทุกตัวที่ใช้ในโปรเจค AI Robot Guide

---

## สารบัญ

| API Key | ความสำคัญ | ใช้งานสำหรับ |
|:---|:---|:---|
| [GEMINI_API_KEYS](#1-gemini_api_keys) | ⭐ **Required** | Deep Brain — AI วิเคราะห์ข้อมูลซับซ้อน |
| [GROQ_API_KEYS](#2-groq_api_keys) | ⭐ **Required** | Fast Brain — AI สนทนาทั่วไป |
| [YOUTUBE_API_KEY](#3-youtube_api_key) | Optional | ค้นหาและเล่นเพลง/วิดีโอ |
| [GOOGLE_API_KEY](#4-google_api_key--google_cse_id) | Optional | ค้นหารูปภาพสถานที่ |
| [GOOGLE_CSE_ID](#4-google_api_key--google_cse_id) | Optional | Custom Search Engine ID |
| [OPENWEATHER_API_KEY](#5-weather--air-quality-apis) | Optional | ข้อมูลสภาพอากาศ |
| [TMD_API_KEY](#5-weather--air-quality-apis) | Optional | กรมอุตุนิยมวิทยา |
| [WAQI_API_KEY](#5-weather--air-quality-apis) | Optional | คุณภาพอากาศ (PM2.5) |
| [LINE Keys](#6-line-oa-integration) | Optional | เชื่อมต่อ LINE Official Account |

---

## 1. GEMINI_API_KEYS

**ใช้สำหรับ:** Gemini 2.5 Flash — Deep Brain สำหรับวิเคราะห์ข้อมูลซับซ้อน, วางแผนเที่ยว, RAG reasoning

### วิธีขอ:
1. ไปที่ [Google AI Studio](https://aistudio.google.com/app/apikey)
2. ลงชื่อเข้าใช้ด้วย Google Account
3. คลิก **"Create API Key"**
4. เลือก Project หรือสร้างใหม่
5. คัดลอก API Key

### Format ใน .env:
```env
# รองรับหลาย Key (Key Rotation) — คั่นด้วย comma
GEMINI_API_KEYS=AIzaSy...key1,AIzaSy...key2,AIzaSy...key3
```

> **💡 Tip:** แนะนำให้สร้าง 2-3 keys เพื่อใช้ระบบ Key Rotation หลีกเลี่ยง Rate Limit

---

## 2. GROQ_API_KEYS

**ใช้สำหรับ:** Llama 3.3 70B (via Groq) — Fast Brain สำหรับ Small Talk + Whisper Large V3 (Speech-to-Text)

### วิธีขอ:
1. ไปที่ [Groq Console](https://console.groq.com/keys)
2. ลงทะเบียน / ลงชื่อเข้าใช้
3. คลิก **"Create API Key"**
4. ตั้งชื่อ Key แล้วคัดลอก

### Format ใน .env:
```env
GROQ_API_KEYS=gsk_...key1,gsk_...key2
```

> **💡 Tip:** Groq Free Tier มี Rate Limit ต่อนาที — ใส่หลาย keys ช่วยให้ระบบไหลลื่นขึ้น

---

## 3. YOUTUBE_API_KEY

**ใช้สำหรับ:** ค้นหาและสตรีมเพลง/วิดีโอเมื่อผู้ใช้ขอฟังเพลง

### วิธีขอ:
1. ไปที่ [Google Cloud Console](https://console.cloud.google.com/)
2. สร้าง/เลือก Project
3. ไปที่ **APIs & Services** → **Library**
4. ค้นหา **"YouTube Data API v3"** → คลิก **Enable**
5. ไปที่ **Credentials** → **+ CREATE CREDENTIALS** → **API Key**
6. คัดลอก Key

### Format ใน .env:
```env
YOUTUBE_API_KEY=AIzaSy...your_key
```

---

## 4. GOOGLE_API_KEY & GOOGLE_CSE_ID

**ใช้สำหรับ:** ค้นหารูปภาพสถานที่ท่องเที่ยวจาก Google Images (Image Fallback)

### วิธีขอ GOOGLE_API_KEY:
1. ใช้ Key เดียวกับ YouTube API Key ได้ (ถ้าเปิดใช้ Custom Search API ด้วย)
2. หรือสร้าง Key ใหม่ที่ [Google Cloud Credentials](https://console.cloud.google.com/apis/credentials)
3. เปิด **"Custom Search API"** ใน Library

### วิธีขอ GOOGLE_CSE_ID:
1. ไปที่ [Programmable Search Engine](https://programmablesearchengine.google.com/controlpanel/all)
2. คลิก **"Add"** → สร้าง Search Engine ใหม่
3. ในส่วน **"Search the entire web"** → เลือก On
4. คลิก **Create** → ไปที่ **Setup** → คัดลอก **Search Engine ID**

### Format ใน .env:
```env
GOOGLE_API_KEY=AIzaSy...your_key
GOOGLE_CSE_ID=a1b2c3d4e5f6g7h8i
```

---

## 5. Weather & Air Quality APIs

**ใช้สำหรับ:** แสดงข้อมูลสภาพอากาศ, พยากรณ์, และคุณภาพอากาศ (PM2.5)

| Key | Service | URL สมัคร |
|:---|:---|:---|
| `OPENWEATHER_API_KEY` | OpenWeatherMap | [openweathermap.org/api](https://openweathermap.org/api) |
| `TMD_API_KEY` | กรมอุตุนิยมวิทยา | [data.tmd.go.th](https://data.tmd.go.th/) |
| `WAQI_API_KEY` | World Air Quality Index | [aqicn.org/data-platform](https://aqicn.org/data-platform/token/) |

### Format ใน .env:
```env
OPENWEATHER_API_KEY=your_key
TMD_API_KEY=your_key
WAQI_API_KEY=your_token
```

---

## 6. LINE OA Integration

**ใช้สำหรับ:** เชื่อมต่อ LINE Official Account — ให้ผู้ใช้คุยกับ AI ผ่าน LINE

### วิธีขอ:
1. ไปที่ [LINE Developers Console](https://developers.line.biz/console/)
2. สร้าง Provider → สร้าง Messaging API Channel
3. ในแท็บ **Messaging API** → คัดลอก **Channel Access Token**
4. ในแท็บ **Basic Settings** → คัดลอก **Channel Secret**

### Format ใน .env:
```env
LINE_CHANNEL_ACCESS_TOKEN=your_channel_access_token
LINE_CHANNEL_SECRET=your_channel_secret
```

> **📖 ดูคู่มือ LINE แบบละเอียด:** [04_ระบบเชื่อมต่อ_LINE_OA_ฉบับสมบูรณ์.md](01_สถาปัตยกรรมระบบ/04_ระบบเชื่อมต่อ_LINE_OA_ฉบับสมบูรณ์.md)

---

## ไฟล์ .env ตัวอย่างฉบับสมบูรณ์

```env
# =======================================
# Database Configuration
# =======================================
MONGO_URI=mongodb://localhost:27017/
MONGO_DATABASE_NAME=nanaiguide
QDRANT_HOST=localhost
QDRANT_PORT=6333

# =======================================
# AI Models
# =======================================
EMBEDDING_MODEL_NAME=intfloat/multilingual-e5-large
RERANKER_MODEL_NAME=BAAI/bge-reranker-base

# =======================================
# API Keys (Required)
# =======================================
GEMINI_API_KEYS=key1,key2
GROQ_API_KEYS=key1,key2

# =======================================
# API Keys (Optional - Extended Features)
# =======================================
YOUTUBE_API_KEY=
GOOGLE_API_KEY=
GOOGLE_CSE_ID=

# Weather & Air Quality
OPENWEATHER_API_KEY=
TMD_API_KEY=
WAQI_API_KEY=

# LINE OA
LINE_CHANNEL_ACCESS_TOKEN=
LINE_CHANNEL_SECRET=

# =======================================
# Server Config
# =======================================
API_HOST=0.0.0.0
API_PORT=8014
```

---

## ⚠️ ข้อควรระวัง

1. **ห้าม commit `.env` ขึ้น Git** — ไฟล์นี้อยู่ใน `.gitignore` แล้ว
2. **ห้ามมีช่องว่าง** รอบเครื่องหมาย `=` (เช่น `KEY = value` ❌)
3. **หลาย Keys คั่นด้วย comma** ไม่มีช่องว่าง (เช่น `key1,key2` ✅)
4. **Key Rotation** — ระบบจะสลับใช้ keys อัตโนมัติเมื่อ key ใด key หนึ่ง Rate Limited
