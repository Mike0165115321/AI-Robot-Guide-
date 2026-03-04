# ⚡ Quick Start Guide

> สำหรับนักพัฒนาที่มี **Docker** และ **Python 3.12+** พร้อมแล้ว — เริ่มต้นได้ใน 3 นาที

---

## Prerequisites

| Software | Version | Check Command |
|:---|:---|:---|
| Python | 3.12+ | `python3 --version` |
| Docker Engine | 20.10+ | `docker --version` |
| Docker Compose | V2+ | `docker compose version` |
| Git | Any | `git --version` |

---

## Step 1 — Clone & Enter Project

```bash
git clone https://github.com/Mike0165115321/AI-Robot-Guide-.git
cd "AI Robot Guide จังหวัดน่าน"
```

## Step 2 — Start Infrastructure (MongoDB, Qdrant, Redis)

```bash
docker compose up -d
```

ตรวจสอบว่า containers ทำงาน:
```bash
docker ps
# ควรเห็น: mongodb_db, qdrant_db, redis_mq
```

## Step 3 — Setup Python Environment

```bash
python3 -m venv .venv-robot
source .venv-robot/bin/activate        # Linux/macOS
# .venv-robot\Scripts\activate         # Windows

pip install --upgrade pip
pip install -r Back-end/requirements.txt
```

> ⏱️ การติดตั้ง Dependencies ครั้งแรกอาจใช้เวลา 5-15 นาที (รวม PyTorch + Sentence Transformers)

## Step 4 — Configure API Keys

```bash
cp Back-end/.env.example Back-end/.env
```

แก้ไข `Back-end/.env` ใส่ API Keys ที่จำเป็น:

```env
# (Required) — AI Inference
GEMINI_API_KEYS=your_gemini_key_1,your_gemini_key_2
GROQ_API_KEYS=your_groq_key_1,your_groq_key_2

# (Optional) — Extended Features
YOUTUBE_API_KEY=your_youtube_api_key
GOOGLE_API_KEY=your_google_api_key
GOOGLE_CSE_ID=your_cse_id
```

📖 **ดูวิธีขอ API Keys ทั้งหมด:** [SETUP_API_KEYS.md](SETUP_API_KEYS.md)

## Step 5 — Build Vector Database (First Run Only)

```bash
# สร้าง Embedding Vectors จากข้อมูลสถานที่
python3 Back-end/scripts/build_vectors.py
```

> ⏱️ ครั้งแรกจะดาวน์โหลด Embedding Model (~2GB) + สร้าง Vectors อาจใช้เวลา 5-30 นาที

## Step 6 — Launch! 🚀

```bash
./start_web.sh       # Web Development Mode (แนะนำสำหรับเริ่มต้น)
```

เข้าใช้งาน:

| Service | URL |
|:---|:---|
| 🌐 Frontend | http://localhost:8014 |
| 📊 Admin Panel | http://localhost:8014/admin |
| 📖 API Docs (Swagger) | http://localhost:8014/docs |

---

## Operation Modes

| Script | Mode | Description |
|:---|:---|:---|
| `./start_web.sh` | 🌐 **Web Dev** | Frontend + Backend เท่านั้น (เหมาะสำหรับพัฒนา) |
| `./start_all.sh` | 🚀 **Production** | ครบทุกอย่าง: Web + LINE Worker + ngrok |
| `./start_line.sh` | 📱 **LINE Only** | เน้นทดสอบ LINE OA + Webhook |

> 📖 ดูขั้นตอนติดตั้งแบบละเอียด (สำหรับผู้เริ่มต้น): [SETUP_FULL_INSTALLATION.md](SETUP_FULL_INSTALLATION.md)

---

## Stopping Services

```bash
# กด Ctrl+C เพื่อหยุด Backend
# จากนั้นหยุด Docker:
docker compose down
```
