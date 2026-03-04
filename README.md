<div align="center">

# 🤖 AI Robot Guide — จังหวัดน่าน

### Advanced Hybrid Intelligence Platform for Nan Province Tourism

**ระบบ AI นำเที่ยวอัจฉริยะที่มี "สมองคู่ขนาน" — ไม่ใช่แค่ Chatbot ธรรมดา**
**แต่เป็น Cognitive Architecture ที่คิด ฟัง พูด และนำทางได้จริง**

<br/>

[![Python](https://img.shields.io/badge/Python-3.12-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://python.org)
[![FastAPI](https://img.shields.io/badge/FastAPI-Async-009688?style=for-the-badge&logo=fastapi&logoColor=white)](https://fastapi.tiangolo.com)
[![Gemini](https://img.shields.io/badge/Deep_Brain-Gemini_2.5-8E75B2?style=for-the-badge&logo=google&logoColor=white)](https://deepmind.google/technologies/gemini/)
[![Llama](https://img.shields.io/badge/Fast_Brain-Llama_3.3-0467DF?style=for-the-badge&logo=meta&logoColor=white)](https://llama.meta.com/)
[![Qdrant](https://img.shields.io/badge/Vector_DB-Qdrant-DC382D?style=for-the-badge&logo=qdrant&logoColor=white)](https://qdrant.tech)
[![MongoDB](https://img.shields.io/badge/Database-MongoDB_7-47A248?style=for-the-badge&logo=mongodb&logoColor=white)](https://mongodb.com)
[![Redis](https://img.shields.io/badge/Queue-Redis-FF4438?style=for-the-badge&logo=redis&logoColor=white)](https://redis.io)
[![LINE](https://img.shields.io/badge/LINE-OA_Integration-00C300?style=for-the-badge&logo=line&logoColor=white)](https://developers.line.biz/)

</div>

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Key Features](#-key-features)
- [System Architecture](#-system-architecture)
- [Project Structure](#-project-structure)
- [Quick Start](#-quick-start)
- [Tech Stack](#-tech-stack)
- [Documentation Hub](#-documentation-hub)
- [Awards & Achievements](#-awards--achievements)
- [Roadmap](#-roadmap)
- [Credits](#-credits)

---

## 🌏 Overview

**AI Robot Guide จังหวัดน่าน** คือแพลตฟอร์ม AI อัจฉริยะสำหรับการท่องเที่ยวจังหวัดน่าน ที่ทำงานทั้งบน **Web**, **LINE OA**, และ **หุ่นยนต์จริง** — ออกแบบมาเพื่อแก้ปัญหาจริงที่นักท่องเที่ยวพบ

> **Problem:** นักท่องเที่ยวหาข้อมูลเชิงลึกเกี่ยวกับจังหวัดน่านได้ยาก โดยเฉพาะข้อมูลที่ต้องการความเข้าใจบริบท
>
> **Solution:** AI ที่มี "สมองคู่ขนาน" — ตอบเร็วด้วย Llama 3.3 (< 0.5 วินาที) และวิเคราะห์ลึกด้วย Gemini 2.5 — รองรับ 7 ภาษา พร้อม Avatar แสดงอารมณ์ตามบทสนทนา

### ⚡ Engineering Highlights

| Metric | Value | Detail |
|:---|:---|:---|
| **Throughput** | 10,000+ req/sec | Async Architecture (FastAPI + AsyncIO) |
| **AI Latency** | < 0.5s | Optimized RAG pipeline + streaming TTS |
| **RAG Accuracy** | Cross-Encoder Reranking | E5-Large + BGE Reranker 3 ชั้น |
| **Languages** | 7 Languages | TH, EN, JA, ZH, RU, HI, MS |
| **Availability** | Graceful Degradation | Fallback: Cloud TTS → Edge-TTS → gTTS |

---

## ✨ Key Features

### 🧠 Dual-Brain System — ระบบสมองคู่ขนาน

ไม่ใช้ AI ตัวเดียวทำทุกอย่าง — เราแบ่งงานให้ AI สองตัวทำงานร่วมกัน:

| Brain | Model | Role | Speed |
|:---|:---|:---|:---|
| 🚀 **Fast Brain** | Llama 3.3 70B (Groq) | Small Talk, คำทักทาย, บทสนทนาทั่วไป | < 0.5s |
| 💡 **Deep Brain** | Gemini 2.5 Flash | วิเคราะห์ข้อมูลซับซ้อน, วางแผนเที่ยว, อ่านรีวิว | 1-3s |

ระบบ **Intent Router** จะวิเคราะห์เจตนาผู้ใช้แล้วส่งไปยัง Brain ที่เหมาะสม — ลด API cost 60% โดยไม่เสียคุณภาพ

### 🛡️ 3-Layer Intent Filter — ระบบกรองความคิด 3 ชั้น

ก่อนที่ AI จะตอบ ระบบกรอง 3 ตลบเพื่อให้แม่นยำที่สุด:

1. **Reflex Layer** — Regex + Pre-correction: แก้คำผิดและตอบสนองคำสั่งด่วนทันที (0ms)
2. **Memory Layer** — Vector Search: ค้นหาข้อมูลจาก Qdrant ด้วย Semantic Search
3. **Logic Layer** — LLM Classification: ใช้ AI ตัดสินใจเจตนาที่แท้จริง

### 🔍 Advanced RAG Pipeline

ระบบ **Retrieval Augmented Generation** ที่ไม่ได้หาแค่ Keyword:

- ✅ **Semantic Search** — ค้นหาจาก "ความหมาย" (ถาม "ที่เงียบๆ" ได้ "วัดป่า" แม้ไม่มีคำว่าเงียบ)
- ✅ **Cross-Encoder Reranking** — จัดอันดับผลลัพธ์ด้วย BGE Reranker
- ✅ **Broad Query Detection** — แยกแยะคำถามกว้าง vs เจาะจงอัตโนมัติ
- ✅ **Knowledge Gap Detection** — ตรวจจับคำถามที่ระบบยังตอบไม่ได้ + แจ้ง Admin

### 🎭 Avatar V2 System — น้องน่าน

ระบบ Avatar แบบ Modular ที่สร้างจาก GSAP Animation Engine:

- 🎤 **Real-time Lip Sync** — ขยับปากตามเสียงพูด
- 😊 **Mood System** — แสดงอารมณ์ตามบทสนทนา (Happy, Thinking, Excited, Sad)
- 🎨 **Skin System** — เปลี่ยนธีมได้ (Default, NeonPunk, Winter, Thai Traditional ฯลฯ)
- 🖱️ **Interactive Behaviors** — ตอบสนองการคลิก, hover, idle animation

### 🌐 Multi-Language Support (i18n)

รองรับ **7 ภาษา** ทั้ง UI และ AI Response:

| 🇹🇭 Thai | 🇬🇧 English | 🇯🇵 Japanese | 🇨🇳 Chinese | 🇷🇺 Russian | 🇮🇳 Hindi | 🇲🇾 Malay |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|

**Smart TTS** สลับสำเนียงอัตโนมัติตามภาษาที่ AI ตอบ — ถ้า AI ตอบเป็นภาษาอังกฤษจะใช้เสียง English, ไทยจะใช้เสียงไทย

### 📱 Multi-Platform

| Platform | Feature |
|:---|:---|
| 🌐 **Web UI** | Full-featured chat, avatar, navigation, places gallery |
| 📱 **LINE OA** | สนทนาผ่าน LINE พร้อม Rich Card responses |
| 🤖 **Robot** | ROS 2 + Arduino — เคลื่อนที่, LiDAR, Wake Word |

### 🛠️ Admin Dashboard

ระบบจัดการหลังบ้านครบวงจร:

- 📊 **Dashboard** — สถิติการใช้งาน, response time, confidence scores
- 📥 **Smart Import** — นำเข้าข้อมูลจาก Excel, JSON, PDF, Google Sheets
- 🧠 **Knowledge Gaps** — Self-Correcting RAG: ดูคำถามที่ AI ตอบไม่ได้
- 🗺️ **Map Manager** — จัดการ waypoints สำหรับนำทาง
- ⚙️ **Settings** — ตั้งค่าระบบ, credentials
- 🔧 **Hardware Test** — ทดสอบการเชื่อมต่อ ROS 2 + Arduino

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         USER INTERFACE                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌────────────────────┐  │
│  │  Web UI  │  │ LINE OA  │  │  Robot   │  │   Admin Dashboard  │  │
│  │ (Avatar) │  │(Webhook) │  │(ROS 2)   │  │ (Import/Analytics) │  │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────────┬───────────┘  │
│       │              │             │                  │              │
│       ▼              ▼             ▼                  ▼              │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │              Frontend Director (Local Brain)                 │   │
│  │     Reflex Layer → Canned Responses → WebSocket Relay       │   │
│  └──────────────────────────┬───────────────────────────────────┘   │
└─────────────────────────────┼───────────────────────────────────────┘
                              │ WebSocket (JSON/Binary)
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    BACKEND (FastAPI + Python 3.12)                   │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    API Gateway (14 Routers)                  │   │
│  │  chat | admin | import | sheets | analytics | line | alert  │   │
│  │  auth | assistant | knowledge | hardware | navigation       │   │
│  └──────────────────────────┬──────────────────────────────────┘   │
│                              │                                      │
│  ┌──────────────────────────▼──────────────────────────────────┐   │
│  │                   RAG Orchestrator (Brain)                   │   │
│  │                                                              │   │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────────────┐    │   │
│  │  │ Fast Brain │  │ Deep Brain │  │ Query Interpreter  │    │   │
│  │  │ Llama 3.3  │  │ Gemini 2.5 │  │ (Intent Routing)   │    │   │
│  │  └────────────┘  └────────────┘  └────────────────────┘    │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  ┌─── Core Services ──────────────────────────────────────────┐   │
│  │ Speech (Whisper/TTS) │ Navigation │ Weather │ Air Quality  │   │
│  │ Image Service │ Analytics │ News Monitor │ Alert Manager   │   │
│  │ Google Sheets Sync │ YouTube │ Geocoding │ Language Detect │   │
│  └────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  ┌─── Background Workers ────────────────────────────────────┐    │
│  │ LINE Worker (Redis Consumer) │ News Scheduler (APScheduler)│    │
│  └────────────────────────────────────────────────────────────┘    │
└──────────────────┬────────────────┬───────────────┬─────────────────┘
                   │                │               │
                   ▼                ▼               ▼
            ┌────────────┐  ┌────────────┐  ┌────────────┐
            │  MongoDB 7 │  │   Qdrant   │  │   Redis    │
            │  Documents │  │  Vectors   │  │   Queue    │
            │  Logs/Data │  │  Semantic   │  │  Pub/Sub   │
            └────────────┘  └────────────┘  └────────────┘
```

### Architecture Highlights

- **Monolithic Modular** — Single deployment แต่แยก module ชัดเจนตาม SoC
- **100% Async** — FastAPI + AsyncIO ทั้งระบบ
- **3 Databases** — แต่ละ DB ทำหน้าที่เฉพาะทาง (Document Store / Vector Search / Message Queue)
- **Graceful Degradation** — ถ้า Cloud TTS ล่ม → สลับ Edge-TTS → gTTS อัตโนมัติ
- **Key Rotation** — สลับ API Keys อัตโนมัติเมื่อถูก Rate Limit

---

## 📁 Project Structure

```
AI Robot Guide จังหวัดน่าน/
│
├── Back-end/                    # 🐍 Python Backend (FastAPI)
│   ├── api/                     #   ├── API Layer
│   │   ├── main.py              #   │   ├── Application entry point & lifespan
│   │   ├── routers/             #   │   └── 14 API routers (chat, admin, import, LINE...)
│   │   └── schemas.py           #   │
│   ├── core/                    #   ├── Business Logic Layer
│   │   ├── ai_models/           #   │   ├── RAG Orchestrator, LLM Handlers, Speech
│   │   ├── services/            #   │   ├── 23 Services (navigation, weather, analytics...)
│   │   ├── database/            #   │   ├── MongoDB, Qdrant, Redis managers
│   │   ├── hardware/            #   │   └── ROS 2 Bridge, Kinematics, Odometry
│   │   └── config.py            #   │
│   ├── prompts/                 #   ├── AI Prompts (7 languages: th/en/ja/zh/ru/hi/ms)
│   ├── workers/                 #   ├── Background Workers (LINE consumer)
│   ├── scripts/                 #   └── Utility scripts (build_vectors, clear_database...)
│   └── .env.example             #
│
├── frontend/                    # 🌐 Web Frontend
│   ├── index.html               #   ├── Main page
│   ├── css/                     #   ├── Modular CSS (tokens → base → utilities)
│   ├── js/                      #   ├── JavaScript
│   │   ├── app.js               #   │   ├── Main application
│   │   ├── services/            #   │   ├── Services (chat, speech, websocket, avatar...)
│   │   ├── modules/             #   │   ├── Modules (AvatarManager, UIManager, StateManager)
│   │   ├── components/          #   │   ├── Components (Navbar, Toast, FAB, ResponseRenderer)
│   │   └── translations/        #   │   └── i18n (8 language files)
│   └── admin/                   #   └── Admin Panel (dashboard, import, knowledge-gaps...)
│
├── avatar/                      # 🎭 Avatar V2 Engine
│   ├── core/                    #   ├── AvatarController, AnimationEngine, EventBus
│   ├── moods/                   #   ├── Mood plugins (Happy, Sad, Thinking, Excited...)
│   ├── skins/                   #   └── Theme plugins (Default, NeonPunk, Winter...)
│   └── behaviors/               #
│
├── hardware/                    # 🔧 Hardware Integration
│   └── arduino/                 #   └── Arduino base control (ESP32)
│
├── docs/                        # 📚 Documentation
│   ├── SETUP_QUICKSTART.md      #   ├── ⚡ Quick Start (3 minutes)
│   ├── SETUP_FULL_INSTALLATION.md #  ├── 📖 Full Installation Guide
│   ├── SETUP_API_KEYS.md        #   ├── 🔑 API Keys Configuration
│   ├── 01_สถาปัตยกรรมระบบ/       #   ├── 🏛️ Architecture Deep-dive
│   ├── 05_คู่มือการใช้งาน/        #   ├── 📘 User Guides
│   └── 07_รางวัลและผลงาน/        #   └── 🏆 Awards & Certificates
│
├── docker-compose.yml           # 🐳 Infrastructure (MongoDB + Qdrant + Redis)
├── start_web.sh                 # 🌐 Web Development mode
├── start_all.sh                 # 🚀 Full Production mode (Web + LINE + ngrok)
├── start_line.sh                # 📱 LINE Integration mode
└── requirements.txt             # 📦 Python dependencies
```

---

## 🚀 Quick Start

### Prerequisites

- **Python 3.12+** / **Docker + Compose V2** / **Git**

### Get Running in 3 Minutes

```bash
# 1. Clone
git clone https://github.com/Mike0165115321/AI-Robot-Guide-.git
cd "AI Robot Guide จังหวัดน่าน"

# 2. Start databases
docker compose up -d

# 3. Setup Python
python3 -m venv .venv-robot && source .venv-robot/bin/activate
pip install -r Back-end/requirements.txt

# 4. Configure API Keys
cp Back-end/.env.example Back-end/.env
# Edit Back-end/.env → add GEMINI_API_KEYS and GROQ_API_KEYS

# 5. Build Vector DB (first run only, ~5-30 min)
python3 Back-end/scripts/build_vectors.py

# 6. Launch! 🚀
./start_web.sh
```

**เปิดเบราว์เซอร์:** http://localhost:8014

### 📖 ดูขั้นตอนละเอียดเพิ่มเติม

| Guide | For |
|:---|:---|
| **[⚡ Quick Start](docs/SETUP_QUICKSTART.md)** | นักพัฒนาที่มี Docker + Python พร้อมแล้ว |
| **[📖 Full Installation](docs/SETUP_FULL_INSTALLATION.md)** | ผู้เริ่มต้น — ติดตั้งทุกอย่างตั้งแต่ศูนย์ |
| **[🔑 API Keys Guide](docs/SETUP_API_KEYS.md)** | วิธีขอ API Keys ทุกตัว (Gemini, Groq, YouTube, LINE...) |
| **[🐳 Docker Guide](docs/05_คู่มือการใช้งาน/คู่มือการติดตั้ง_Docker.md)** | การติดตั้ง Docker โดยละเอียด |

### Operation Modes

| Script | Mode | Description | Port |
|:---|:---|:---|:---|
| `./start_web.sh` | 🌐 **Web Dev** | Frontend + Backend (เหมาะสำหรับพัฒนา) | `8014` |
| `./start_all.sh` | 🚀 **Production** | ครบทุกอย่าง: Web + LINE Worker + ngrok | `8014` |
| `./start_line.sh` | 📱 **LINE Only** | ทดสอบ LINE OA + Webhook | `8014` |

### Access Points

| Service | URL |
|:---|:---|
| 🌐 Frontend | http://localhost:8014 |
| 📊 Admin Panel | http://localhost:8014/admin |
| 📖 Swagger API | http://localhost:8014/docs |
| ❤️ Health Check | http://localhost:8014/health |

---

## 🔧 Tech Stack

### AI & Machine Learning

| Component | Technology | Purpose |
|:---|:---|:---|
| **Deep Brain** | Google Gemini 2.5 Flash | Complex reasoning, trip planning, data analysis |
| **Fast Brain** | Meta Llama 3.3 70B (via Groq) | Small talk, greetings, quick responses |
| **Small Talk** | Llama 3.1 8B Instant (via Groq) | Ultra-fast casual conversation |
| **Embedding** | `intfloat/multilingual-e5-large` | Multilingual semantic vector encoding |
| **Reranker** | `BAAI/bge-reranker-base` | Cross-encoder relevance reranking |
| **STT** | Whisper Large V3 (Groq Cloud) | Speech-to-Text with local fallback |
| **TTS** | Edge-TTS + gTTS | Text-to-Speech with multi-voice support |

### Backend

| Component | Technology | Purpose |
|:---|:---|:---|
| **Framework** | FastAPI (Python 3.12) | Async API gateway, WebSocket server |
| **Document DB** | MongoDB 7 | Location data, logs, analytics, user sessions |
| **Vector DB** | Qdrant | Semantic search, vector embeddings storage |
| **Message Queue** | Redis | Async task queue (LINE worker, news scheduler) |
| **Task Scheduler** | APScheduler | Periodic news monitoring & alerts |

### Frontend

| Component | Technology | Purpose |
|:---|:---|:---|
| **UI** | HTML5 + Vanilla JS | Lightweight, no framework overhead |
| **Animation** | GSAP | Avatar animation engine, micro-interactions |
| **Communication** | WebSocket | Full-duplex real-time streaming |
| **i18n** | Custom translation system | 7-language UI support |
| **Styling** | Modular CSS (tokens/base/utilities) | Design system architecture |

### Infrastructure & Integration

| Component | Technology | Purpose |
|:---|:---|:---|
| **Containerization** | Docker Compose | MongoDB, Qdrant, Redis orchestration |
| **LINE OA** | LINE Messaging API | Chat via LINE + Rich Card responses |
| **Robot** | ROS 2 + Arduino (ESP32) | Physical robot control, LiDAR navigation |
| **Tunnel** | ngrok | LINE webhook public URL |
| **Data Import** | Google Sheets API | Admin data sync from spreadsheets |

---

## 📚 Documentation Hub

### 🏁 Getting Started

| Document | Description |
|:---|:---|
| [⚡ Quick Start](docs/SETUP_QUICKSTART.md) | เริ่มต้นใช้งานใน 3 นาที |
| [📖 Full Installation](docs/SETUP_FULL_INSTALLATION.md) | ติดตั้งแบบละเอียดสำหรับผู้เริ่มต้น |
| [🔑 API Keys Guide](docs/SETUP_API_KEYS.md) | วิธีขอและตั้งค่า API Keys ทุกตัว |
| [🐳 Docker Guide](docs/05_คู่มือการใช้งาน/คู่มือการติดตั้ง_Docker.md) | คู่มือ Docker โดยละเอียด |
| [📘 User Guide](docs/05_คู่มือการใช้งาน/คู่มือผู้ใช้.md) | คู่มือการใช้งานสำหรับผู้ใช้ทั่วไป |

### 🏛️ Architecture Deep-dive

| Document | Description |
|:---|:---|
| [🏗️ Architecture Bible](docs/01_สถาปัตยกรรมระบบ/01_สถาปัตยกรรมและโครงสร้างระบบ_ฉบับสมบูรณ์.md) | คัมภีร์โครงสร้างระบบฉบับสมบูรณ์ |
| [🧠 AI Cognitive Process](docs/01_สถาปัตยกรรมระบบ/02_เจาะลึกกระบวนการคิดของAI.md) | เจาะลึก Dual-Brain, Intent Filter, RAG Pipeline |
| [⚙️ Backend Services](docs/01_สถาปัตยกรรมระบบ/02_เจาะลึกบริการหลังบ้าน.md) | News Scheduler, Redis Queue, Background Workers |
| [🎭 Frontend & Avatar](docs/01_สถาปัตยกรรมระบบ/03_เจาะลึกระบบหน้าบ้านและอวตาร.md) | Frontend Director, Avatar Engine, State Management |
| [📱 LINE OA System](docs/01_สถาปัตยกรรมระบบ/04_ระบบเชื่อมต่อ_LINE_OA_ฉบับสมบูรณ์.md) | LINE Messaging API integration |
| [🔧 Maintenance Guide](docs/01_สถาปัตยกรรมระบบ/05_คู่มือการดูแลและกู้คืนระบบ.md) | การดูแลและกู้คืนระบบ |

### 🏅 Hall of Fame

| Document | Description |
|:---|:---|
| [🏆 Awards & Showcase](docs/07_รางวัลและผลงาน/PROJECT_SHOWCASE.md) | รางวัลและเกียรติบัตรจากการแข่งขัน |
| [📝 Post-Mortem](docs/07_รางวัลและผลงาน/POST_MORTEM.md) | บทเรียนและสิ่งที่เรียนรู้จากการทำงาน |

---

## 🏆 Awards & Achievements

โครงการนี้ได้รับรางวัลจากการแข่งขันระดับประเทศ:

📄 ดูเกียรติบัตรทั้งหมดได้ที่ [Project Showcase](docs/07_รางวัลและผลงาน/PROJECT_SHOWCASE.md)

---

## 🗺️ Roadmap

### Q1 2026 — Production Scale

| Phase | Objective | Strategy |
|:---|:---|:---|
| **Deployment** | Container Orchestration | Docker Compose → Kubernetes (K8s) on GKE/EKS |
| **Observability** | Deep Monitoring | Prometheus + Grafana (latency, error rates, token usage) |
| **Quality** | Rigorous Testing | CI/CD + Pytest (>80% coverage) + E2E testing |
| **Scale** | High Concurrency | 1,000+ concurrent users, Nginx LB, Redis Cluster |

### Future Vision

- 🗺️ **Graph-based Relation** — เชื่อมโยงความสัมพันธ์ระหว่างสถานที่ (Knowledge Graph)
- 📸 **Vision AI** — วิเคราะห์ภาพสถานที่จากกล้องหุ่นยนต์
- 🎙️ **Emotion Detection** — ตรวจจับอารมณ์จากเสียงผู้ใช้
- 🌍 **Multi-Province** — ขยายไปรองรับจังหวัดอื่นๆ

---

## 👨‍💻 Credits

Developed with ❤️ by **MikeDev Team**

*Pushing the boundaries of Local AI for Thai Tourism*

[![GitHub](https://img.shields.io/badge/GitHub-Repository-181717?style=for-the-badge&logo=github)](https://github.com/Mike0165115321/AI-Robot-Guide-)

---

## 📄 License

This project is developed for educational and research purposes as part of the Nan Province AI Tourism initiative.
