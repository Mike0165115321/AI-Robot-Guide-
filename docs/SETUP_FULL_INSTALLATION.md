# 📖 Full Installation Guide — AI Robot Guide จังหวัดน่าน

> คู่มือการติดตั้งฉบับสมบูรณ์ สำหรับผู้เริ่มต้นที่ยังไม่มีซอฟต์แวร์พื้นฐาน

---

## สารบัญ

- [1. ความต้องการของระบบ](#1-ความต้องการของระบบ)
- [2. ติดตั้ง Python](#2-ติดตั้ง-python)
- [3. ติดตั้ง Docker](#3-ติดตั้ง-docker)
- [4. ติดตั้ง Git](#4-ติดตั้ง-git-ไม่บังคับ)
- [5. ดาวน์โหลดโปรเจค](#5-ดาวน์โหลดโปรเจค)
- [6. ตั้งค่า Python Environment](#6-ตั้งค่า-python-environment)
- [7. ตั้งค่า API Keys](#7-ตั้งค่า-api-keys)
- [8. ตั้งค่าฐานข้อมูล](#8-ตั้งค่าฐานข้อมูล)
- [9. สร้าง Vector Database](#9-สร้าง-vector-database)
- [10. รันระบบ](#10-รันระบบ)
- [11. ทดสอบระบบ](#11-ทดสอบระบบ)
- [12. แก้ไขปัญหาเบื้องต้น](#12-แก้ไขปัญหาเบื้องต้น)

---

## 1. ความต้องการของระบบ

### Hardware ขั้นต่ำ

| Component | Minimum | Recommended |
|:---|:---|:---|
| CPU | Intel Core i5 / เทียบเท่า | Intel Core i7 / เทียบเท่า |
| RAM | 8 GB | 16 GB+ |
| Storage | 20 GB ว่าง | 50 GB+ |
| GPU | ไม่จำเป็น | NVIDIA GPU (สำหรับ Local Whisper) |
| Network | Internet required | Stable broadband |

### Software ที่ต้องติดตั้ง

| Software | Version | Required |
|:---|:---|:---|
| Python | 3.12+ | ✅ Yes |
| Docker + Compose V2 | 20.10+ | ✅ Yes |
| Git | Any | ⬜ Optional |

### ระบบปฏิบัติการที่รองรับ
- ✅ Ubuntu 20.04 LTS ขึ้นไป (แนะนำ)
- ✅ Windows 10/11 (ผ่าน WSL 2 หรือ Git Bash)
- ✅ macOS 11+

---

## 2. ติดตั้ง Python

### สำหรับ Linux (Ubuntu/Debian)

```bash
# อัปเดตระบบ
sudo apt update && sudo apt upgrade -y

# ติดตั้ง Python 3.12
sudo apt install python3.12 python3.12-venv python3-pip -y

# ตรวจสอบ
python3 --version
# ✅ Python 3.12.x
```

### สำหรับ Windows

1. ดาวน์โหลดจาก [python.org/downloads](https://www.python.org/downloads/)
2. **⚠️ สำคัญ:** ติ๊กเลือก **"Add python.exe to PATH"** ก่อนกด Install
3. คลิก **"Install Now"**
4. ตรวจสอบ:
   ```bash
   python --version
   # ✅ Python 3.12.x
   ```

### สำหรับ macOS

```bash
# ใช้ Homebrew
brew install python@3.12

python3 --version
# ✅ Python 3.12.x
```

---

## 3. ติดตั้ง Docker

### สำหรับ Linux (Ubuntu/Debian) — แนะนำวิธี Official

```bash
# 1. ลบเวอร์ชันเก่า (ถ้ามี)
sudo apt-get remove docker docker-engine docker.io containerd runc 2>/dev/null

# 2. ติดตั้ง Dependencies
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg -y

# 3. เพิ่ม Docker Official GPG Key
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# 4. เพิ่ม Repository
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 5. ติดตั้ง Docker Engine & Compose V2
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y

# 6. เพิ่ม User เข้ากลุ่ม Docker (ไม่ต้องใช้ sudo รันทุกครั้ง)
sudo usermod -aG docker $USER
# ⚠️ ต้อง Logout แล้ว Login ใหม่เพื่อให้มีผล

# 7. ตรวจสอบ
docker --version
docker compose version
# ✅ Docker Compose version v2.x.x
```

### สำหรับ Windows / macOS

1. ดาวน์โหลด [Docker Desktop](https://www.docker.com/products/docker-desktop/)
2. ติดตั้งตามขั้นตอน:
   - **Windows:** เลือก "Use WSL 2 instead of Hyper-V" → รีสตาร์ทเครื่อง
   - **macOS:** ลากไปใส่ Applications
3. เปิด Docker Desktop → รอ Docker Engine แสดง "Running"
4. ตรวจสอบ:
   ```bash
   docker --version
   docker compose version
   ```

---

## 4. ติดตั้ง Git (ไม่บังคับ)

Git ใช้สำหรับ clone โปรเจคจาก GitHub — หากไม่ใช้ สามารถดาวน์โหลด ZIP ได้

```bash
# Linux
sudo apt install git -y

# macOS
brew install git

# Windows — ดาวน์โหลดจาก https://git-scm.com/downloads
```

---

## 5. ดาวน์โหลดโปรเจค

### วิธีที่ 1: Clone ด้วย Git (แนะนำ)

```bash
git clone https://github.com/Mike0165115321/AI-Robot-Guide-.git
cd "AI Robot Guide จังหวัดน่าน"
```

### วิธีที่ 2: ดาวน์โหลด ZIP

1. ไปที่ [GitHub Repository](https://github.com/Mike0165115321/AI-Robot-Guide-)
2. คลิกปุ่ม **"Code"** สีเขียว → **"Download ZIP"**
3. แตกไฟล์ไปยังตำแหน่งที่ต้องการ
4. เปิด Terminal ที่โฟลเดอร์โปรเจค

---

## 6. ตั้งค่า Python Environment

### 6.1 สร้าง Virtual Environment

```bash
# Linux/macOS
python3 -m venv .venv-robot

# Windows
python -m venv .venv-robot
```

### 6.2 Activate Virtual Environment

```bash
# Linux/macOS
source .venv-robot/bin/activate

# Windows (Command Prompt)
.venv-robot\Scripts\activate

# Windows (PowerShell)
.venv-robot\Scripts\Activate.ps1
```

> ✅ เมื่อ activate สำเร็จ จะเห็น `(.venv-robot)` นำหน้า prompt

### 6.3 ติดตั้ง Dependencies

```bash
pip install --upgrade pip
pip install -r Back-end/requirements.txt
```

> ⏱️ ใช้เวลา ~5-15 นาที (รวม PyTorch, Sentence Transformers, Faster-Whisper)

### 6.4 ตรวจสอบ

```bash
pip list | head -20
# ควรเห็น fastapi, torch, sentence-transformers, qdrant-client ฯลฯ
```

---

## 7. ตั้งค่า API Keys

```bash
cp Back-end/.env.example Back-end/.env
```

แก้ไข `Back-end/.env` ด้วย Text Editor — ใส่ API Keys ที่จำเป็น

📖 **ดูคู่มือ API Keys แบบละเอียด:** [SETUP_API_KEYS.md](SETUP_API_KEYS.md)

---

## 8. ตั้งค่าฐานข้อมูล

### 8.1 เปิด Docker Containers

```bash
docker compose up -d
```

### 8.2 ตรวจสอบ

```bash
docker ps
```

| Container | Port | Status |
|:---|:---|:---|
| `mongodb_db` | 27017 | ✅ Up |
| `qdrant_db` | 6333, 6334 | ✅ Up |
| `redis_mq` | 6379 | ✅ Up |

Qdrant Dashboard: http://localhost:6333/dashboard

---

## 9. สร้าง Vector Database

> ⚠️ ขั้นตอนนี้ต้องทำ **เฉพาะครั้งแรก** หรือเมื่อต้องการ rebuild ข้อมูล

### 9.1 สร้าง Vectors

```bash
python3 Back-end/scripts/build_vectors.py
```

สิ่งที่จะเกิดขึ้น:
1. ดาวน์โหลด Embedding Model `intfloat/multilingual-e5-large` (~2GB ครั้งแรก)
2. อ่านข้อมูลสถานที่จากไฟล์ JSONL
3. สร้าง Vector Embeddings
4. บันทึกลง MongoDB + Qdrant

> ⏱️ ใช้เวลา 5-30 นาที ขึ้นกับ hardware

### 9.2 ตรวจสอบ

- เปิด [Qdrant Dashboard](http://localhost:6333/dashboard) → ดูว่ามี Collection `nan_locations`
- ตรวจ MongoDB:
  ```bash
  docker exec -it mongodb_db mongosh --eval "use nanaiguide; db.getCollectionNames()"
  ```

### 9.3 ล้างและสร้างใหม่ (ถ้าจำเป็น)

> ⚠️ **คำเตือน:** คำสั่งนี้จะลบข้อมูลทั้งหมด

```bash
python3 Back-end/scripts/clear_database.py
python3 Back-end/scripts/build_vectors.py
```

---

## 10. รันระบบ

### วิธีที่ 1: ใช้ Start Script (แนะนำ)

```bash
chmod +x start_web.sh start_all.sh start_line.sh

# เลือกโหมดที่ต้องการ
./start_web.sh       # Web Development
./start_all.sh       # Full Production (Web + LINE)
./start_line.sh      # LINE Integration Only
```

### วิธีที่ 2: รัน Manual

```bash
# Terminal 1 — ฐานข้อมูล
docker compose up -d

# Terminal 2 — Backend
source .venv-robot/bin/activate
cd Back-end
uvicorn api.main:app --host 0.0.0.0 --port 8014 --reload
```

### เข้าใช้งาน

| Service | URL |
|:---|:---|
| 🌐 Frontend | http://localhost:8014 |
| 📊 Admin Panel | http://localhost:8014/admin |
| 📖 API Docs | http://localhost:8014/docs |
| ❤️ Health Check | http://localhost:8014/health |

---

## 11. ทดสอบระบบ

### 11.1 Health Check

```bash
curl http://localhost:8014/health
# ✅ {"status": "healthy", "services": {"mongodb": "healthy", "qdrant": "healthy"}}
```

### 11.2 API Documentation

เปิดเบราว์เซอร์ไปที่ http://localhost:8014/docs → ลองใช้ Swagger UI

### 11.3 Frontend

1. เปิด http://localhost:8014
2. ลองพิมพ์ข้อความ เช่น *"แนะนำที่เที่ยวน่านหน่อย"*
3. ระบบควรตอบกลับพร้อมข้อมูลสถานที่

---

## 12. แก้ไขปัญหาเบื้องต้น

### Docker ไม่ทำงาน

| ปัญหา | วิธีแก้ |
|:---|:---|
| Docker Desktop ไม่แสดง Running | รีสตาร์ท Docker Desktop |
| `Permission denied` (Linux) | รัน `sudo usermod -aG docker $USER` แล้ว Login ใหม่ |
| Port ถูกใช้งานแล้ว | หยุดโปรแกรมที่ใช้ port นั้น หรือแก้ port ใน `docker-compose.yml` |

### Python Errors

| ปัญหา | วิธีแก้ |
|:---|:---|
| `ModuleNotFoundError` | ตรวจสอบว่า activate venv แล้ว + `pip install -r Back-end/requirements.txt` |
| `No module named 'torch'` | ลอง `pip install torch` แยก (บาง distro ต้องลงจาก PyTorch website) |
| `Permission Error` (Linux) | ใช้ `--user` flag: `pip install --user -r Back-end/requirements.txt` |

### API Key Errors

| ปัญหา | วิธีแก้ |
|:---|:---|
| `Invalid API key` (401) | ตรวจสอบว่า Key คัดลอกมาถูกต้อง ไม่มีช่องว่าง |
| `Rate limit exceeded` (429) | เพิ่ม Keys ใน `.env` (คั่นด้วย comma) เพื่อใช้ Key Rotation |
| `DefaultCredentialsError` | ตรวจสอบว่า `GEMINI_API_KEYS` ใน `.env` ไม่ว่างเปล่า |

### Database Connection

| ปัญหา | วิธีแก้ |
|:---|:---|
| ไม่สามารถเชื่อมต่อ MongoDB/Qdrant | รัน `docker ps` ตรวจสอบว่า containers ทำงานอยู่ |
| `Connection refused` | ตรวจสอบ `MONGO_URI` และ `QDRANT_HOST` ใน `.env` ว่าเป็น `localhost` |

---

## 📞 ต้องการความช่วยเหลือ?

- 🐛 **Bug Report:** [GitHub Issues](https://github.com/Mike0165115321/AI-Robot-Guide-/issues)
- 📖 **Quick Start:** [SETUP_QUICKSTART.md](SETUP_QUICKSTART.md)
- 🔑 **API Keys:** [SETUP_API_KEYS.md](SETUP_API_KEYS.md)
