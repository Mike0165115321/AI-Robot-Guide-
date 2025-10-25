# 🤖 AI Robot Guide จังหวัดน่าน

โปรเจกต์ AI Guide แนะนำสถานที่ท่องเที่ยวและข้อมูลสำคัญของจังหวัดน่าน พัฒนาด้วยสถาปัตยกรรม RAG (Retrieval-Augmented Generation) เพื่อให้คำตอบที่ถูกต้องและเป็นประโยชน์จากฐานข้อมูลของเราเอง

## ✨ คุณสมบัติหลัก (Features)

* **Chat Interface:** หน้าแชทสำหรับพูดคุย สอบถามข้อมูลกับ AI (รองรับ Text-to-Speech และ Speech-to-Text)
* **Avatar Interface:** หน้าหุ่นยนต์ Avatar ที่สามารถโต้ตอบและแสดงท่าทางประกอบ
* **Admin Panel:** หน้าสำหรับแอดมินเพื่อจัดการฐานข้อมูล (เพิ่ม/ลบข้อมูล)
* **RAG Pipeline:** เชื่อมต่อกับ Vector Database (Qdrant) และ Document Database (MongoDB) เพื่อดึงข้อมูลที่เกี่ยวข้องมาประกอบการตอบ
* **Multi-LLM:** รองรับการสลับใช้งานระหว่าง Gemini และ Groq

---

## 🛠️ เทคโนโลยีที่ใช้ (Tech Stack)

* **Frontend:** HTML5, CSS3, JavaScript (Vanilla JS)
* **Backend:** FastAPI (Python)
* **Database:**
    * MongoDB (สำหรับเก็บข้อมูลหลัก)
    * Qdrant (สำหรับ Vector Search)
* **AI / LLM:** Gemini, Groq (Llama 3.x), LangChain/LlamaIndex (สำหรับ RAG)
* **Speech:** Whisper (STT), gTTS (TTS)

---

## 🚀 วิธีการติดตั้งและรันโปรเจกต์ (Setup & Run)

นี่คือขั้นตอนการติดตั้งโปรเจกต์ทั้งหมดตั้งแต่ต้นจนจบ

### 1. ติดตั้งโปรแกรมที่จำเป็น (Prerequisites)

ก่อนเริ่ม ตรวจสอบว่าเครื่องของเรามีโปรแกรมเหล่านี้ติดตั้งอยู่:
* [Python](https://www.python.org/) (แนะนำเวอร์ชัน 3.10 ขึ้นไป)
* [Docker Desktop](https://www.docker.com/products/docker-desktop/) (สำหรับรัน MongoDB และ Qdrant)

### 2. รันฐานข้อมูล (Run Databases)

วิธีที่ง่ายที่สุดในการรันฐานข้อมูล (MongoDB และ Qdrant) คือการใช้ Docker Compose

1.  ตรวจสอบว่า Docker Desktop กำลังทำงานอยู่ (Running)
2.  เปิด Terminal ที่โฟลเดอร์หลักของโปรเจกต์ (ที่มีไฟล์ `docker-compose.yml` อยู่)
3.  รันคำสั่ง:

```bash
# คำสั่งนี้จะสร้างและรัน MongoDB (ที่ port 27017)
# และ Qdrant (ที่ port 6333) ให้เราอัตโนมัติ
docker-compose up -d
(ถ้าต้องการหยุดฐานข้อมูล ให้ใช้ docker-compose down)

3. โคลนโปรเจกต์ (Clone)
(ข้ามขั้นตอนนี้ไป ถ้าเราได้ไฟล์โปรเจกต์มาแล้ว)

```
```
git clone [https://github.com/Mike0165115321/AI-Robot-Guide-.git](https://github.com/Mike0165115321/AI-Robot-Guide-.git)
cd AI-Robot-Guide-
```
4. ตั้งค่า Python Virtual Environment (venv)
```
# สร้าง venv (ใช้ชื่อ .venv เพื่อความเป็นมาตรฐาน)
python -m venv .venv
```
# Activate venv
# บน macOS/Linux
source .venv/bin/activate
5. ติดตั้ง Dependencies
```
# ตรวจสอบว่าอยู่ใน venv แล้ว
# (สังเกตว่ามี (.venv) นำหน้า prompt)
pip install -r Back-end/requirements.txt

```
6. ‼️ ตั้งค่า Environment (สำคัญที่สุด!)
โปรเจกต์นี้ต้องใช้ API Keys หลายตัวในการทำงาน
```
ไปที่โฟลเดอร์ Back-end/
สร้างไฟล์ใหม่ ชื่อว่า .env
เปิดไฟล์ Back-end/เอาคีย์มาใส่ สร้าง .env.txt เพื่อดู "คู่มือ" ว่าต้องใช้ Key อะไรบ้าง และไปหามาจากที่ไหน
คัดลอก Key ทั้งหมดมาวางในไฟล์ .env ที่เพิ่งสร้าง
สำคัญ: ค่า MONGO_URI และ QDRANT_HOST ในไฟล์ .env (หรือใน config.py default) ควรเป็น localhost หรือ 127.0.0.1 เพื่อให้มันเชื่อมต่อกับ Docker ที่รันในเครื่องเราได้
ตัวอย่างโครงสร้างไฟล์ .env ที่ถูกต้อง:
Plaintext
MONGO_URI="mongodb://localhost:27017/"
QDRANT_HOST="localhost"
QDRANT_PORT=6333

GEMINI_API_KEYS="AIzaSy...key1,AIzaSy...key2"
GROQ_API_KEYS="gsk_...key1"
# ... (และ Keys อื่นๆ) ...
```
7. สร้างฐานข้อมูล Vector (Build Vectors)
ก่อนรันเซิร์ฟเวอร์ครั้งแรก เราต้องนำข้อมูล .jsonl ไปประมวลผลและเก็บใน Qdrant ก่อน

```
# ตรวจสอบว่ายังอยู่ใน venv
# รันสคริปต์เพื่อสร้าง Vector Database
python Back-end/scripts/build_vectors.py
(หากมีข้อผิดพลาด ตรวจสอบว่า Docker (MongoDB/Qdrant) กำลังทำงานอยู่หรือไม่)
```
8. รันเซิร์ฟเวอร์ (Run Backend)
เมื่อทุกอย่างพร้อม ให้รัน FastAPI Server:

```
# รันเซิร์ฟเวอร์ (จากโฟลเดอร์หลัก)
uvicorn Back-end.api.main:app --host 127.0.0.1 --port 9090 --reload
--reload หมายความว่าถ้าเราแก้โค้ด Backend เซิร์ฟเวอร์จะรีสตาร์ทตัวเองอัตโนมัติ
```
