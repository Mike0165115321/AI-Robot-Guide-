# 🐳 คู่มือการติดตั้งและใช้งาน Docker (Docker Installation Guide)

เพื่อให้การรันโปรเจค **AI Robot Guide จังหวัดน่าน** เป็นไปอย่างราบรื่น เราแนะนำให้รัน Database (MongoDB, Qdrant) และ Service เสริมผ่าน **Docker** ซึ่งช่วยลดปัญหาสภาพแวดล้อมไม่ตรงกัน

---

## 1. การติดตั้ง Docker

### สำหรับ Windows / Mac (ง่ายที่สุด)
แนะนำให้ติดตั้ง **Docker Desktop**
*   [Download Docker Desktop](https://www.docker.com/products/docker-desktop/)
*   ติดตั้งเสร็จแล้ว ให้เปิดโปรแกรมทิ้งไว้

### สำหรับ Linux (แนะนำสำหรับ Production / Dev)
บน Linux เราแนะนำให้ลง **Docker Engine + Docker Compose V2 (Go Version)** จาก Official Repository เพื่อความเสถียร

> **ทำไมต้องลงแบบนี้?** Docker ที่มากับ Package Manager ของ Distro (เช่น `apt install docker.io`) มักจะเก่าและอาจมีปัญหากับ Compose V2

**ขั้นตอนการติดตั้งบน Ubuntu/Debian:**

```bash
# 1. ลบเวอร์ชั่นเก่า (ถ้ามี)
sudo apt-get remove docker docker-engine docker.io containerd runc

# 2. ติดตั้ง Dependencies
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg

# 3. เพิ่ม Docker Official GPG Key
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# 4. เพิ่ม Repository (Stable)
echo \
  "deb [arch=\"$(dpkg --print-architecture)\" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  \"$(. /etc/os-release && echo \"$VERSION_CODENAME\")\" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 5. ติดตั้ง Docker Engine & Compose V2
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# 6. ตรวจสอบเวอร์ชั่น (ต้องเป็น Docker Compose version v2.x.x)
docker compose version
```

---

## 2. การเริ่มต้นใช้งาน (Start Services)

หลังจาก Clone โปรเจคมาแล้ว ให้ใช้คำสั่งต่อไปนี้เพื่อเปิด Database ทั้งหมด:

```bash
# รันคำสั่งนี้ที่ root ของโปรเจค (ที่มีไฟล์ docker-compose.yml)
docker compose up -d

# หมายเหตุ: หากใช้ docker-compose เวอร์ชั่นเก่า (V1) ให้ใช้คำสั่ง: 
# docker-compose up -d
```

### ตรวจสอบสถานะ
พิมพ์คำสั่ง:
```bash
docker ps
```
คุณควรจะเห็น Container ต่อไปนี้สถานะ `Up`:
1.  **mongodb_db** (Database หลัก)
2.  **qdrant_db** (Vector Database)
3.  **redis_mq** (Redis Message Queue)

---

## 3. การหยุดและลบ Container
หากต้องการปิดระบบ:
```bash
# ปิด Container
docker compose stop

# ปิดและลบ Container ออกไปเลย (ข้อมูลไม่หาย เพราะเรา Mount Volume ไว้)
docker compose down
```

---
*เอกสารนี้เป็นส่วนหนึ่งของโครงการ AI Robot Guide จังหวัดน่าน*
