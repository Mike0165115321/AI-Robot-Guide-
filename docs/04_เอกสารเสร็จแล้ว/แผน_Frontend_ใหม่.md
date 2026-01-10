# 🎯 แผน Frontend ใหม่ - Dual Mode System

> **สถานะ:** กำลังพัฒนา  
> **อัปเดตล่าสุด:** 26 ธันวาคม 2567

---

## 📌 เป้าหมายหลัก

สร้าง Frontend ใหม่ด้วย **Vanilla JS ES6+** ที่รองรับ 2 โหมดการใช้งาน:

| โหมด | Platform | ลักษณะ |
|---|---|---|
| **🤖 Kiosk Mode** | หน้าจอ Kiosk | Avatar เต็มจอ + QR Code |
| **💬 Chat Mode** | มือถือ (Scan QR) | Chat interface เบาๆ |

---

## 🏗️ สถาปัตยกรรม

```
                    ┌─────────────────────┐
                    │     ☁️ Backend      │
                    │   (FastAPI + AI)    │
                    └──────────┬──────────┘
                               │
              ┌────────────────┼────────────────┐
              ▼                ▼                ▼
    ┌─────────────────┐ ┌─────────────┐ ┌─────────────┐
    │  🤖 Kiosk Mode  │ │ 💬 Chat Mode│ │ 🔧 Admin    │
    │  (kiosk.html)   │ │ (chat.html) │ │ (admin.html)│
    │                 │ │             │ │             │
    │  Avatar + QR    │ │ Mobile Chat │ │ จัดการข้อมูล│
    └─────────────────┘ └─────────────┘ └─────────────┘
```

---

## 📁 โครงสร้างโฟลเดอร์

```
frontend/
├── kiosk.html          # 🤖 หน้า Kiosk (Avatar + QR)
├── chat.html           # 💬 หน้า Chat สำหรับมือถือ
├── admin.html          # 🔧 หน้า Admin
├── css/
│   ├── base.css        # ✅ เสร็จแล้ว
│   ├── layout.css      # ✅ เสร็จแล้ว
│   └── components.css  # ✅ เสร็จแล้ว
├── js/
│   ├── app.js          # ✅ Entry point
│   ├── config.js       # ✅ Configuration
│   ├── api/client.js   # ✅ API Client
│   ├── components/
│   │   ├── ChatBox.js
│   │   ├── QRCode.js
│   │   └── VoiceButton.js
│   ├── services/
│   │   ├── AudioService.js
│   │   └── WebSocket.js
│   └── utils/
│       └── dom.js      # ✅ เสร็จแล้ว
└── avatar/             # ✅ เสร็จแล้ว (แยก folder)
```

---

## 📋 แผนงาน

### ✅ เสร็จแล้ว
- [x] Avatar System (7 moods, 9 skins)
- [x] Avatar Modular Structure
- [x] CSS Foundation (base, layout, components)
- [x] JS Foundation (app.js, config, dom utils, API client)

### 🔲 รอดำเนินการ

#### Phase 2: Kiosk Mode
- [ ] สร้าง `kiosk.html` (Avatar เต็มจอ)
- [ ] รวม Avatar เข้ากับ kiosk
- [ ] เพิ่ม QR Code generator
- [ ] เชื่อม Voice (STT/TTS)

#### Phase 3: Chat Mode
- [ ] สร้าง `chat.html` (Mobile-first)
- [ ] ออกแบบ UI เรียบง่าย
- [ ] รองรับ Session จาก QR Scan

#### Phase 4: Backend Integration
- [ ] เชื่อม Chat API
- [ ] เชื่อม WebSocket (realtime)
- [ ] เชื่อม TTS/STT

#### Phase 5: Admin Panel
- [ ] สร้าง `admin.html`
- [ ] CRUD สถานที่ท่องเที่ยว
- [ ] Analytics Dashboard

---

## 🎭 รายละเอียด Kiosk Mode

**หน้าตาที่ต้องการ:**
```
┌─────────────────────────────────────────────┐
│                                             │
│                  🤖                         │
│              (น้องน่าน)                     │
│                                             │
│    "สวัสดีค่ะ! ยินดีให้บริการค่ะ"          │
│                                             │
│                                             │
│                        ┌──────┐             │
│                        │ QR   │   📱        │
│                        │ Code │  สแกนแชท    │
│                        └──────┘             │
└─────────────────────────────────────────────┘
```

---

## 💬 รายละเอียด Chat Mode

**สำหรับคนสแกน QR จากมือถือ:**
- ไม่มี Avatar (เพื่อความเร็ว)
- Chat interface เรียบง่าย
- รองรับพิมพ์ข้อความ
- อาจมีปุ่มไมค์ (optional)

---

## 🔗 QR Code System

1. Kiosk แสดง QR Code → URL: `https://domain.com/chat?session=xxx`
2. ผู้ใช้สแกน → เปิดหน้า Chat บนมือถือ
3. Session เชื่อมกับ Kiosk → Avatar แสดง reaction

---

## 📝 หมายเหตุ

- ใช้ **Vanilla JS ES6+** เท่านั้น (ไม่ใช้ React/Vue)
- Avatar แยก folder `avatar/` อิสระ
- Backend ใช้ตัวเดิม (FastAPI)
