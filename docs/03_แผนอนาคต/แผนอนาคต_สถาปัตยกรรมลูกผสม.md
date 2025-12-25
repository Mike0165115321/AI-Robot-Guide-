# 🚀 แผนพัฒนาระยะยาว: Hybrid Architecture (Go + C++)

> **สถานะ:** 📌 บันทึกไว้สำหรับอนาคต  
> **วันที่สร้าง:** 2025-12-13  
> **เป้าหมาย:** นำไปใช้หลังผ่านรอบระดับจังหวัด เพื่อเตรียมแข่งระดับประเทศ

---

## 📋 สรุปแผน

เปลี่ยนจาก Python-only architecture → Hybrid Architecture:

```
┌─────────────────────────────────────────────────────┐
│                     Frontend (JS)                    │
└─────────────────────┬───────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────┐
│              GO 1.22 API Gateway (ใหม่)              │
│  • High-concurrency HTTP/WebSocket                   │
│  • Load balancing                                    │
│  • Auth/Rate limiting                                │
└────────┬────────────────────────────┬───────────────┘
         │ gRPC                       │ gRPC
┌────────▼────────┐          ┌────────▼────────────────┐
│ Python 3.12     │          │  C++17 Audio Engine     │
│ (FastAPI เดิม)  │          │  • whisper.cpp STT      │
│ • RAG Pipeline  │◄────────►│  • Audio preprocessing  │
│ • LLM calls     │          │  • Wake word detection  │
└─────────────────┘          └──────────────────────────┘
```

## 🛠️ Tech Stack ที่เลือก

| Component | Version | เหตุผล |
|-----------|---------|--------|
| **Go** | 1.22.x (LTS) | เสถียร, รองรับ generics |
| **C++** | C++17 | รองรับทุก compiler, พอดีกับ whisper.cpp |
| **Python** | 3.12.x | คงเดิม |
| **gRPC** | 1.60+ | สื่อสารระหว่าง services |

## ⏱️ Timeline ประมาณ

| Phase | งาน | ระยะเวลา |
|-------|-----|----------|
| 1 | Setup + Protobuf | 2-3 วัน |
| 2 | Go Gateway | 1 สัปดาห์ |
| 3 | C++ Audio Engine | 1-2 สัปดาห์ |
| 4 | Python Modifications | 2-3 วัน |
| 5 | Integration & Testing | 1 สัปดาห์ |
| **รวม** | | **~4-5 สัปดาห์** |

## 📂 โครงสร้างโฟลเดอร์ที่จะเพิ่ม

```
AI Robot Guide จังหวัดน่าน/
├── go-gateway/           # NEW: Go API Gateway
│   ├── cmd/server/
│   ├── internal/
│   └── proto/
├── cpp-audio-engine/     # NEW: C++ Audio Engine
│   ├── src/
│   ├── include/
│   └── third_party/whisper.cpp/
├── Back-end/             # EXISTING: เพิ่ม gRPC server
│   └── grpc/
└── frontend/             # EXISTING: เปลี่ยน endpoint
```

## ✅ เมื่อพร้อมกลับมา ให้ทำ:

1. บอกผมว่า "เริ่มแผน Hybrid Architecture"
2. ผมจะ setup โฟลเดอร์และ scaffold code ให้
3. เริ่มจาก Go Gateway ก่อน (ง่ายกว่า)

---

**🍀 ขอให้โชคดีในการแข่งขันระดับจังหวัดครับ!**  
**เจอกันตอนเตรียมสู้ระดับประเทศ! 🏆**
