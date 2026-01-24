# Voice Commands Architecture - ระบบสั่งงานด้วยเสียง

## 🎯 Vision

ระบบที่ฟังอยู่เบื้องหลังตลอดเวลา และสามารถ **เข้าใจคำสั่งแล้วดำเนินการทันที** โดยไม่ต้องรอ user กดปุ่ม เหมาะสำหรับ:
- 🖥️ Kiosk หน้าจอสาธารณะ
- 🤖 หุ่นยนต์ต้อนรับ (Service Robot)
- 🏠 Smart Display ในบ้าน

---

## 📋 Command Categories

### 1. Wake Word (Activation)
```
"สวัสดีน้องน่าน" / "น้องน่าน"
→ ทักทายกลับ + เข้า STT mode รอคำถาม
```

### 2. Music Commands (เปิดเพลง)
```
"น้องน่านเปิดเพลง"
→ "ค่ะ บอกชื่อเพลงได้เลยค่ะ" + แสดง UI เลือกเพลง

"น้องน่านเปิดเพลงคำเมือง"
→ ค้นหาเพลง "คำเมือง" เลย + เล่นทันที
```

### 3. News Commands (ข่าวสาร)
```
"น้องน่านข่าววันนี้"
→ ดึงข่าวล่าสุด + TTS อ่านให้ฟัง

"น้องน่านข่าวจราจร"
→ ดึงข่าวจราจร/ถนน + แสดงแผนที่
```

### 4. Navigation Commands (นำทาง)
```
"น้องน่านพาไปวัดภูมินทร์"
→ เปิดแผนที่ + แสดงเส้นทาง

"น้องน่านใกล้ๆมีอะไรบ้าง"
→ แนะนำสถานที่ใกล้เคียง
```

### 5. Weather & Info (สภาพอากาศ)
```
"น้องน่านวันนี้อากาศเป็นยังไง"
→ ดึงข้อมูลสภาพอากาศ + TTS

"น้องน่าน PM2.5 เท่าไหร่"
→ ดึงค่าคุณภาพอากาศ
```

### 6. System Commands (ระบบ)
```
"น้องน่านหยุด" / "หยุด"
→ หยุด TTS / หยุดเพลง

"น้องน่านพูดซ้ำ"
→ TTS ซ้ำคำตอบล่าสุด
```

---

## 🏗️ Technical Architecture

### Current (Phase 1 - Implemented)
```
[WakeWordService] → ฟัง "น้องน่าน" → [TTS ทักทาย] → [STT Mode]
```

### Target (Phase 2 - Planned)
```
[VoiceCommandService]
    │
    ├─→ [Intent Classifier] ← ใช้ LLM หรือ Keyword Matching
    │        │
    │        ├─→ WAKE_WORD → TTS + STT Mode
    │        ├─→ MUSIC → MusicHandler
    │        ├─→ NEWS → NewsHandler
    │        ├─→ NAVIGATION → NavigationHandler
    │        ├─→ WEATHER → WeatherHandler
    │        └─→ SYSTEM → SystemHandler
    │
    └─→ [Action Executor] → ดำเนินการตาม intent
```

---

## 📁 Proposed File Structure

```
frontend/js/services/
├── WakeWordService.js       (ปัจจุบัน - เปลี่ยนชื่อ)
├── VoiceCommandService.js   (🆕 Main Voice Service)
├── VoiceIntentClassifier.js (🆕 Intent Detection)
└── VoiceActionHandlers.js   (🆕 Action Executors)
```

---

## 🔧 Implementation Phases

### Phase 1: Wake Word (✅ Done)
- [x] WakeWordService.js
- [x] Basic wake word detection
- [x] TTS greeting + STT mode

### Phase 2: Command Patterns (Next)
- [ ] เพิ่ม keyword patterns สำหรับ Music, News, Navigation
- [ ] Intent classification (keyword-based)
- [ ] Direct action execution

### Phase 3: LLM Intent (Future)
- [ ] ใช้ LLM (local หรือ API) วิเคราะห์ intent
- [ ] รองรับคำสั่งที่ยืดหยุ่นกว่า
- [ ] Context-aware commands

### Phase 4: Robot Integration
- [ ] เชื่อมต่อกับ ROS 2 สำหรับหุ่นยนต์
- [ ] Hardware button triggers
- [ ] LED/Display feedback

---

## 💡 Keyword Pattern Examples

```javascript
const COMMAND_PATTERNS = {
    MUSIC: [
        /น้องน่าน.*เปิดเพลง/,
        /เปิดเพลง.*น้องน่าน/,
        /ฟังเพลง/
    ],
    NEWS: [
        /น้องน่าน.*ข่าว/,
        /ข่าววันนี้/,
        /มีข่าวอะไร/
    ],
    NAVIGATION: [
        /น้องน่าน.*พาไป/,
        /น้องน่าน.*นำทาง/,
        /ใกล้ๆมีอะไร/
    ],
    WEATHER: [
        /น้องน่าน.*อากาศ/,
        /วันนี้ร้อนไหม/,
        /PM2.5/i
    ],
    STOP: [
        /หยุด/,
        /พอก่อน/,
        /stop/i
    ]
};
```

---

## ⚠️ Considerations

### Privacy & User Experience
- แสดง indicator ชัดเจนว่ากำลังฟังอยู่
- ให้ user ปิด wake word ได้
- ไม่เก็บเสียงที่ไม่ match command

### Performance
- Keyword matching ก่อน (เร็ว, offline)
- LLM fallback (แม่นกว่า, ช้ากว่า)

### Robot-Specific
- รองรับ hardware button เป็น alternative trigger
- LED indicator แสดงสถานะ (ฟัง/ประมวลผล/พูด)
- Emergency stop command

---

## 🚀 Next Steps

1. **Phase 2: Command Patterns**
   - เพิ่ม COMMAND_PATTERNS ใน WakeWordService
   - สร้าง VoiceIntentClassifier.js
   - Connect กับ existing handlers (Music, News, Navigation)

2. **Testing**
   - ทดสอบแต่ละ command pattern
   - วัดความแม่นยำ

3. **UI/UX**
   - แสดง feedback ว่าเข้าใจคำสั่งอะไร
   - Visual cues สำหรับ action ที่กำลังทำ
