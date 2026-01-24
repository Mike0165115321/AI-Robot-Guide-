# 🤖 Avatar Dev - น้องน่าน Avatar System

> ระบบ Avatar แบบ Modular สำหรับ AI Robot Guide จังหวัดน่าน

## 📁 โครงสร้างโฟลเดอร์

```
avatar/
├── index.js                 # 🚀 Entry point - import จากที่นี่
├── core/                    # ⚙️ ระบบหลัก
│   ├── AvatarController.js  # ตัวควบคุมหลัก
│   ├── EyeTracking.js       # ระบบติดตามตา
│   └── BlinkController.js   # ระบบกระพริบตา
├── moods/                   # 🎭 อารมณ์ต่างๆ
│   ├── BaseMood.js          # Base class
│   ├── NormalMood.js        # 😊 ปกติ
│   ├── SpeakingMood.js      # 🗣️ พูด
│   ├── ThinkingMood.js      # 🤔 คิด
│   ├── ListeningMood.js     # 👂 ฟัง
│   ├── HappyMood.js         # 😄 ดีใจ
│   ├── CuriousMood.js       # 🧐 สงสัย
│   └── SleepyMood.js        # 😴 ง่วง
├── behaviors/               # 🎬 พฤติกรรม Idle
│   └── IdleManager.js       # จัดการ idle behaviors
├── skins/                   # 🎨 Skin (รูปลักษณ์)
│   ├── BaseSkin.js          # Base class
│   └── NanRobot.js          # น้องน่าน (default)
└── config/                  # ⚙️ การตั้งค่า
    └── colors.js            # ค่าสี mood + timing
```

---

## 🚀 วิธีใช้งาน

### 1. Import และ Init

```javascript
// ES6 Module
import NanAvatar from './avatar/index.js';

// เมื่อ DOM พร้อม
NanAvatar.init();
```

### 2. เปลี่ยน Mood

```javascript
NanAvatar.setMood('speaking');  // พูด
NanAvatar.setMood('thinking');  // คิด
NanAvatar.setMood('listening'); // ฟัง
NanAvatar.setMood('happy');     // ดีใจ
NanAvatar.setMood('curious');   // สงสัย
NanAvatar.setMood('sleepy');    // ง่วง
NanAvatar.setMood('normal');    // ปกติ

// หรือใช้ shortcut
NanAvatar.speak();
NanAvatar.think();
NanAvatar.listen();
```

---

## 🎭 การเพิ่ม Mood ใหม่

### ขั้นตอน:

1. **สร้างไฟล์ใน `moods/`**

```javascript
// moods/ExcitedMood.js
import { BaseMood } from './BaseMood.js';

export class ExcitedMood extends BaseMood {
    constructor(controller) {
        super('excited', controller);
    }
    
    enter() {
        super.enter();
        this.setEyeTracking(true);
        this.setBlink(true);
        this.setArmState('arm-idle');
        // เพิ่ม animation พิเศษ
    }
    
    exit() {
        super.exit();
        // cleanup
    }
}
```

2. **เพิ่มสีใน `config/colors.js`**

```javascript
export const MOOD_COLORS = {
    // ... existing moods
    excited: {
        eye: '#ff6b6b',
        accent: '#ff4757',
        glow: 'rgba(255, 107, 107, 0.5)',
        name: 'ตื่นเต้น'
    }
};
```

3. **ลงทะเบียนใน `core/AvatarController.js`**

```javascript
import { ExcitedMood } from '../moods/ExcitedMood.js';

// ใน registerMoods()
this.moods = {
    // ... existing moods
    excited: new ExcitedMood(this)
};
```

4. **เพิ่ม CSS (ถ้าต้องการ)**

```css
.mood-excited .robot-eye {
    /* custom styles */
}
```

---

## 🎨 การเพิ่ม Skin ใหม่

### ขั้นตอน:

1. **สร้างไฟล์ใน `skins/`**

```javascript
// skins/CatRobot.js
import { BaseSkin } from './BaseSkin.js';

export class CatRobotSkin extends BaseSkin {
    constructor() {
        super('CatRobot');
    }

    getInfo() {
        return {
            name: 'แมวน้อย',
            author: 'Your Team',
            version: '1.0.0'
        };
    }

    getCSSVariables() {
        return {
            '--color-robot-white': '#fff5e6',
            '--color-robot-dark': '#4a3728'
        };
    }
}
```

2. **ใช้งาน**

```javascript
import { CatRobotSkin } from './skins/CatRobot.js';

// ใน AvatarController
this.skin = new CatRobotSkin();
this.skin.apply();
```

---

## ⚙️ API Reference

### AvatarController

| Method | คำอธิบาย |
|---|---|
| `init()` | เริ่มต้น avatar |
| `setMood(name)` | เปลี่ยน mood |
| `speak()` | เข้า speaking mode |
| `stopSpeak()` | หยุดพูด |
| `think()` | เข้า thinking mode |
| `listen()` | เข้า listening mode |
| `idle()` | กลับ normal mode |

### EyeTracking

| Method | คำอธิบาย |
|---|---|
| `enable()` | เปิดการติดตามตา |
| `disable()` | ปิดการติดตามตา |
| `track(event)` | คำนวณและเคลื่อนตา |
| `reset()` | รีเซ็ตตาไปกลาง |

### BlinkController

| Method | คำอธิบาย |
|---|---|
| `start()` | เริ่ม loop กระพริบตา |
| `stop()` | หยุด loop |
| `pause()` | หยุดชั่วคราว |
| `resume()` | เล่นต่อ |
| `blinkNow()` | กระพริบทันที |

---

## 📝 หมายเหตุ

- ใช้ **ES6 Modules** ต้อง serve ผ่าน HTTP
- ต้องการ **GSAP** สำหรับ animation ที่ลื่นไหล
- CSS ยังอยู่ใน `avatar_export.html` (จะแยกในอนาคต)

---

## 📄 License

MIT License - AI Robot Guide Project
