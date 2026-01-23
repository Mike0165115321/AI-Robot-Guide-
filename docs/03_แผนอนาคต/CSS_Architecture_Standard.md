# CSS Architecture Standard: "Nong Nan" Design System

## 1. Core Philosophy (ปรัชญาหลัก)
เราใช้สถาปัตยกรรม **"Design Tokens Layering"** ร่วมกับโครงสร้างโฟลเดอร์แบบ **ITCSS (Inverted Triangle CSS)** เพื่อแก้ปัญหา CSS บวม, ทับซ้อน, และแก้ยาก

### กฎเหล็ก 3 ข้อ
1.  **ห้าม Hardcode สี:** ห้ามใช้ `#Hex` หรือ `rgb()` ในไฟล์ Component เด็ดขาด ต้องใช้ตัวแปร `var(--ชื่อ)` เท่านั้น
2.  **แยก "สีดิบ" ออกจาก "หน้าที่":** อย่าใช้ `teal-500` เป็นสีพื้นหลังปุ่มตรงๆ ให้ตั้งตัวแปรใหม่เป็น `btn-primary-bg` รับค่าจาก `teal-500` อีกที
3.  **Component ต้องเป็นอิสระ:** ไฟล์ `.css` ของ Component หนึ่ง ห้ามไปยุ่งกับ Component อื่น

---

## 2. Folder Structure (โครงสร้างโฟลเดอร์)
เราจะย้ายไฟล์จาก `frontend/css/*` เดิม มาสู่โครงสร้างใหม่ที่ `frontend/css/` ดังนี้:

```text
frontend/css/
├── main.css                (📄 Entry Point: ไฟล์เดียวที่ HTML โหลด)
│
├── 1-tokens/               (🎨 DNA ของเว็บ)
│   ├── _primitives.css     <-- รวมสีดิบ (Raw Palette) ทั้งหมด
│   ├── _typography.css     <-- ฟอนต์, ขนาดตัวหนังสือ
│   └── _spacing.css        <-- ระยะห่าง (Margins/Paddings/Radius)
│
├── 2-themes/               (🎭 ตัวกำหนดความสวยงาม)
│   ├── _theme-base.css     <-- Mapping ตัวแปร Semantics (กลาง)
│   ├── _theme-neon.css     <-- ค่าเฉพาะธีม Neon
│   └── _theme-nongnan.css  <-- ค่าเฉพาะธีม Nong Nan
│
├── 3-base/                 (🏗️ โครงสร้างพื้นฐาน)
│   ├── _reset.css          <-- ล้างค่า Browser (Normalize/Reset)
│   └── _global.css         <-- ค่าเริ่มต้น body, html
│
├── 4-layout/               (📐 การจัดหน้า)
│   ├── _grid.css           <-- ระบบ Grid, Flex Utilities
│   └── _containers.css     <-- กรอบเนื้อหา (Responsive Containers)
│
└── 5-components/           (🧩 ชิ้นส่วนต่างๆ)
    ├── _navbar.css
    ├── _buttons.css
    ├── _cards.css
    ├── _chat.css
    └── _avatar.css
```

---

## 3. Token Strategy (ยุทธศาสตร์ตัวแปร)

### Layer 1: Primitives (สีดิบ)
*ไฟล์: `1-tokens/_primitives.css`*
คือนิยาม "จานสี" ทั้งหมดที่มีให้ใช้
```css
:root {
  --ref-teal-900: #0A4F60;
  --ref-teal-500: #14b8a6;
  --ref-blue-500: #3b82f6;
  --ref-gray-100: #f3f4f6;
}
```

### Layer 2: Semantics (หน้าที่)
*ไฟล์: `2-themes/_theme-base.css`*
คือการบอกว่า "เอาสีดิบไปทำอะไร" สิ่งนี้จะเปลี่ยนไปตามธีม
```css
/* ☀️ Theme: Nong Nan */
[data-theme="nongnan"] {
  --bg-page: var(--ref-gray-100);
  --text-main: var(--ref-teal-900);
  --btn-primary-bg: var(--ref-teal-900);
}

/* 🌙 Theme: Neon */
[data-theme="neon"] {
  --bg-page: var(--ref-slate-900);
  --text-main: var(--ref-white);
  --btn-primary-bg: var(--ref-blue-500);
}
```

---

## 4. Workflow การเพิ่มธีมใหม่
1.  สร้างไฟล์ `2-themes/_theme-newname.css`
2.  ประกาศ `[data-theme="newname"]`
3.  Override ค่า Semantics (Layer 2) ตามต้องการ (ไม่ต้องแก้ Primitives)

## 5. การเรียกใช้ใน JS
ใช้ `ThemeManager` ในการสลับ class หรือ attribute ที่ `<html>`
```javascript
document.documentElement.setAttribute('data-theme', 'nongnan');
```
