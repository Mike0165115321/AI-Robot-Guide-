# 🔐 แผนระบบ Admin

> **สถานะ:** รอตรวจสอบ  
> **อัปเดต:** 26 ธันวาคม 2567

---

## 🎯 เป้าหมาย

สร้างระบบ Admin ที่:
- **ปลอดภัย** - มีการ Login, รหัสผ่านเข้ารหัส
- **ใช้งานง่าย** - UI สะอาด
- **ครบถ้วน** - จัดการข้อมูลทั้งหมดได้

---

## 🔑 ระบบความปลอดภัย

### Authentication Flow

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Login     │ --> │   Verify    │ --> │   Admin     │
│   Page      │     │  Password   │     │  Dashboard  │
└─────────────┘     └─────────────┘     └─────────────┘
                          │
                    ❌ ถ้าผิด
                          │
                    ┌─────────────┐
                    │ Error Msg   │
                    └─────────────┘
```

### Security Features

| ฟีเจอร์ | รายละเอียด |
|---|---|
| **Password Hashing** | ใช้ bcrypt เข้ารหัส |
| **JWT Token** | ใช้ JWT สำหรับ session |
| **Token Expiry** | หมดอายุใน 24 ชม. |
| **Rate Limiting** | จำกัด 5 ครั้ง/นาที |
| **HTTPS** | บังคับใช้ (production) |

---

## 👤 ระบบ User

### ระดับ (Roles)

| Role | สิทธิ์ |
|---|---|
| **Super Admin** | ทำได้ทุกอย่าง + จัดการ User |
| **Admin** | CRUD สถานที่, ดู Analytics |
| **Viewer** | ดูข้อมูลได้อย่างเดียว |

### ตาราง Users (MongoDB)

```javascript
{
    _id: ObjectId,
    username: String,          // unique
    password: String,          // hashed
    role: "super_admin" | "admin" | "viewer",
    email: String,
    createdAt: Date,
    lastLogin: Date,
    isActive: Boolean
}
```

---

## 📄 หน้า Admin

### 1. Login Page (`/admin/login.html`)

```
┌─────────────────────────────────────────────────────────┐
│                                                         │
│                    🔐 Admin Login                       │
│                                                         │
│           ┌──────────────────────────────┐              │
│           │ 👤 Username                  │              │
│           └──────────────────────────────┘              │
│           ┌──────────────────────────────┐              │
│           │ 🔒 Password                  │              │
│           └──────────────────────────────┘              │
│                                                         │
│           ┌──────────────────────────────┐              │
│           │          เข้าสู่ระบบ         │              │
│           └──────────────────────────────┘              │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### 2. Dashboard (`/admin/index.html`)

```
┌─────────────────────────────────────────────────────────┐
│  Admin Panel                    [👤 ชื่อ] [🚪 ออก]     │
├────────────┬────────────────────────────────────────────┤
│            │                                            │
│  📊 สรุป   │   📊 DASHBOARD                            │
│  📍 สถานที่│                                            │
│  💬 แชท   │   ┌────────┐ ┌────────┐ ┌────────┐         │
│  👥 ผู้ใช้ │   │ 150    │ │ 45     │ │ 12     │         │
│  ⚙️ ตั้งค่า│   │ สถานที่│ │ วันนี้ │ │ ออนไลน์│         │
│            │   └────────┘ └────────┘ └────────┘         │
│            │                                            │
│            │   📈 กราฟการใช้งาน                         │
│            │   [.................................]      │
│            │                                            │
└────────────┴────────────────────────────────────────────┘
```

### 3. Locations Management (`/admin/locations.html`)

```
┌─────────────────────────────────────────────────────────┐
│  📍 จัดการสถานที่           [+ เพิ่มใหม่]              │
├─────────────────────────────────────────────────────────┤
│  🔍 [ค้นหา...]  [หมวดหมู่ ▼]                           │
├─────────────────────────────────────────────────────────┤
│  │ ชื่อ          │ หมวดหมู่    │ อำเภอ    │ Actions │  │
│  ├───────────────┼─────────────┼──────────┼─────────┤  │
│  │ วัดภูมินทร์   │ วัด        │ เมือง    │ ✏️ 🗑️  │  │
│  │ ดอยเสมอดาว   │ ธรรมชาติ    │ นาน้อย   │ ✏️ 🗑️  │  │
│  │ ถนนคนเดิน    │ ช้อปปิ้ง    │ เมือง    │ ✏️ 🗑️  │  │
└─────────────────────────────────────────────────────────┘
```

### 4. User Management (`/admin/users.html`) - Super Admin Only

```
┌─────────────────────────────────────────────────────────┐
│  👥 จัดการผู้ใช้              [+ เพิ่ม User]            │
├─────────────────────────────────────────────────────────┤
│  │ Username    │ Role       │ สถานะ    │ Actions │     │
│  ├─────────────┼────────────┼──────────┼─────────┤     │
│  │ admin       │ Super Admin│ ✅ Active│ ✏️      │     │
│  │ editor1     │ Admin      │ ✅ Active│ ✏️ 🗑️  │     │
│  │ viewer1     │ Viewer     │ ❌ Inactive│ ✏️ 🗑️ │     │
└─────────────────────────────────────────────────────────┘
```

---

## 🔧 Backend API

### Auth Endpoints

| Method | Endpoint | คำอธิบาย |
|---|---|---|
| POST | `/api/auth/login` | Login |
| POST | `/api/auth/logout` | Logout |
| GET | `/api/auth/me` | ข้อมูล User ปัจจุบัน |
| POST | `/api/auth/change-password` | เปลี่ยนรหัส |

### Admin Endpoints (ต้อง Auth)

| Method | Endpoint | คำอธิบาย |
|---|---|---|
| GET | `/api/admin/dashboard` | Dashboard stats |
| GET | `/api/admin/locations` | รายการสถานที่ |
| POST | `/api/admin/locations` | เพิ่มสถานที่ |
| PUT | `/api/admin/locations/:id` | แก้ไข |
| DELETE | `/api/admin/locations/:id` | ลบ |
| GET | `/api/admin/users` | รายการ User (Super Admin) |

---

## 📁 โครงสร้างไฟล์

```
frontend/admin/
├── login.html          # หน้า Login
├── index.html          # Dashboard
├── locations.html      # จัดการสถานที่
├── chats.html          # ดูประวัติแชท
├── users.html          # จัดการ User
├── settings.html       # ตั้งค่า
├── css/
│   └── admin.css       # Admin-specific styles
└── js/
    ├── auth.js         # Authentication logic
    ├── dashboard.js
    ├── locations.js
    └── users.js
```

---

## 🔐 Default Admin Account

| Username | Password | Role |
|---|---|---|
| admin | *(ตั้งเอง)* | Super Admin |

> ⚠️ **ต้องเปลี่ยนรหัสทันทีหลังติดตั้ง!**

---

## ❓ คำถามสำหรับพี่

1. ต้องการ 2FA (Two-Factor Authentication) ไหม?
2. อยากให้ล็อก User ถ้าใส่รหัสผิดเกิน X ครั้งไหม?
3. ประวัติ Activity Log ต้องการแค่ไหน?
4. อยากให้ Admin หลายคนทำงานพร้อมกันได้ไหม?
