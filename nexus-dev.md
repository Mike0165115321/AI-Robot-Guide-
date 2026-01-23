# Workflow: /nexus-dev
Description: ขั้นตอนการพัฒนาฟีเจอร์ใหม่สำหรับ Project Nexus ตามมาตรฐาน Senior Architect

## Steps:
1. **Planning:** วิเคราะห์ความต้องการ แตก Sub-tasks และระบุความเสี่ยง (หยุดรอการอนุมัติจากคุณไมค์)
2. **Architecture:** ออกแบบโครงสร้างตามหลัก SoC และ Modularity
3. **Implementation:** เขียนโค้ด Clean Code (Python/pip)
4. **Optimization:** ตรวจสอบ Latency, Memory และ Token พร้อมอธิบาย Why & How
5. **Finalize:** อัปเดต requirements.txt, บันทึก Log ใน docs/ และอัปเดตเอกสารโปรเจกต์