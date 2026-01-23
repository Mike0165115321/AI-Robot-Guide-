# บันทึกการพัฒนา Avatar Lip Sync (Simulated Optimization)

เอกสารนี้รวบรวมประวัติการพัฒนาและสถานะล่าสุดของระบบ **Lip Sync** ของ Avatar น้องน่าน

---

## 1. เป้าหมายเดิม (Real-Time Lip Sync)
ความตั้งใจแรกคือการทำให้ปากของ Avatar ขยับตามค่าความดัง (RMS Volume) ของเสียง TTS แบบ Real-time

### วิธีการเดิม (Attempted Approach)
*   **Technology:** `AudioContext` + `AnalyserNode` ใน Browser
*   **Flow:** วิเคราะห์ Waveform -> คำนวณ RMS -> ส่งค่า Volume ไปที่ Avatar Iframe (60fps)
*   **ผลลัพธ์:** ทำงานได้ แต่เจอปัญหาใหญ่ด้าน Performance

### ปัญหาที่พบ (Critial Issues)
1.  **Backend Spam:** การใช้ `avatarService.send` ใน Loop 60fps ทำให้มีการยิง WebSocket ไปที่ Server รัวๆ จน Server ล่ม (Empty Query Error)
2.  **Browser Complexity:** ติดปัญหา Web Audio API Policy (ต้องรอ User Click), CORS issue กับ Blob URL
3.  **Low Volume:** เสียงสังเคราะห์ (TTS) บางช่วงเบาเกินไป ทำให้ Node ตัดค่าทิ้ง ปากไม่ขยับ (Dead zone)

---

## 2. วิธีการแก้ปัญหาปัจจุบัน (Current Solution: Simulated Animation)
เราตัดสินใจ **Pivot** มาใช้วิธีการจำลอง (Simulation) แทนการวิเคราะห์เสียงจริง เพื่อความเสถียรและประสิทธิภาพสูงสุด

### หลักการทำงาน (Simulated Logic)
แทนที่จะเสีย CPU ไปวิเคราะห์คลื่นเสียง เราใช้สมการคณิตศาสตร์สร้างความเคลื่อนไหวที่เป็นธรรมชาติแทน:

```javascript
// Lightweight Loop
const update = () => {
    // Math.sin(time) = จังหวะการหายใจ/จังหวะหลัก (Base Rhythm)
    // Math.random() = ความไม่แน่นอนของพยางค์ (Randomness)
    const openAmount = ((Math.sin(Date.now() / 150) + 1) * 0.3) + (Math.random() * 0.4);
    
    // ส่งข้อมูลเข้า Iframe โดยตรง (ไม่ผ่าน Server)
    this.sendLocalVisual({ type: 'voiceData', volume: openAmount });
}
```

### ข้อดีของวิธีใหม่
1.  **Zero Overhead:** ไม่กิน CPU ในการ Process Audio เลย
2.  **Direct Local Update:** ใช้ `wrapper.contentWindow.postMessage` สื่อสารกับ Avatar โดยตรง ไม่กวน Server (Server Load = 0)
3.  **Guaranteed Movement:** ปากขยับแน่นอน 100% เมื่อเข้าโหมด Speaking ไม่ต้องกลัวเสียงเบา หรือ Mic ไม่ดัง
4.  **Natural Look:** การผสม Sine Wave + Random ทำให้ดูเหมือนพูดจริงๆ มากกว่าการขยับแบบตายตัว

---

## 3. สรุปสถานะระบบ (Final Status)
*   [x] **AvatarManager:** Refactored ให้ใช้ Simulation Loop
*   [x] **Visualizer:** ปากและคลื่นเสียงบนหัว ขยับตามค่า Simulation
*   [x] **Performance:** ไหลลื่น เครื่องไม่ร้อน ไม่มีการ Spam Log

*อัปเดตล่าสุด: 2026-01-23*
