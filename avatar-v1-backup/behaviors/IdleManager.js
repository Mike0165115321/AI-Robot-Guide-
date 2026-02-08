/**
 * # ตัวจัดการ Idle Behaviors
 * ควบคุม behavior อัตโนมัติเมื่อ avatar ว่าง
 */

import { TIMING } from '../config/colors.js';

export class IdleManager {
    constructor(controller) {
        this.controller = controller;
        this.idleTimer = null;
        this.isIdleMood = false;
        this.behaviors = [];
    }

    /**
     * # ลงทะเบียน behavior ใหม่
     * @param {Function} behaviorFn - ฟังก์ชัน behavior
     */
    register(behaviorFn) {
        this.behaviors.push(behaviorFn);
    }

    /**
     * # รีเซ็ต Timer (เรียกเมื่อมี activity)
     */
    reset() {
        if (this.idleTimer) clearTimeout(this.idleTimer);

        // กลับสู่ปกติถ้าอยู่ใน idle mood
        if (this.isIdleMood) {
            this.controller.setMood('normal');
            this.isIdleMood = false;
        }

        this.idleTimer = setTimeout(() => this.trigger(), TIMING.idle.timeout);
    }

    /**
     * # Trigger behavior แบบสุ่ม
     */
    trigger() {
        if (this.behaviors.length === 0) return;

        this.isIdleMood = true;
        const behavior = this.behaviors[Math.floor(Math.random() * this.behaviors.length)];
        behavior();

        // นัด trigger ถัดไป
        this.idleTimer = setTimeout(() => this.trigger(), 5000 + Math.random() * 5000);
    }

    /**
     * # หยุด Idle behaviors
     */
    stop() {
        if (this.idleTimer) {
            clearTimeout(this.idleTimer);
            this.idleTimer = null;
        }
    }
}
