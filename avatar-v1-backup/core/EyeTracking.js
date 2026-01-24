/**
 * # ระบบติดตามตา (Eye Tracking)
 * ทำให้ตาของ Avatar ติดตามตำแหน่งเมาส์
 */

import { TIMING } from '../config/colors.js';

export class EyeTracking {
    constructor(eyes, face) {
        this.eyes = eyes;
        this.face = face;
        this.isEnabled = true;
        this.lastMove = { x: 0, y: 0 };
        this.maxMove = TIMING.eyeTracking.maxMove;
    }

    /**
     * # เปิดการติดตามตา
     */
    enable() {
        this.isEnabled = true;
    }

    /**
     * # ปิดการติดตามตา
     */
    disable() {
        this.isEnabled = false;
    }

    /**
     * # คำนวณและเคลื่อนตาตามตำแหน่งเมาส์
     * @param {MouseEvent} event - Mouse event
     */
    track(event) {
        if (!this.isEnabled || !this.face || !this.eyes.length) return;

        const rect = this.face.getBoundingClientRect();
        const centerX = rect.left + rect.width / 2;
        const centerY = rect.top + rect.height / 2;

        let moveX = (event.clientX - centerX) / 50;
        let moveY = (event.clientY - centerY) / 50;

        // จำกัดระยะการเคลื่อนที่
        moveX = Math.max(-this.maxMove, Math.min(this.maxMove, moveX));
        moveY = Math.max(-this.maxMove, Math.min(this.maxMove, moveY));

        this.lastMove = { x: moveX, y: moveY };

        // ใช้ GSAP ถ้ามี ไม่งั้นใช้ CSS
        if (typeof gsap !== 'undefined') {
            gsap.to(this.eyes, {
                x: moveX,
                y: moveY,
                duration: TIMING.eyeTracking.duration,
                ease: 'power3.out'
            });
        } else {
            this.eyes.forEach(eye => {
                eye.style.transform = `translate(${moveX}px, ${moveY}px)`;
            });
        }
    }

    /**
     * # รีเซ็ตตาไปตำแหน่งกลาง
     */
    reset() {
        this.lastMove = { x: 0, y: 0 };
        if (typeof gsap !== 'undefined') {
            gsap.to(this.eyes, { x: 0, y: 0, duration: 0.3 });
        } else {
            this.eyes.forEach(eye => {
                eye.style.transform = 'translate(0, 0)';
            });
        }
    }
}
