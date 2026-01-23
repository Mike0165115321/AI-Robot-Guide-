/**
 * # ระบบกระพริบตา (Blink Controller)
 * ควบคุมการกระพริบตาอัตโนมัติ
 */

import { TIMING } from '../config/colors.js';

export class BlinkController {
    constructor(eyes) {
        this.eyes = eyes;
        this.blinkInterval = null;
        this.isPaused = false;
    }

    /**
     * # เริ่ม Loop กระพริบตา
     */
    start() {
        this.stop(); // หยุดอันเก่าก่อน

        const blink = () => {
            if (this.isPaused) return;

            if (typeof gsap !== 'undefined') {
                gsap.to(this.eyes, {
                    scaleY: 0.1,
                    duration: 0.08,
                    yoyo: true,
                    repeat: 1,
                    ease: 'power2.inOut'
                });
            } else {
                this.eyes.forEach(eye => {
                    eye.classList.add('blink');
                    setTimeout(() => eye.classList.remove('blink'), 150);
                });
            }
        };

        // สุ่มเวลากระพริบ (3-6 วินาที)
        const randomInterval = () => {
            return TIMING.blink.minInterval +
                Math.random() * (TIMING.blink.maxInterval - TIMING.blink.minInterval);
        };

        const scheduleNextBlink = () => {
            this.blinkInterval = setTimeout(() => {
                blink();
                scheduleNextBlink();
            }, randomInterval());
        };

        scheduleNextBlink();
    }

    /**
     * # หยุด Loop กระพริบตา
     */
    stop() {
        if (this.blinkInterval) {
            clearTimeout(this.blinkInterval);
            this.blinkInterval = null;
        }
    }

    /**
     * # หยุดชั่วคราว (ไม่ปิด loop)
     */
    pause() {
        this.isPaused = true;
    }

    /**
     * # เล่นต่อ
     */
    resume() {
        this.isPaused = false;
    }

    /**
     * # กระพริบตาทันที
     */
    blinkNow() {
        if (typeof gsap !== 'undefined') {
            gsap.to(this.eyes, {
                scaleY: 0.1,
                duration: 0.08,
                yoyo: true,
                repeat: 1,
                ease: 'power2.inOut'
            });
        }
    }
}
