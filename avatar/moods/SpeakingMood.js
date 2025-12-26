/**
 * # โหมดพูด (Speaking Mood)
 * เมื่อ Avatar กำลังพูด/ตอบคำถาม
 * - ปากขยับ
 * - แขนขยับตามการพูด
 * - ปิด eye tracking (มองตรง)
 */

import { BaseMood } from './BaseMood.js';

export class SpeakingMood extends BaseMood {
    constructor(controller) {
        super('speaking', controller);
    }

    enter() {
        super.enter();

        // ปิด eye tracking (มองตรง)
        this.setEyeTracking(false);

        // เปิด blink
        this.setBlink(true);

        // แขนขยับตามการพูด
        this.setArmState('arm-speaking');

        // เปิด animation ปาก
        const { mouth } = this.controller;
        if (mouth) {
            mouth.classList.add('speaking');
        }
    }

    exit() {
        super.exit();

        // ปิด animation ปาก
        const { mouth } = this.controller;
        if (mouth) {
            mouth.classList.remove('speaking');
        }
    }
}
