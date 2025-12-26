/**
 * # โหมดดีใจ (Happy Mood)
 * เมื่อ Avatar ดีใจ/ยินดี
 * - ตาโค้งยิ้ม เด้งๆ
 * - ปากยิ้ม
 */

import { BaseMood } from './BaseMood.js';

export class HappyMood extends BaseMood {
    constructor(controller) {
        super('happy', controller);
    }

    enter() {
        super.enter();

        // เปิด eye tracking
        this.setEyeTracking(true);

        // เปิด blink
        this.setBlink(true);

        // แขนท่า idle
        this.setArmState('arm-idle');
    }

    exit() {
        super.exit();
    }
}
