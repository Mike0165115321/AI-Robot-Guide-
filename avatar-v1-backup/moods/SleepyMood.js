/**
 * # โหมดง่วง (Sleepy Mood)
 * เมื่อ Avatar ง่วง/พักผ่อน
 * - ตาหรี่
 * - ปากหาว
 */

import { BaseMood } from './BaseMood.js';

export class SleepyMood extends BaseMood {
    constructor(controller) {
        super('sleepy', controller);
    }

    enter() {
        super.enter();

        // ปิด eye tracking
        this.setEyeTracking(false);

        // ปิด blink (ตาหรี่อยู่แล้ว)
        this.setBlink(false);

        // แขนท่า idle
        this.setArmState('arm-idle');
    }

    exit() {
        super.exit();
    }
}
