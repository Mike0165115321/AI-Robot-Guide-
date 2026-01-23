/**
 * # โหมดสงสัย (Curious Mood)
 * เมื่อ Avatar สงสัย/ไม่แน่ใจ
 * - ตาใหญ่
 * - แสดง ? ลอยข้าง
 * - เปิด eye tracking
 */

import { BaseMood } from './BaseMood.js';

export class CuriousMood extends BaseMood {
    constructor(controller) {
        super('curious', controller);
    }

    enter() {
        super.enter();

        // เปิด eye tracking
        this.setEyeTracking(true);

        // เปิด blink
        this.setBlink(true);

        // แขนท่า curious
        this.setArmState('arm-curious');
    }

    exit() {
        super.exit();
    }
}
