/**
 * # โหมดคิด (Thinking Mood)
 * เมื่อ Avatar กำลังประมวลผล/คิด
 * - แสดง ... ตรงกลางหน้า
 * - ซ่อนตา
 * - ปิด eye tracking
 */

import { BaseMood } from './BaseMood.js';

export class ThinkingMood extends BaseMood {
    constructor(controller) {
        super('thinking', controller);
    }

    enter() {
        super.enter();

        // ปิด eye tracking
        this.setEyeTracking(false);

        // ปิด blink (ตาถูกซ่อนอยู่แล้ว)
        this.setBlink(false);

        // แขนท่าคิด
        this.setArmState('arm-thinking');
    }

    exit() {
        super.exit();
    }
}
