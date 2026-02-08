/**
 * # โหมดปกติ (Normal Mood)
 * สถานะเริ่มต้นของ Avatar
 * - ตาติดตามเมาส์
 * - กระพริบตาปกติ
 * - แขนอยู่ท่า idle
 */

import { BaseMood } from './BaseMood.js';

export class NormalMood extends BaseMood {
    constructor(controller) {
        super('normal', controller);
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
        // ไม่ต้อง cleanup อะไร
    }
}
