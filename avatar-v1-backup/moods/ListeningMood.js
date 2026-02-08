/**
 * # โหมดฟัง (Listening Mood)
 * เมื่อ Avatar กำลังฟังผู้ใช้พูด
 * - ตาโฟกัส (มีจุดสว่างตรงกลาง)
 * - หูเรืองแสง
 * - เปิด eye tracking
 */

import { BaseMood } from './BaseMood.js';

export class ListeningMood extends BaseMood {
    constructor(controller) {
        super('listening', controller);
    }

    enter() {
        super.enter();

        // เปิด eye tracking
        this.setEyeTracking(true);

        // เปิด blink
        this.setBlink(true);

        // แขนท่าฟัง
        this.setArmState('arm-listening');
    }

    exit() {
        super.exit();
    }
}
