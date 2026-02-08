/**
 * # Base Class สำหรับ Mood ทุกประเภท
 * Mood ใหม่ทุกตัวต้อง extends จาก class นี้
 */

export class BaseMood {
    constructor(name, controller) {
        this.name = name;
        this.controller = controller;
    }

    /**
     * # เรียกเมื่อเข้า mood นี้
     * Override ใน subclass เพื่อกำหนด behavior
     */
    enter() {
        console.log(`🎭 Entering mood: ${this.name}`);
    }

    /**
     * # เรียกเมื่อออกจาก mood นี้
     * Override ใน subclass เพื่อ cleanup
     */
    exit() {
        console.log(`👋 Exiting mood: ${this.name}`);
    }

    /**
     * # ตั้งค่าแขน
     * @param {string} state - class name สำหรับแขน
     */
    setArmState(state) {
        const { leftArm, rightArm } = this.controller;
        if (!leftArm || !rightArm) return;

        const armStates = ['arm-idle', 'arm-speaking', 'arm-thinking',
            'arm-listening', 'arm-waving', 'arm-curious'];

        [leftArm, rightArm].forEach(arm => {
            arm.classList.remove(...armStates);
            arm.classList.add(state);
        });
    }

    /**
     * # ตั้งค่า Eye Tracking
     * @param {boolean} enabled - เปิด/ปิด
     */
    setEyeTracking(enabled) {
        if (this.controller.eyeTracking) {
            enabled ? this.controller.eyeTracking.enable()
                : this.controller.eyeTracking.disable();
        }
    }

    /**
     * # ตั้งค่า Blink
     * @param {boolean} enabled - เปิด/ปิด
     */
    setBlink(enabled) {
        if (this.controller.blinkController) {
            enabled ? this.controller.blinkController.resume()
                : this.controller.blinkController.pause();
        }
    }
}
