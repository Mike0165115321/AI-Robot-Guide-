/**
 * # Base Skin - รากฐานสำหรับ Skin ทุกแบบ
 * Skin ใหม่ต้อง extends จาก class นี้
 */

export class BaseSkin {
    constructor(name) {
        this.name = name;
    }

    /**
     * # ข้อมูล Skin
     */
    getInfo() {
        return {
            name: this.name,
            author: 'Unknown',
            version: '1.0.0'
        };
    }

    /**
     * # CSS Variables ที่ใช้
     * Override ใน subclass
     */
    getCSSVariables() {
        return {};
    }

    /**
     * # DOM Elements ที่ต้องมี
     * Override ใน subclass
     */
    getRequiredElements() {
        return [
            'robot-master-container',
            'robot-face',
            'robot-eye',
            'left-arm',
            'right-arm'
        ];
    }
}
