/**
 * # Skin น้องน่าน (NanRobot)
 * Skin หลักของโครงการ AI Robot Guide จังหวัดน่าน
 * 
 * 🤖 ลักษณะ:
 * - สีขาวหลัก
 * - Accent สีฟ้า/ชมพู
 * - ตาเรืองแสง
 * - แขน 2 ข้าง
 */

import { BaseSkin } from './BaseSkin.js';
import { MOOD_COLORS } from '../config/colors.js';

export class NanRobotSkin extends BaseSkin {
    constructor() {
        super('NanRobot');
    }

    getInfo() {
        return {
            name: 'น้องน่าน',
            nameEN: 'NanRobot',
            author: 'AI Robot Guide Team',
            version: '1.0.0',
            description: 'หุ่นยนต์ AI Guide ประจำจังหวัดน่าน'
        };
    }

    /**
     * # CSS Variables สำหรับ skin นี้
     */
    getCSSVariables() {
        return {
            '--color-robot-white': '#fcfcfc',
            '--color-robot-dark': '#2c2f33',
            '--transition-mood': '0.5s ease'
        };
    }

    /**
     * # ค่าสี Mood
     */
    getMoodColors() {
        return MOOD_COLORS;
    }

    /**
     * # Elements ที่ต้องมี
     */
    getRequiredElements() {
        return [
            'robot-master-container',
            'robot-face',
            'robot-eye',
            'robot-mouth',
            'left-arm',
            'right-arm',
            'thinking-dots',
            'question-mark'
        ];
    }

    /**
     * # Apply skin CSS
     */
    apply() {
        const cssVars = this.getCSSVariables();
        Object.entries(cssVars).forEach(([key, value]) => {
            document.documentElement.style.setProperty(key, value);
        });
        console.log(`🎨 Applied skin: ${this.getInfo().name}`);
    }
}
