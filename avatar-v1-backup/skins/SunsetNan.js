/**
 * # Skin พระอาทิตย์ตก (Sunset Nan)
 * ธีมพระอาทิตย์ตก gradient สวย
 * 
 * 🌅 ลักษณะ:
 * - gradient ส้ม-ชมพู-ม่วง
 * - ความรู้สึกโรแมนติก อบอุ่น
 * - เหมาะกับช่วงเย็น
 */

import { BaseSkin } from './BaseSkin.js';

export class SunsetNanSkin extends BaseSkin {
    constructor() {
        super('SunsetNan');
    }

    getInfo() {
        return {
            name: 'น้องน่านพระอาทิตย์ตก',
            nameEN: 'Sunset Nan',
            author: 'AI Robot Guide Team',
            version: '1.0.0',
            description: 'ธีมพระอาทิตย์ตก โทนส้ม-ม่วง อบอุ่น',
            emoji: '🌅'
        };
    }

    getMoodColors() {
        return {
            normal: {
                eye: '#fb923c',
                accent: '#f97316',
                glow: 'rgba(251, 146, 60, 0.5)',
                name: 'ปกติ'
            },
            speaking: {
                eye: '#fdba74',
                accent: '#fb923c',
                glow: 'rgba(253, 186, 116, 0.6)',
                name: 'พูด'
            },
            thinking: {
                eye: '#c084fc',
                accent: '#a855f7',
                glow: 'rgba(192, 132, 252, 0.5)',
                name: 'คิด'
            },
            listening: {
                eye: '#f472b6',
                accent: '#ec4899',
                glow: 'rgba(244, 114, 182, 0.5)',
                name: 'ฟัง'
            },
            happy: {
                eye: '#fcd34d',
                accent: '#fbbf24',
                glow: 'rgba(252, 211, 77, 0.6)',
                name: 'ดีใจ'
            },
            curious: {
                eye: '#e879f9',
                accent: '#d946ef',
                glow: 'rgba(232, 121, 249, 0.5)',
                name: 'สงสัย'
            },
            sleepy: {
                eye: '#78716c',
                accent: '#57534e',
                glow: 'rgba(120, 113, 108, 0.3)',
                name: 'ง่วง'
            }
        };
    }

    getCSSVariables() {
        return {
            '--color-robot-white': '#fef7f0',
            '--color-robot-dark': '#431407',
            '--transition-mood': '0.5s ease'
        };
    }
}
