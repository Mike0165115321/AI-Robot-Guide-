/**
 * # Skin กลางคืน (Night Nan)
 * ธีมโทนมืด สำหรับดูตอนกลางคืน
 * 
 * 🌙 ลักษณะ:
 * - พื้นหลังโทนมืด
 * - Accent สีม่วงน้ำเงิน
 * - เหมาะกับ Dark Mode
 */

import { BaseSkin } from './BaseSkin.js';

export class NightNanSkin extends BaseSkin {
    constructor() {
        super('NightNan');
    }

    getInfo() {
        return {
            name: 'น้องน่านกลางคืน',
            nameEN: 'Night Nan',
            author: 'AI Robot Guide Team',
            version: '1.0.0',
            description: 'ธีมกลางคืน โทนมืด สบายตา',
            emoji: '🌙'
        };
    }

    getMoodColors() {
        return {
            normal: {
                eye: '#8b9dc3',
                accent: '#6a7fdb',
                glow: 'rgba(106, 127, 219, 0.5)',
                name: 'ปกติ'
            },
            speaking: {
                eye: '#a8c0ff',
                accent: '#8fadff',
                glow: 'rgba(143, 173, 255, 0.6)',
                name: 'พูด'
            },
            thinking: {
                eye: '#c4b7ff',
                accent: '#9d8cff',
                glow: 'rgba(157, 140, 255, 0.5)',
                name: 'คิด'
            },
            listening: {
                eye: '#e0b3ff',
                accent: '#c87dff',
                glow: 'rgba(200, 125, 255, 0.5)',
                name: 'ฟัง'
            },
            happy: {
                eye: '#b8d4ff',
                accent: '#96c3ff',
                glow: 'rgba(150, 195, 255, 0.6)',
                name: 'ดีใจ'
            },
            curious: {
                eye: '#dbb3ff',
                accent: '#bb8cff',
                glow: 'rgba(187, 140, 255, 0.5)',
                name: 'สงสัย'
            },
            sleepy: {
                eye: '#5c6378',
                accent: '#3d4555',
                glow: 'rgba(61, 69, 85, 0.3)',
                name: 'ง่วง'
            }
        };
    }

    getCSSVariables() {
        return {
            '--color-robot-white': '#2a2d3e',
            '--color-robot-dark': '#0f1119',
            '--transition-mood': '0.5s ease'
        };
    }
}
