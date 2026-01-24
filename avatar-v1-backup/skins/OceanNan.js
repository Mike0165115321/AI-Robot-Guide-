/**
 * # Skin ทะเล (Ocean Nan)
 * ธีมทะเล มหาสมุทร
 * 
 * 🌊 ลักษณะ:
 * - โทนฟ้าทะเล
 * - ความรู้สึกสดชื่น เย็นสบาย
 * - เหมาะกับเนื้อหาทะเล/พักผ่อน
 */

import { BaseSkin } from './BaseSkin.js';

export class OceanNanSkin extends BaseSkin {
    constructor() {
        super('OceanNan');
    }

    getInfo() {
        return {
            name: 'น้องน่านทะเล',
            nameEN: 'Ocean Nan',
            author: 'AI Robot Guide Team',
            version: '1.0.0',
            description: 'ธีมทะเล โทนฟ้าสดชื่น',
            emoji: '🌊'
        };
    }

    getMoodColors() {
        return {
            normal: {
                eye: '#38bdf8',
                accent: '#0ea5e9',
                glow: 'rgba(56, 189, 248, 0.5)',
                name: 'ปกติ'
            },
            speaking: {
                eye: '#7dd3fc',
                accent: '#38bdf8',
                glow: 'rgba(125, 211, 252, 0.6)',
                name: 'พูด'
            },
            thinking: {
                eye: '#22d3ee',
                accent: '#06b6d4',
                glow: 'rgba(34, 211, 238, 0.5)',
                name: 'คิด'
            },
            listening: {
                eye: '#67e8f9',
                accent: '#22d3ee',
                glow: 'rgba(103, 232, 249, 0.5)',
                name: 'ฟัง'
            },
            happy: {
                eye: '#a5f3fc',
                accent: '#67e8f9',
                glow: 'rgba(165, 243, 252, 0.6)',
                name: 'ดีใจ'
            },
            curious: {
                eye: '#c4b5fd',
                accent: '#a78bfa',
                glow: 'rgba(196, 181, 253, 0.5)',
                name: 'สงสัย'
            },
            sleepy: {
                eye: '#64748b',
                accent: '#475569',
                glow: 'rgba(100, 116, 139, 0.3)',
                name: 'ง่วง'
            }
        };
    }

    getCSSVariables() {
        return {
            '--color-robot-white': '#f0f9ff',
            '--color-robot-dark': '#0c4a6e',
            '--transition-mood': '0.5s ease'
        };
    }
}
