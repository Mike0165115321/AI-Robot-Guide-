/**
 * # Skin ป่าเขา (Forest Nan)
 * ธีมธรรมชาติ ป่าไม้
 * 
 * 🌲 ลักษณะ:
 * - โทนเขียวธรรมชาติ
 * - ความรู้สึกสงบ ร่มเย็น
 * - เหมาะกับเนื้อหาธรรมชาติ/ท่องเที่ยว
 */

import { BaseSkin } from './BaseSkin.js';

export class ForestNanSkin extends BaseSkin {
    constructor() {
        super('ForestNan');
    }

    getInfo() {
        return {
            name: 'น้องน่านป่าเขา',
            nameEN: 'Forest Nan',
            author: 'AI Robot Guide Team',
            version: '1.0.0',
            description: 'ธีมป่าเขา โทนเขียวธรรมชาติ',
            emoji: '🌲'
        };
    }

    getMoodColors() {
        return {
            normal: {
                eye: '#4ade80',
                accent: '#22c55e',
                glow: 'rgba(74, 222, 128, 0.5)',
                name: 'ปกติ'
            },
            speaking: {
                eye: '#86efac',
                accent: '#4ade80',
                glow: 'rgba(134, 239, 172, 0.6)',
                name: 'พูด'
            },
            thinking: {
                eye: '#a3e635',
                accent: '#84cc16',
                glow: 'rgba(163, 230, 53, 0.5)',
                name: 'คิด'
            },
            listening: {
                eye: '#6ee7b7',
                accent: '#34d399',
                glow: 'rgba(110, 231, 183, 0.5)',
                name: 'ฟัง'
            },
            happy: {
                eye: '#bef264',
                accent: '#a3e635',
                glow: 'rgba(190, 242, 100, 0.6)',
                name: 'ดีใจ'
            },
            curious: {
                eye: '#fde047',
                accent: '#facc15',
                glow: 'rgba(253, 224, 71, 0.5)',
                name: 'สงสัย'
            },
            sleepy: {
                eye: '#6b7280',
                accent: '#4b5563',
                glow: 'rgba(107, 114, 128, 0.3)',
                name: 'ง่วง'
            }
        };
    }

    getCSSVariables() {
        return {
            '--color-robot-white': '#f0fdf4',
            '--color-robot-dark': '#14532d',
            '--transition-mood': '0.5s ease'
        };
    }
}
