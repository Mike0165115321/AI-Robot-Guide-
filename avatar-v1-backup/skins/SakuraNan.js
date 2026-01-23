/**
 * # Skin ซากุระ (Sakura Nan)
 * ธีมชมพูซากุระ หวานแหวว
 * 
 * 🌸 ลักษณะ:
 * - โทนชมพูพาสเทล
 * - ความรู้สึกอ่อนโยน น่ารัก
 * - เหมาะกับเนื้อหาเบาๆ
 */

import { BaseSkin } from './BaseSkin.js';

export class SakuraNanSkin extends BaseSkin {
    constructor() {
        super('SakuraNan');
    }

    getInfo() {
        return {
            name: 'น้องน่านซากุระ',
            nameEN: 'Sakura Nan',
            author: 'AI Robot Guide Team',
            version: '1.0.0',
            description: 'ธีมซากุระ โทนชมพูหวาน น่ารัก',
            emoji: '🌸'
        };
    }

    getMoodColors() {
        return {
            normal: {
                eye: '#ffb6c1',
                accent: '#ff91a4',
                glow: 'rgba(255, 182, 193, 0.5)',
                name: 'ปกติ'
            },
            speaking: {
                eye: '#ffc1cc',
                accent: '#ff8fa3',
                glow: 'rgba(255, 143, 163, 0.6)',
                name: 'พูด'
            },
            thinking: {
                eye: '#e8b4d8',
                accent: '#d18ec4',
                glow: 'rgba(209, 142, 196, 0.5)',
                name: 'คิด'
            },
            listening: {
                eye: '#ffaec9',
                accent: '#ff85a1',
                glow: 'rgba(255, 133, 161, 0.5)',
                name: 'ฟัง'
            },
            happy: {
                eye: '#ffd1dc',
                accent: '#ffb3c6',
                glow: 'rgba(255, 179, 198, 0.6)',
                name: 'ดีใจ'
            },
            curious: {
                eye: '#e8a4c4',
                accent: '#d77fa1',
                glow: 'rgba(215, 127, 161, 0.5)',
                name: 'สงสัย'
            },
            sleepy: {
                eye: '#b8a0a8',
                accent: '#8f7880',
                glow: 'rgba(143, 120, 128, 0.3)',
                name: 'ง่วง'
            }
        };
    }

    getCSSVariables() {
        return {
            '--color-robot-white': '#fff5f7',
            '--color-robot-dark': '#4a2c38',
            '--transition-mood': '0.5s ease'
        };
    }
}
