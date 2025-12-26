/**
 * # Skin ไซเบอร์พังค์ (Cyber Nan)
 * ธีมแนว Cyberpunk นีออน
 * 
 * 🎮 ลักษณะ:
 * - สีนีออนสด (ชมพู, เขียว, ฟ้า)
 * - ความรู้สึกล้ำยุค
 * - เหมาะกับโชว์ Tech
 */

import { BaseSkin } from './BaseSkin.js';

export class CyberNanSkin extends BaseSkin {
    constructor() {
        super('CyberNan');
    }

    getInfo() {
        return {
            name: 'น้องน่านไซเบอร์',
            nameEN: 'Cyber Nan',
            author: 'AI Robot Guide Team',
            version: '1.0.0',
            description: 'ธีม Cyberpunk นีออน ล้ำยุค',
            emoji: '🎮'
        };
    }

    getMoodColors() {
        return {
            normal: {
                eye: '#00ffff',
                accent: '#ff00ff',
                glow: 'rgba(0, 255, 255, 0.7)',
                name: 'ปกติ'
            },
            speaking: {
                eye: '#00ff9f',
                accent: '#00ffff',
                glow: 'rgba(0, 255, 159, 0.7)',
                name: 'พูด'
            },
            thinking: {
                eye: '#ff00ff',
                accent: '#bf00ff',
                glow: 'rgba(255, 0, 255, 0.6)',
                name: 'คิด'
            },
            listening: {
                eye: '#ff3366',
                accent: '#ff0055',
                glow: 'rgba(255, 51, 102, 0.6)',
                name: 'ฟัง'
            },
            happy: {
                eye: '#39ff14',
                accent: '#00ff00',
                glow: 'rgba(57, 255, 20, 0.7)',
                name: 'ดีใจ'
            },
            curious: {
                eye: '#ffff00',
                accent: '#ff9900',
                glow: 'rgba(255, 255, 0, 0.6)',
                name: 'สงสัย'
            },
            sleepy: {
                eye: '#4a5568',
                accent: '#2d3748',
                glow: 'rgba(74, 85, 104, 0.4)',
                name: 'ง่วง'
            }
        };
    }

    getCSSVariables() {
        return {
            '--color-robot-white': '#1a1a2e',
            '--color-robot-dark': '#0f0f1a',
            '--transition-mood': '0.3s ease'
        };
    }
}
