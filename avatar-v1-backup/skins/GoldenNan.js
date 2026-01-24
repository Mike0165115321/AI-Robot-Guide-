/**
 * # Skin ทองคำ (Golden Nan)
 * ธีมหรูหรา สีทอง สำหรับโอกาสพิเศษ
 * 
 * 🌟 ลักษณะ:
 * - สีทองเป็น accent หลัก
 * - Glow สีทอง
 * - ความรู้สึกหรูหรา premium
 */

import { BaseSkin } from './BaseSkin.js';

export class GoldenNanSkin extends BaseSkin {
    constructor() {
        super('GoldenNan');
    }

    getInfo() {
        return {
            name: 'น้องน่านทองคำ',
            nameEN: 'Golden Nan',
            author: 'AI Robot Guide Team',
            version: '1.0.0',
            description: 'ธีมหรูหราสีทอง เหมาะกับโอกาสพิเศษ',
            emoji: '🌟'
        };
    }

    getMoodColors() {
        return {
            normal: {
                eye: '#ffd700',
                accent: '#ffb800',
                glow: 'rgba(255, 215, 0, 0.5)',
                name: 'ปกติ'
            },
            speaking: {
                eye: '#ffe066',
                accent: '#ffc107',
                glow: 'rgba(255, 193, 7, 0.6)',
                name: 'พูด'
            },
            thinking: {
                eye: '#f0c14b',
                accent: '#d4a017',
                glow: 'rgba(212, 160, 23, 0.5)',
                name: 'คิด'
            },
            listening: {
                eye: '#ffdf6f',
                accent: '#e6b800',
                glow: 'rgba(230, 184, 0, 0.5)',
                name: 'ฟัง'
            },
            happy: {
                eye: '#fff176',
                accent: '#fdd835',
                glow: 'rgba(253, 216, 53, 0.6)',
                name: 'ดีใจ'
            },
            curious: {
                eye: '#ffcc80',
                accent: '#ffa726',
                glow: 'rgba(255, 167, 38, 0.5)',
                name: 'สงสัย'
            },
            sleepy: {
                eye: '#c9a227',
                accent: '#9a7b0a',
                glow: 'rgba(154, 123, 10, 0.3)',
                name: 'ง่วง'
            }
        };
    }

    getCSSVariables() {
        return {
            '--color-robot-white': '#fffef2',
            '--color-robot-dark': '#3d3000',
            '--transition-mood': '0.5s ease'
        };
    }
}
