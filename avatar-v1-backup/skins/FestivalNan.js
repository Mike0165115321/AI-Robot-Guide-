/**
 * # Skin เทศกาล (Festival Nan)
 * ธีมปาร์ตี้ เทศกาล สีสันสดใส
 * 
 * 🎉 ลักษณะ:
 * - สีสันสดใส หลากหลาย
 * - เปลี่ยน mood เปลี่ยนสี dramatic
 * - เหมาะกับงานเฉลิมฉลอง
 */

import { BaseSkin } from './BaseSkin.js';

export class FestivalNanSkin extends BaseSkin {
    constructor() {
        super('FestivalNan');
    }

    getInfo() {
        return {
            name: 'น้องน่านเทศกาล',
            nameEN: 'Festival Nan',
            author: 'AI Robot Guide Team',
            version: '1.0.0',
            description: 'ธีมเทศกาล สีสันสดใส เฉลิมฉลอง',
            emoji: '🎉'
        };
    }

    getMoodColors() {
        return {
            normal: {
                eye: '#f43f5e',
                accent: '#e11d48',
                glow: 'rgba(244, 63, 94, 0.6)',
                name: 'ปกติ'
            },
            speaking: {
                eye: '#8b5cf6',
                accent: '#7c3aed',
                glow: 'rgba(139, 92, 246, 0.6)',
                name: 'พูด'
            },
            thinking: {
                eye: '#06b6d4',
                accent: '#0891b2',
                glow: 'rgba(6, 182, 212, 0.6)',
                name: 'คิด'
            },
            listening: {
                eye: '#10b981',
                accent: '#059669',
                glow: 'rgba(16, 185, 129, 0.6)',
                name: 'ฟัง'
            },
            happy: {
                eye: '#eab308',
                accent: '#ca8a04',
                glow: 'rgba(234, 179, 8, 0.7)',
                name: 'ดีใจ'
            },
            curious: {
                eye: '#ec4899',
                accent: '#db2777',
                glow: 'rgba(236, 72, 153, 0.6)',
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
            '--color-robot-white': '#fefefe',
            '--color-robot-dark': '#1f1f1f',
            '--transition-mood': '0.3s ease'
        };
    }
}
