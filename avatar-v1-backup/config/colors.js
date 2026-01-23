/**
 * # ค่าสีสำหรับ Mood ต่างๆ
 * ไฟล์นี้เก็บค่าสีทั้งหมดที่ใช้ในระบบ Avatar
 * แก้ไขที่นี่เพื่อเปลี่ยนสี mood ทั้งหมด
 */

export const MOOD_COLORS = {
    normal: {
        eye: '#40c4ff',
        accent: '#00e5ff',
        glow: 'rgba(64, 196, 255, 0.5)',
        name: 'ปกติ'
    },
    speaking: {
        eye: '#4ade80',
        accent: '#22c55e',
        glow: 'rgba(74, 222, 128, 0.6)',
        name: 'พูด'
    },
    thinking: {
        eye: '#fbbf24',
        accent: '#f59e0b',
        glow: 'rgba(251, 191, 36, 0.5)',
        name: 'คิด'
    },
    listening: {
        eye: '#f472b6',
        accent: '#ec4899',
        glow: 'rgba(244, 114, 182, 0.5)',
        name: 'ฟัง'
    },
    happy: {
        eye: '#34d399',
        accent: '#10b981',
        glow: 'rgba(52, 211, 153, 0.6)',
        name: 'ดีใจ'
    },
    curious: {
        eye: '#a78bfa',
        accent: '#8b5cf6',
        glow: 'rgba(167, 139, 250, 0.5)',
        name: 'สงสัย'
    },
    sleepy: {
        eye: '#64748b',
        accent: '#475569',
        glow: 'rgba(100, 116, 139, 0.3)',
        name: 'ง่วง'
    }
};

/**
 * # ค่า Timing สำหรับ Animation
 */
export const TIMING = {
    blink: {
        minInterval: 3000,  // 3 วินาที
        maxInterval: 6000   // 6 วินาที
    },
    idle: {
        timeout: 8000       // 8 วินาที ก่อนเริ่ม idle behavior
    },
    eyeTracking: {
        duration: 0.4,      // ความเร็วในการติดตามตา
        maxMove: 12         // ระยะสูงสุดที่ตาเคลื่อนที่
    }
};
