/**
 * # โหมดพูด (Speaking Mood)
 * เมื่อ Avatar กำลังพูด/ตอบคำถาม
 * - ปากขยับ
 * - แขนขยับตามการพูด
 * - ปิด eye tracking (มองตรง)
 */

import { BaseMood } from './BaseMood.js';

export class SpeakingMood extends BaseMood {
    constructor(controller) {
        super('speaking', controller);
    }

    enter() {
        super.enter();

        // ปิด eye tracking (มองตรง)
        this.setEyeTracking(false);

        // เปิด blink
        this.setBlink(true);

        // แขนขยับตามการพูด
        this.setArmState('arm-speaking');

        // 🆕 Remove 'speaking' class (Disable CSS Animation)
        // We will control it manually now
        const { mouth } = this.controller;
        if (mouth) {
            mouth.classList.remove('speaking');
            mouth.style.transition = 'transform 0.05s linear'; // Smooth transition for real-time updates
        }
    }

    exit() {
        super.exit();

        const { mouth } = this.controller;
        if (mouth) {
            mouth.classList.remove('speaking');
            mouth.style.transform = ''; // Reset transform
            mouth.style.transition = '';
        }

        // Reset audio waves
        const waves = document.querySelectorAll('.wave');
        waves.forEach(w => w.style.height = '');
    }

    // 🆕 Real-time Lip Sync Logic
    updateVoiceLevel(volume) {
        // volume is 0.0 to 1.0 (approx)

        let effectiveVol = volume;

        // 🛡️ Fallback: Only if ABSOLUTE silence (0) but state is speaking
        // Reduce fallback intensity to distinguish from real sync
        if (effectiveVol <= 0.001) {
            const time = Date.now() / 150;
            effectiveVol = (Math.sin(time) + 1) * 0.1; // Gentle breathing motion
        }

        // 1. Mouth Movement (Scale Y)
        const { mouth } = this.controller;
        if (mouth) {
            // Remove noise gate (allow small movements)
            const openAmount = 0.2 + (effectiveVol * 1.5);
            mouth.style.transform = `scaleY(${Math.min(openAmount, 1.8)})`;
        }

        // 2. Head Audio Waves (Visualizer)
        const waves = document.querySelectorAll('.wave');
        waves.forEach((wave, index) => {
            const waveVol = Math.max(0.2, effectiveVol * (1 + index * 0.5));
            const height = 10 + (waveVol * 50);
            wave.style.height = `${height}px`;
        });
    }
}
