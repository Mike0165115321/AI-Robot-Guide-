/**
 * # AvatarController - ตัวควบคุมหลักของ Avatar
 * รวม modules ทั้งหมดเข้าด้วยกัน
 */

import { MOOD_COLORS } from '../config/colors.js';
import { EyeTracking } from './EyeTracking.js';
import { BlinkController } from './BlinkController.js';
import { IdleManager } from '../behaviors/IdleManager.js';
import { NanRobotSkin } from '../skins/NanRobot.js';

// Import Moods
import { NormalMood } from '../moods/NormalMood.js';
import { SpeakingMood } from '../moods/SpeakingMood.js';
import { ThinkingMood } from '../moods/ThinkingMood.js';
import { ListeningMood } from '../moods/ListeningMood.js';
import { HappyMood } from '../moods/HappyMood.js';
import { CuriousMood } from '../moods/CuriousMood.js';
import { SleepyMood } from '../moods/SleepyMood.js';

export class AvatarController {
    constructor() {
        // DOM References
        this.container = null;
        this.face = null;
        this.eyes = [];
        this.leftArm = null;
        this.rightArm = null;
        this.mouth = null;

        // Sub-controllers
        this.eyeTracking = null;
        this.blinkController = null;
        this.idleManager = null;

        // State
        this.currentMood = 'normal';
        this.currentMoodInstance = null;
        this.moods = {};
        this.skin = null;
        this.isInitialized = false;
    }

    /**
     * # เริ่มต้น Avatar
     */
    init() {
        // หา DOM elements
        this.container = document.getElementById('robot-master-container');
        this.face = document.getElementById('robot-face');
        this.eyes = document.querySelectorAll('.robot-eye');
        this.leftArm = document.getElementById('left-arm');
        this.rightArm = document.getElementById('right-arm');
        this.mouth = document.querySelector('.robot-mouth');

        if (!this.container || !this.face || this.eyes.length === 0) {
            console.error('❌ AvatarController: Required DOM elements not found!');
            return;
        }

        // สร้าง sub-controllers
        this.eyeTracking = new EyeTracking(this.eyes, this.face);
        this.blinkController = new BlinkController(this.eyes);
        this.idleManager = new IdleManager(this);

        // ลงทะเบียน moods
        this.registerMoods();

        // ตั้งค่า skin
        this.skin = new NanRobotSkin();
        this.skin.apply();

        // Bind events
        this.bindEvents();

        // เริ่ม loops
        this.blinkController.start();
        this.idleManager.reset();

        // ตั้ง mood เริ่มต้น
        this.setMood('normal');

        this.isInitialized = true;
        console.log('✅ AvatarController (Modular) Ready!');
    }

    /**
     * # ลงทะเบียน Mood ทั้งหมด
     */
    registerMoods() {
        this.moods = {
            normal: new NormalMood(this),
            speaking: new SpeakingMood(this),
            thinking: new ThinkingMood(this),
            listening: new ListeningMood(this),
            happy: new HappyMood(this),
            curious: new CuriousMood(this),
            sleepy: new SleepyMood(this)
        };
    }

    /**
     * # Bind Events
     */
    bindEvents() {
        // Eye tracking
        document.addEventListener('mousemove', (e) => {
            this.eyeTracking.track(e);
        });

        // Reset idle timer on activity
        ['mousemove', 'click', 'keydown', 'touchstart'].forEach(event => {
            document.addEventListener(event, () => {
                this.idleManager.reset();
            });
        });
    }

    /**
     * # เปลี่ยน Mood
     * @param {string} moodName - ชื่อ mood
     */
    setMood(moodName) {
        const colors = MOOD_COLORS[moodName] || MOOD_COLORS.normal;
        const newMood = this.moods[moodName];

        if (!newMood) {
            console.warn(`⚠️ Unknown mood: ${moodName}`);
            return;
        }

        // Exit current mood
        if (this.currentMoodInstance) {
            this.currentMoodInstance.exit();
        }

        // Update CSS Variables
        document.documentElement.style.setProperty('--mood-eye-color', colors.eye);
        document.documentElement.style.setProperty('--mood-accent-color', colors.accent);
        document.documentElement.style.setProperty('--mood-glow', colors.glow);

        // Update face class
        const moodClasses = Object.keys(this.moods).map(m => `mood-${m}`);
        this.face.classList.remove(...moodClasses);
        this.face.classList.add(`mood-${moodName}`);

        // Enter new mood
        newMood.enter();

        this.currentMood = moodName;
        this.currentMoodInstance = newMood;

        console.log(`🎨 Mood changed to: ${moodName}`);
    }

    // ==========================================
    // PUBLIC API (สำหรับเรียกจากภายนอก)
    // ==========================================

    speak() { this.setMood('speaking'); }
    stopSpeak() {
        if (this.currentMood === 'speaking') {
            this.setMood('normal');
        }
    }
    think() { this.setMood('thinking'); }
    listen() { this.setMood('listening'); }
    happy() { this.setMood('happy'); }
    curious() { this.setMood('curious'); }
    sleepy() { this.setMood('sleepy'); }
    idle() { this.setMood('normal'); }
}
