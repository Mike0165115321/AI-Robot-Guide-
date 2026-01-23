/**
 * core/AvatarController.js - Main Orchestrator (Enhanced V2.0)
 */

import { AvatarColors } from '../config/colors.js';
import { EyeTracking } from './EyeTracking.js';
import { BlinkController } from './BlinkController.js';

export class AvatarController {
    constructor() {
        // Configuration
        this.config = {
            colors: AvatarColors,
            idleTimeout: 8000
        };

        // DOM Elements
        this.container = null;
        this.face = null;
        this.eyes = [];
        this.mouth = null;
        this.arms = { left: null, right: null };

        // Components
        this.eyeTracking = new EyeTracking(this);
        this.blinkController = new BlinkController(this);
        this.moods = new Map();
        this.skins = new Map();

        // State
        this.currentMood = null;
        this.currentSkin = null;
        this.isInitialized = false;
    }

    async init() {
        this.container = document.getElementById('robot-master-container');
        if (!this.container) {
            console.error('AvatarController: Container not found!');
            return;
        }

        this.refreshDOMElements();
        this.eyeTracking.init();
        this.blinkController.start();

        this.isInitialized = true;
        console.log('🤖 Nan Avatar V2.0 Engine Started!');
    }

    refreshDOMElements() {
        this.face = this.container.querySelector('.robot-face');
        this.eyes = this.container.querySelectorAll('.robot-eye');
        this.mouth = this.container.querySelector('.robot-mouth');
        this.arms.left = this.container.querySelector('#left-arm');
        this.arms.right = this.container.querySelector('#right-arm');
    }

    registerMood(moodInstance) {
        this.moods.set(moodInstance.name, moodInstance);
    }

    setMood(moodName) {
        if (!this.moods.has(moodName)) {
            console.warn(`Mood ${moodName} not found, falling back to normal`);
            moodName = 'normal';
        }

        const newMood = this.moods.get(moodName);
        if (this.currentMood) this.currentMood.exit();

        this.currentMood = newMood;
        this.currentMood.enter();
    }

    // For Lip Sync
    updateVoiceLevel(volume) {
        if (this.currentMood && typeof this.currentMood.updateVoiceLevel === 'function') {
            this.currentMood.updateVoiceLevel(volume);
        }
    }

    // External API
    changeMood(mood) { this.setMood(mood); }
}
