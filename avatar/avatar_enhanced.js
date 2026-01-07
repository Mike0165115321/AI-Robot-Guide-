/**
 * avatar_enhanced.js - Enhanced Avatar Controller V1.0
 * Features: Mood Lighting, Idle Behaviors, Eye Tracking, Blinking
 * 
 * Usage: Include GSAP first, then this script. Call `NanAvatar.init()`.
 */

class NanAvatarController {
    constructor() {
        // DOM References
        this.container = null;
        this.face = null;
        this.eyes = [];
        this.leftArm = null;
        this.rightArm = null;
        this.mouth = null;

        // State
        this.currentMood = 'normal';
        this.isInitialized = false;
        this.isEyeTrackingEnabled = true;
        this.idleTimer = null;
        this.idleTimeout = 8000; // 8 seconds before idle behaviors
        this.blinkInterval = null;

        // Eye tracking params
        this.lastMove = { x: 0, y: 0 };
        this.maxEyeMove = 12;

        // Mood colors (CSS Variable override)
        this.moodColors = {
            normal: { eye: '#40c4ff', accent: '#00e5ff', glow: 'rgba(64, 196, 255, 0.5)' },
            speaking: { eye: '#4ade80', accent: '#22c55e', glow: 'rgba(74, 222, 128, 0.6)' },
            thinking: { eye: '#fbbf24', accent: '#f59e0b', glow: 'rgba(251, 191, 36, 0.5)' },
            listening: { eye: '#f472b6', accent: '#ec4899', glow: 'rgba(244, 114, 182, 0.5)' },
            happy: { eye: '#34d399', accent: '#10b981', glow: 'rgba(52, 211, 153, 0.6)' },
            curious: { eye: '#a78bfa', accent: '#8b5cf6', glow: 'rgba(167, 139, 250, 0.5)' },
            sleepy: { eye: '#64748b', accent: '#475569', glow: 'rgba(100, 116, 139, 0.3)' }
        };

        // Idle behavior animations
        this.idleBehaviors = [
            // () => this.lookAround(), // Disabled by user request
            () => this.yawn(),
            () => this.waveHand(),
            () => this.tiltHead(),
            // () => this.startRoaming() // Disabled by user request
        ];

        this.isRoaming = false;
    }

    init() {
        // Get DOM elements
        this.container = document.getElementById('robot-master-container');
        this.face = document.getElementById('robot-face');
        this.eyes = document.querySelectorAll('.robot-eye');
        this.leftArm = document.getElementById('left-arm');
        this.rightArm = document.getElementById('right-arm');
        this.mouth = document.querySelector('.robot-mouth');

        if (!this.container || !this.face || this.eyes.length === 0) {
            console.error('NanAvatar: Required DOM elements not found!');
            return;
        }

        // Bind events
        this.bindEvents();

        // Start loops
        this.startBlinkLoop();
        this.resetIdleTimer();

        // Set initial mood
        this.setMood('normal');

        // Roaming disabled by default (can call NanAvatar.enableRoaming() manually)
        // this.enableRoaming();

        this.isInitialized = true;
        console.log('✅ NanAvatar Enhanced Controller Ready!');
    }

    bindEvents() {
        // Mouse tracking for eyes
        document.addEventListener('mousemove', (e) => this.trackEyes(e));

        // Reset idle timer on any user activity
        ['mousemove', 'click', 'keydown', 'touchstart'].forEach(event => {
            document.addEventListener(event, () => this.resetIdleTimer());
        });
    }

    // ==========================================
    // MOOD LIGHTING SYSTEM
    // ==========================================
    setMood(mood) {
        const colors = this.moodColors[mood] || this.moodColors.normal;
        this.currentMood = mood;

        // Update CSS Variables
        document.documentElement.style.setProperty('--mood-eye-color', colors.eye);
        document.documentElement.style.setProperty('--mood-accent-color', colors.accent);
        document.documentElement.style.setProperty('--mood-glow', colors.glow);

        // Update face class for CSS animations
        const moodClasses = Object.keys(this.moodColors).map(m => `mood-${m}`);
        this.face.classList.remove(...moodClasses);
        this.face.classList.add(`mood-${mood}`);

        // Update arm animations
        this.updateArmState(mood);

        // Sync Mouth State (Fix: Ensure mouth moves when speaking)
        if (this.mouth) {
            if (mood === 'speaking') {
                this.mouth.classList.add('speaking');
                console.log('👄 Mouth: Added speaking class', this.mouth.classList);
            } else {
                this.mouth.classList.remove('speaking');
            }
        } else {
            console.warn('❌ Mouth element not found!');
        }

        // Eye tracking only for certain moods
        this.isEyeTrackingEnabled = ['normal', 'listening', 'curious', 'happy', 'speaking'].includes(mood);

        console.log(`🎨 Mood changed to: ${mood}`);
    }

    updateArmState(mood) {
        if (!this.leftArm || !this.rightArm) return;

        const armStates = ['arm-idle', 'arm-speaking', 'arm-thinking', 'arm-listening', 'arm-waving', 'arm-curious'];
        [this.leftArm, this.rightArm].forEach(arm => {
            arm.classList.remove(...armStates);
        });

        switch (mood) {
            case 'speaking':
                // User requested NO arm movement during speaking
                this.leftArm.classList.add('arm-idle');
                this.rightArm.classList.add('arm-idle');
                break;
            case 'thinking':
                this.leftArm.classList.add('arm-thinking');
                this.rightArm.classList.add('arm-thinking');
                break;
            case 'listening':
                this.leftArm.classList.add('arm-listening');
                this.rightArm.classList.add('arm-listening');
                break;
            case 'curious':
                this.leftArm.classList.add('arm-curious');
                this.rightArm.classList.add('arm-curious');
                break;
            default:
                this.leftArm.classList.add('arm-idle');
                this.rightArm.classList.add('arm-idle');
        }
    }

    // ==========================================
    // EYE TRACKING
    // ==========================================
    trackEyes(event) {
        if (!this.isEyeTrackingEnabled || !this.face) return;

        const rect = this.face.getBoundingClientRect();
        const centerX = rect.left + rect.width / 2;
        const centerY = rect.top + rect.height / 2;

        let moveX = (event.clientX - centerX) / 50;
        let moveY = (event.clientY - centerY) / 50;

        // Clamp values
        moveX = Math.max(-this.maxEyeMove, Math.min(this.maxEyeMove, moveX));
        moveY = Math.max(-this.maxEyeMove, Math.min(this.maxEyeMove, moveY));

        this.lastMove = { x: moveX, y: moveY };

        // Apply with GSAP if available, otherwise CSS
        if (typeof gsap !== 'undefined') {
            gsap.to(this.eyes, { x: moveX, y: moveY, duration: 0.4, ease: 'power3.out' });
        } else {
            this.eyes.forEach(eye => {
                eye.style.transform = `translate(${moveX}px, ${moveY}px)`;
            });
        }
    }

    // ==========================================
    // BLINKING
    // ==========================================
    startBlinkLoop() {
        this.stopBlinkLoop();

        const blink = () => {
            if (this.currentMood === 'sleepy' || this.currentMood === 'thinking') return;

            if (typeof gsap !== 'undefined') {
                gsap.to(this.eyes, {
                    scaleY: 0.1,
                    duration: 0.08,
                    yoyo: true,
                    repeat: 1,
                    ease: 'power2.inOut'
                });
            } else {
                this.eyes.forEach(eye => {
                    eye.classList.add('blink');
                    setTimeout(() => eye.classList.remove('blink'), 150);
                });
            }
        };

        // Random blink interval (3-6 seconds)
        this.blinkInterval = setInterval(blink, 3000 + Math.random() * 3000);
    }

    stopBlinkLoop() {
        if (this.blinkInterval) {
            clearInterval(this.blinkInterval);
            this.blinkInterval = null;
        }
    }

    // ==========================================
    // IDLE BEHAVIORS
    // ==========================================
    resetIdleTimer() {
        if (this.idleTimer) clearTimeout(this.idleTimer);

        // Don't do anything if idle is paused (during TTS/thinking)
        if (this.isIdlePaused) {
            return;
        }

        // Only reset to normal if mood was set by IDLE BEHAVIOR (not user click)
        // We track this with a flag
        if (this.isIdleMood) {
            this.setMood('normal');
            this.isIdleMood = false;
        }

        this.idleTimer = setTimeout(() => this.triggerIdleBehavior(), this.idleTimeout);
    }

    triggerIdleBehavior() {
        // Don't trigger if paused (during TTS playback)
        if (this.isIdlePaused) {
            return;
        }

        // Don't interrupt speaking or if validation is running
        if (this.currentMood === 'speaking' || this.currentMood === 'listening') {
            // Reschedule check
            this.idleTimer = setTimeout(() => this.triggerIdleBehavior(), 2000);
            return;
        }

        // Mark that this mood was set by idle behavior
        this.isIdleMood = true;

        // Pick random idle behavior
        const behavior = this.idleBehaviors[Math.floor(Math.random() * this.idleBehaviors.length)];
        behavior();

        // Schedule next idle behavior
        this.idleTimer = setTimeout(() => this.triggerIdleBehavior(), 5000 + Math.random() * 5000);
    }

    lookAround() {
        console.log('👀 Looking around...');
        this.setMood('curious');

        if (typeof gsap !== 'undefined') {
            const timeline = gsap.timeline();
            timeline
                .to(this.eyes, { x: -15, duration: 0.5, ease: 'power2.out' })
                .to(this.eyes, { x: 15, duration: 0.8, ease: 'power2.inOut' })
                .to(this.eyes, { x: 0, duration: 0.5, ease: 'power2.out' })
                .add(() => this.setMood('normal'));
        }
    }

    yawn() {
        console.log('🥱 Yawning...');
        this.setMood('sleepy');

        // Close eyes slowly
        if (typeof gsap !== 'undefined') {
            const timeline = gsap.timeline();
            timeline
                .to(this.eyes, { scaleY: 0.2, duration: 0.5, ease: 'power2.in' })
                .to(this.mouth, { scaleY: 1.5, scaleX: 1.2, opacity: 1, duration: 0.3 }, '<')
                .to({}, { duration: 1.5 }) // Hold yawn
                .to(this.eyes, { scaleY: 1, duration: 0.4, ease: 'power2.out' })
                .to(this.mouth, { scaleY: 1, scaleX: 1, opacity: 0, duration: 0.3 }, '<')
                .add(() => {
                    if (!this.isIdlePaused) this.setMood('normal');
                });
        }
    }

    waveHand() {
        console.log('👋 Waving...');

        if (typeof gsap !== 'undefined' && this.rightArm) {
            this.rightArm.classList.add('arm-waving');

            gsap.timeline()
                .to(this.rightArm, { rotation: -45, y: -30, duration: 0.3, ease: 'power2.out' })
                .to(this.rightArm, { rotation: -30, duration: 0.15 })
                .to(this.rightArm, { rotation: -45, duration: 0.15 })
                .to(this.rightArm, { rotation: -30, duration: 0.15 })
                .to(this.rightArm, { rotation: -45, duration: 0.15 })
                .to(this.rightArm, { rotation: 5, y: 0, duration: 0.4, ease: 'power2.in' })
                .add(() => this.rightArm.classList.remove('arm-waving'));
        }
    }

    tiltHead() {
        console.log('🤔 Tilting head...');
        this.setMood('curious');

        if (typeof gsap !== 'undefined' && this.container) {
            const head = this.container.querySelector('.robot-head');
            if (head) {
                gsap.timeline()
                    .to(head, { rotation: 8, duration: 0.4, ease: 'power2.out' })
                    .to({}, { duration: 1 })
                    .to(head, { rotation: 0, duration: 0.3, ease: 'power2.in' })
                    .add(() => {
                        if (!this.isIdlePaused) this.setMood('normal');
                    });
            }
        }
    }

    // ==========================================
    // ROAMING / WALKING
    // ==========================================
    enableRoaming() {
        if (!this.container) return;
        this.isRoaming = true;
        this.container.classList.add('roaming');
        console.log('🚶 Roaming enabled');
    }

    disableRoaming() {
        if (!this.container) return;
        this.isRoaming = false;
        this.container.classList.remove('roaming', 'walking');
        console.log('🛑 Roaming disabled');
    }

    startRoaming() {
        if (!this.isRoaming) {
            this.enableRoaming();
        }
        console.log('🚶 Started roaming...');
    }

    stopRoaming() {
        this.disableRoaming();
    }

    // ==========================================
    // PUBLIC API
    // ==========================================
    speak() {
        this.setMood('speaking');
        // Mouth animation handled by CSS
        if (this.mouth) this.mouth.classList.add('speaking');
    }

    stopSpeak() {
        // Only remove mouth animation, don't change mood
        if (this.mouth) this.mouth.classList.remove('speaking');
        // Only reset to normal if currently speaking
        if (this.currentMood === 'speaking') {
            this.setMood('normal');
        }
    }

    think() {
        this.setMood('thinking');
    }

    listen() {
        this.setMood('listening');
    }

    happy() {
        this.setMood('happy');
    }

    curious() {
        this.setMood('curious');
    }

    sleepy() {
        this.setMood('sleepy');
    }

    idle() {
        this.setMood('normal');
    }
}

// Global instance
const NanAvatar = new NanAvatarController();

// Auto-init when DOM ready
document.addEventListener('DOMContentLoaded', () => {
    NanAvatar.init();

    // 📩 Listen for commands from parent window (app.js)
    window.addEventListener('message', (event) => {
        const data = event.data;
        console.log('📩 [Avatar] Received message:', data);

        if (data.type === 'changeMood') {
            NanAvatar.setMood(data.mood);
        } else if (data.type === 'action') {
            if (typeof NanAvatar[data.action] === 'function') {
                NanAvatar[data.action]();
            } else {
                console.warn(`❌ Unknown action: ${data.action}`);
            }
        } else if (data.type === 'pauseIdle') {
            NanAvatar.isIdlePaused = true;
            console.log('⏸️ Avatar idle paused');
        } else if (data.type === 'resumeIdle') {
            NanAvatar.isIdlePaused = false;
            console.log('▶️ Avatar idle resumed');
            NanAvatar.resetIdleTimer();
        }
    });
});
