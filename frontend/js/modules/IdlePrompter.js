/**
 * ========================================
 * IdlePrompter.js (Logic Only)
 * ========================================
 * 
 * Purpose: Controls WHEN and HOW to trigger idle prompts.
 * Styling: /frontend/css/idle-prompter.css (Separate file)
 * 
 * Responsibilities:
 * - Timer management (when to show prompt)
 * - TTS speech triggering
 * - Adding/removing CSS class (no inline styles!)
 * 
 * Configuration:
 * - Timing: Change minDelay/maxDelay below
 * - Messages: Edit prompts array
 * - Animation: Edit /frontend/css/idle-prompter.css
 */

import avatarManager from './AvatarManager.js';
import stateManager from './StateManager.js';
import { languageManager } from './LanguageManager.js';

class IdlePrompter {
    constructor() {
        if (IdlePrompter.instance) return IdlePrompter.instance;
        IdlePrompter.instance = this;

        // === TIMING CONFIGURATION ===
        this.minDelay = 10000;  // 10 วินาที - รอขั้นต่ำก่อน prompt
        this.maxDelay = 15000;  // 15 วินาที - รอสูงสุดก่อน prompt
        this.cooldownAfterInteraction = 10000;  // 10 วินาที - หลัง user โต้ตอบ
        this.highlightDuration = 6000;  // 6 วินาที - ระยะเวลา highlight ปุ่ม

        // === STATE ===
        this.timerId = null;
        this.highlightTimer = null;
        this.isActive = false;
        this.lastInteraction = Date.now();
        this.lastUserActivity = Date.now();  // 🆕 Track scroll, mouse, keyboard
        this.currentPromptIndex = -1;
        this.activityListenersBound = false;  // 🆕 Prevent duplicate listeners
        this.highlightedEl = null;

        // === PROMPTS (Multi-Language) ===
        // key = targetIcon, value = speech per language
        this.prompts = [
            {
                targetIcon: "👋",
                speech: {
                    th: "อยากรู้จักน้องน่านใช่มั้ยคะ? กดตรงปุ่ม 👋รู้จักกับน้องน่าน ด้านซ้ายได้เลยค่ะ",
                    en: "Want to know me? Tap the 👋 Meet Nong Nan button on the left!",
                    ja: "私のことを知りたいですか？左の👋ノン・ナーンを知るボタンをタップしてね！",
                    zh: "想认识我吗？点击左边的👋认识Nong Nan按钮！",
                    ru: "Хотите познакомиться? Нажмите кнопку 👋 Знакомство слева!",
                    hi: "मुझसे मिलना चाहते हैं? बाईं ओर 👋 नोंग नान से मिलें बटन दबाएं!",
                    ms: "Nak kenal saya? Tekan butang 👋 Kenali Nong Nan di sebelah kiri!"
                }
            },
            {
                targetIcon: "📖",
                speech: {
                    th: "น้องน่านช่วยอะไรได้บ้างนะคะ? ลองกดปุ่ม 📖น้องน่านทำอะไรได้บ้าง ดูสิคะ",
                    en: "What can I help with? Tap the 📖 What can I do button!",
                    ja: "何のお手伝いができるかな？📖私にできることボタンをタップしてね！",
                    zh: "我能帮什么忙？点击📖我能做什么按钮看看！",
                    ru: "Чем могу помочь? Нажмите кнопку 📖 Что я умею!",
                    hi: "मैं क्या मदद कर सकती हूँ? 📖 मैं क्या कर सकती हूँ बटन दबाएं!",
                    ms: "Apa yang saya boleh bantu? Tekan butang 📖 Apa yang saya boleh lakukan!"
                }
            },
            {
                targetIcon: "⚙️",
                speech: {
                    th: "อยากรู้ว่าระบบน้องน่านเจ๋งแค่ไหนไหมคะ? กดตรงปุ่ม ⚙️ความสุดยอดของระบบน้องน่าน ได้เลยค่ะ",
                    en: "Want to know my capabilities? Tap the ⚙️ My Capabilities button!",
                    ja: "私の機能を知りたいですか？⚙️システムの強みボタンをタップしてね！",
                    zh: "想了解我的能力吗？点击⚙️系统的超能力按钮！",
                    ru: "Хотите узнать мои возможности? Нажмите ⚙️ Мои возможности!",
                    hi: "मेरी क्षमताएं जानना चाहते हैं? ⚙️ मेरी क्षमताएं बटन दबाएं!",
                    ms: "Nak tahu kehebatan saya? Tekan butang ⚙️ Kehebatan Sistem!"
                }
            },
            {
                targetIcon: "🎵",
                speech: {
                    th: "อยากฟังเพลงเพลินๆ ไหมคะ? กดปุ่ม 🎵ฟังเพลง ด้านขวาได้เลยค่ะ",
                    en: "Want to listen to some music? Tap the 🎵 Music button on the right!",
                    ja: "音楽を聴きたいですか？右側の🎵音楽ボタンをタップしてね！",
                    zh: "想听音乐吗？点击右边的🎵听音乐按钮！",
                    ru: "Хотите послушать музыку? Нажмите кнопку 🎵 Музыка справа!",
                    hi: "संगीत सुनना चाहते हैं? दाईं ओर 🎵 संगीत बटन दबाएं!",
                    ms: "Nak dengar lagu? Tekan butang 🎵 Dengar Lagu di sebelah kanan!"
                }
            },
            {
                targetIcon: "🧮",
                speech: {
                    th: "ต้องการคำนวณอะไรไหมคะ? กดปุ่ม 🧮คิดเลข ได้เลยค่ะ",
                    en: "Need to calculate something? Tap the 🧮 Calculator button!",
                    ja: "計算が必要ですか？🧮電卓ボタンをタップしてね！",
                    zh: "需要计算吗？点击🧮计算器按钮！",
                    ru: "Нужно что-то посчитать? Нажмите кнопку 🧮 Калькулятор!",
                    hi: "कुछ गणना करनी है? 🧮 कैलकुलेटर बटन दबाएं!",
                    ms: "Perlu kira sesuatu? Tekan butang 🧮 Kalkulator!"
                }
            },
        ];
    }

    // ==========================================
    // PUBLIC API
    // ==========================================

    /**
     * Start the idle prompter
     */
    start() {
        if (this.isActive) return;
        this.isActive = true;
        this.bindActivityListeners();  // 🆕 Track user activity
        this.scheduleNextPrompt();
        console.log('▶️ [IdlePrompter] Started');
    }

    /**
     * 🆕 Bind listeners for user activity (scroll, mouse, keyboard)
     */
    bindActivityListeners() {
        if (this.activityListenersBound) return;
        this.activityListenersBound = true;

        const updateActivity = () => {
            this.lastUserActivity = Date.now();
        };

        // Debounced activity tracker
        let activityTimeout = null;
        const debouncedActivity = () => {
            if (activityTimeout) clearTimeout(activityTimeout);
            activityTimeout = setTimeout(updateActivity, 100);
        };

        // Listen to common user activities
        document.addEventListener('scroll', debouncedActivity, { passive: true });
        document.addEventListener('mousemove', debouncedActivity, { passive: true });
        document.addEventListener('keydown', debouncedActivity, { passive: true });
        document.addEventListener('click', updateActivity, { passive: true });
        document.addEventListener('touchstart', debouncedActivity, { passive: true });

        console.log('👂 [IdlePrompter] Activity listeners bound');
    }

    /**
     * Stop the idle prompter
     */
    stop() {
        this.isActive = false;
        this.clearTimers();
        this.removeHighlight();
        console.log('⏹️ [IdlePrompter] Stopped');
    }

    /**
     * Reset timer (call when user interacts)
     */
    reset() {
        this.lastInteraction = Date.now();
        this.removeHighlight();

        if (this.isActive) {
            this.clearTimers();
            this.scheduleNextPrompt();
        }
    }

    // ==========================================
    // PRIVATE METHODS
    // ==========================================

    /**
     * Clear all active timers
     */
    clearTimers() {
        if (this.timerId) {
            clearTimeout(this.timerId);
            this.timerId = null;
        }
        if (this.highlightTimer) {
            clearTimeout(this.highlightTimer);
            this.highlightTimer = null;
        }
    }

    /**
     * Schedule the next prompt
     */
    scheduleNextPrompt() {
        if (!this.isActive) return;

        const delay = this.minDelay + Math.random() * (this.maxDelay - this.minDelay);

        this.timerId = setTimeout(() => {
            this.triggerPrompt();
        }, delay);
    }

    /**
     * Check if system is truly idle (no TTS, no processing, no animation)
     * @returns {boolean} true if safe to trigger prompt
     */
    isSystemIdle() {
        // 1. Check if TTS is playing or queued
        if (avatarManager.isSpeaking) {
            console.log('⏸️ [IdlePrompter] Skipped: Avatar is speaking');
            return false;
        }

        // 2. Check if audio queue has items
        if (avatarManager.audioQueue && avatarManager.audioQueue.length > 0) {
            console.log('⏸️ [IdlePrompter] Skipped: Audio queue not empty');
            return false;
        }

        // 3. Check if audio player is currently playing
        if (avatarManager.audioPlayer && !avatarManager.audioPlayer.paused) {
            console.log('⏸️ [IdlePrompter] Skipped: Audio player is playing');
            return false;
        }

        // 4. Check if system is processing a request
        if (stateManager.get('isProcessing')) {
            console.log('⏸️ [IdlePrompter] Skipped: System is processing');
            return false;
        }

        // 5. Check if still in cooldown after last interaction
        if (Date.now() - this.lastInteraction < this.cooldownAfterInteraction) {
            console.log('⏸️ [IdlePrompter] Skipped: Still in cooldown');
            return false;
        }

        // 6. Check avatar mood (don't interrupt speaking, thinking, listening)
        const currentMood = stateManager.get('avatarMood');
        const busyMoods = ['speaking', 'thinking', 'listening'];
        if (busyMoods.includes(currentMood)) {
            console.log(`⏸️ [IdlePrompter] Skipped: Avatar mood is "${currentMood}"`);
            return false;
        }

        // 🆕 7. Check if presentation panel is visible (user is reading)
        const panel = document.querySelector('.presentation-panel');
        if (panel && panel.classList.contains('visible')) {
            console.log('⏸️ [IdlePrompter] Skipped: Presentation panel is visible');
            return false;
        }

        // 🆕 8. Check recent user activity (scroll, mouse, keyboard)
        const activityCooldown = 5000;  // 5 seconds after last activity
        if (Date.now() - this.lastUserActivity < activityCooldown) {
            console.log('⏸️ [IdlePrompter] Skipped: Recent user activity detected');
            return false;
        }

        return true;
    }

    /**
     * Trigger the idle prompt (TTS + highlight)
     */
    triggerPrompt() {
        // Safety checks
        if (!this.isActive) return;

        // ✅ Comprehensive idle check
        if (!this.isSystemIdle()) {
            // Retry later
            this.scheduleNextPrompt();
            return;
        }

        console.log('✅ [IdlePrompter] System is idle, triggering prompt...');

        // Pick random prompt (different from last)
        const prompt = this.pickRandomPrompt();

        // Get current language (with fallback to Thai)
        const currentLang = languageManager.getCurrentLanguage() || 'th';
        const speechText = prompt.speech[currentLang] || prompt.speech['th'];

        // 1. Speak via TTS (in current language)
        avatarManager.speak(speechText, 'happy', currentLang);

        // 2. Highlight the target button
        this.highlightButton(prompt.targetIcon);

        // Schedule next prompt
        this.scheduleNextPrompt();
    }

    /**
     * Pick a random prompt (different from the last one)
     */
    pickRandomPrompt() {
        let newIndex;
        do {
            newIndex = Math.floor(Math.random() * this.prompts.length);
        } while (newIndex === this.currentPromptIndex && this.prompts.length > 1);

        this.currentPromptIndex = newIndex;
        return this.prompts[newIndex];
    }

    /**
     * Highlight a button by its icon (2-phase smooth animation)
     * @param {string} targetIcon - The emoji icon of the button to highlight
     */
    highlightButton(targetIcon) {
        const chips = document.querySelectorAll('.quick-chip');

        for (const chip of chips) {
            const iconEl = chip.querySelector('.quick-chip-icon');

            if (iconEl?.textContent === targetIcon) {
                this.highlightedEl = chip;

                // Phase 1: Add starting class (disables animation, sets up transition)
                chip.classList.add('idle-starting');

                // Phase 2: After browser paint, add highlight class (triggers transition)
                requestAnimationFrame(() => {
                    requestAnimationFrame(() => {
                        chip.classList.add('idle-highlight');
                        chip.classList.remove('idle-starting');
                    });
                });

                // Remove highlight after duration
                this.highlightTimer = setTimeout(() => {
                    this.removeHighlight();
                }, this.highlightDuration);

                break;
            }
        }
    }

    /**
     * Remove highlight with smooth return animation
     */
    removeHighlight() {
        if (this.highlightTimer) {
            clearTimeout(this.highlightTimer);
            this.highlightTimer = null;
        }

        if (this.highlightedEl) {
            const chip = this.highlightedEl;

            // Step 1: Remove highlight, add returning class (starts transition down)
            chip.classList.remove('idle-highlight');
            chip.classList.remove('idle-starting');
            chip.classList.add('idle-returning');

            // Step 2: After transition completes, swap to idle-done (prevents slideInLeft restart)
            // Match CSS --idle-animation-down (1.5s = 1500ms)
            setTimeout(() => {
                chip.classList.remove('idle-returning');
                chip.classList.add('idle-done'); // Keep this permanently!
            }, 1600);

            this.highlightedEl = null;
        }
    }
}

// Singleton export
const idlePrompter = new IdlePrompter();
export default idlePrompter;
