/**
 * # AvatarManager.js
 * Manages the Avatar's visual state, animation, and speech synchronization.
 * Separated from app.js for clarity and robustness.
 */
import avatarService from '../services/avatarService.js';
import stateManager from './StateManager.js';
import uiManager from './UIManager.js';

class AvatarManager {
    constructor() {
        if (AvatarManager.instance) return AvatarManager.instance;
        AvatarManager.instance = this;

        // Internal Audio State
        this.audioPlayer = new Audio();
        this.audioQueue = [];
        this.isPlaying = false;

        // Bind audio events
        this.audioPlayer.addEventListener('ended', () => this.onAudioEnded());
        this.audioPlayer.addEventListener('error', (e) => console.error("Audio Playback Error", e));

        // 🔓 Autoplay Unlocker
        // Chrome/Edge blocks auth-play unless user interacts once.
        this.hasUnlocked = false;
        const unlocker = () => {
            if (this.hasUnlocked) return;
            this.hasUnlocked = true;
            // Play silent sound to unlock audio engine
            const silentCtx = new (window.AudioContext || window.webkitAudioContext)();
            silentCtx.resume().then(() => {
                console.log("🔓 Audio Engine Unlocked!");
                // Also try playing empty audio on the player
                this.audioPlayer.play().catch(() => { });
            });
            document.removeEventListener('click', unlocker);
            document.removeEventListener('touchstart', unlocker);
            document.removeEventListener('keydown', unlocker);
        };
        document.addEventListener('click', unlocker);
        document.addEventListener('touchstart', unlocker);
        document.addEventListener('keydown', unlocker);
    }

    // ==========================================
    // CORE COMMANDS
    // ==========================================

    /**
     * Send command to avatar iframe AND backend via service
     */
    sendCommand(cmd) {
        // 1. Send to Backend (for sync/logs)
        avatarService.send(cmd);

        // 2. Send to Iframe (Local Visuals)
        const wrapper = document.querySelector('#avatar-wrapper iframe');
        if (wrapper && wrapper.contentWindow) {
            // Map 'setMood' to 'changeMood' as used in avatar_enhanced.js
            let msg = { ...cmd };
            if (msg.type === 'setMood') msg.type = 'changeMood';

            wrapper.contentWindow.postMessage(msg, '*');
        }
    }

    /**
     * Set the avatar's mood/animation state
     * @param {string} mood - 'normal', 'happy', 'thinking', 'waving', 'speaking', 'worried'
     */
    setMood(mood) {
        console.log(`🎭 Avatar Mood: ${mood}`);
        stateManager.set('avatarMood', mood);
        this.sendCommand({ type: 'setMood', mood: mood });
    }

    /**
     * Change avatar skin
     * @param {string} skinName 
     */
    changeSkin(skinName) {
        if (!skinName) return;
        console.log(`👕 Changing Skin: ${skinName}`);

        const wrapper = document.querySelector('#avatar-wrapper iframe');
        if (wrapper && wrapper.contentWindow) {
            wrapper.contentWindow.postMessage({ type: 'changeSkin', skin: skinName }, '*');
        }
    }

    // ==========================================
    // SPEECH & LIP SYNC
    // ==========================================

    /**
     * Speak text with lip sync
     * @param {string} text - Text to speak
     * @param {string} mood - Mood during speech (default: 'speaking'/'normal')
     * @param {string} lang - Language code
     * @param {Function} onComplete - Callback when this specific text finishes
     * @param {boolean} interrupt - Whether to stop current speech immediately (default: true)
     */
    speak(text, mood = 'normal', lang = 'th', onComplete = null, interrupt = true) {
        if (!text) return;
        console.log(`🗣️ Speak Request: "${text.substring(0, 20)}..." (Interrupt: ${interrupt})`);

        // Visual Feedback (Bubble)
        if (text.length < 100) uiManager.updateSpeech(text);

        // Reset queue if urgent (interrupt)
        if (interrupt) {
            this.stop();
        }

        // Optimizing Speech: Chunking logic as per User Requirement
        // 1. Accumulate words (delimiters) until ~200 chars
        // 2. If limit hit, cut at the LAST delimiter (Space/Punctuation)
        // 3. Handle large text blocks carefully

        const cleanText = text;
        // Split by delimiters but keep them. 
        // regex: split by space, newline, or punctuation
        const rawEvents = cleanText.split(/([ \n.!?]+)/).filter(s => s.length > 0);

        const TARGET_CHUNK_LENGTH = 200;
        let chunks = [];
        let currentBuffer = '';

        for (const event of rawEvents) {
            // Case 1: Adding this word fits in the buffer
            if ((currentBuffer + event).length <= TARGET_CHUNK_LENGTH) {
                currentBuffer += event;
            }
            // Case 2: Buffer overflow
            else {
                // Flush the current buffer (it represents the "last logical stop" before overflow)
                if (currentBuffer.trim().length > 0) {
                    chunks.push(currentBuffer);
                    currentBuffer = '';
                }

                // Setup for next
                // Check if the NEW event is ITSELF larger than target (Giant text with no spaces)
                if (event.length > TARGET_CHUNK_LENGTH) {
                    // Force split the giant event
                    let temp = event;
                    while (temp.length > TARGET_CHUNK_LENGTH) {
                        chunks.push(temp.slice(0, TARGET_CHUNK_LENGTH));
                        temp = temp.slice(TARGET_CHUNK_LENGTH);
                    }
                    currentBuffer = temp; // Keep remainder
                } else {
                    currentBuffer = event;
                }
            }
        }

        // Push remaining buffer
        if (currentBuffer.trim().length > 0) {
            chunks.push(currentBuffer);
        }

        console.log(`📦 TTS Chunks (${chunks.length}):`, chunks);

        // 3. Queue chunks for playback
        chunks.forEach((chunk, index) => {
            this.audioQueue.push({
                text: chunk,
                lang: lang,
                mood: mood,
                isLast: index === chunks.length - 1,
                onComplete: index === chunks.length - 1 ? onComplete : null
            });
        });

        // If not playing, start. If playing and NOT interrupt, it will pick up next from queue.
        if (!this.isPlaying) {
            this.processQueue();
        }
    }

    stop() {
        this.audioPlayer.pause();
        this.audioPlayer.currentTime = 0;
        this.audioQueue = [];
        this.isPlaying = false;
        this.setMood('normal');
        stateManager.set('isSpeaking', false);
    }

    async processQueue() {
        if (this.isPlaying || this.audioQueue.length === 0) return;

        this.isPlaying = true;
        const item = this.audioQueue.shift();

        try {
            // Fetch TTS
            const blob = await this.fetchTTS(item.text, item.lang);

            // 🛡️ Validate Blob
            if (!blob || blob.size < 100) {
                console.warn("⚠️ TTS Blob too small or empty. Skipping.");
                this.isPlaying = false;
                this.processQueue();
                return;
            }

            const url = URL.createObjectURL(blob);
            this.audioPlayer.src = url;

            try {
                await this.audioPlayer.play();
                // Set mood to speaking (lip sync)
                this.setMood('speaking');
                stateManager.set('isSpeaking', true);
                // Store current item to handle 'onComplete' in 'onAudioEnded'
                this.currentItem = item;
            } catch (playError) {
                console.warn("⚠️ Audio Autoplay Blocked or Failed:", playError);

                if (playError.name === 'NotAllowedError') {
                    uiManager.showToastAlert({
                        type: 'warning',
                        title: 'คลิกเพื่อเปิดเสียง',
                        summary: 'Browser บล็อกเสียงอัตโนมัติ กรุณาคลิกที่หน้าจอ 1 ครั้งครับ'
                    });
                }

                this.isPlaying = false;
                this.processQueue(); // Skip to next or Retrying might loop, better skip
            }

        } catch (e) {
            console.error("TTS Fetch/Play Error:", e);
            this.isPlaying = false;
            this.processQueue(); // Try next
        }
    }


    onAudioEnded() {
        this.isPlaying = false;

        // Handle Item Completion
        if (this.currentItem && this.currentItem.onComplete) {
            this.currentItem.onComplete();
        }

        if (this.audioQueue.length > 0) {
            this.processQueue();
        } else {
            // Queue Finished
            stateManager.set('isSpeaking', false);

            // IMPORTANT: Return to appropriate state
            // If we are processing a request, go to 'thinking'
            if (stateManager.get('isProcessing')) {
                this.setMood('thinking');
            } else {
                this.setMood('normal');
                this.sendCommand({ type: 'resumeIdle' });
            }
        }
    }

    async fetchTTS(text, lang) {
        const response = await fetch('/api/chat/tts', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ text, language: lang })
        });
        if (!response.ok) throw new Error("TTS API Error");
        return await response.blob();
    }
}

const avatarManager = new AvatarManager();
export default avatarManager;
