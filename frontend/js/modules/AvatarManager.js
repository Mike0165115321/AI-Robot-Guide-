/**
 * # AvatarManager.js
 * Manages the Avatar's visual state, animation, and speech synchronization.
 * Separated from app.js for clarity and robustness.
 * 
 * [Optimization Log]
 * - Switched from Real-time AudioContext Analysis to Simulated Lip Sync
 * - Reason: Performance & Complexity reduction as per user request.
 * - Result: Smooth mouth animation with Zero overhead.
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
        // Append to DOM to ensure browser prioritizes it
        document.body.appendChild(this.audioPlayer);
        this.audioPlayer.style.display = 'none';

        this.audioQueue = [];
        this.isPlaying = false;

        // Simulation State
        this.animationFrameId = null;

        // Bind audio events
        this.audioPlayer.addEventListener('ended', () => this.onAudioEnded());
        this.audioPlayer.addEventListener('error', (e) => console.error("Audio Playback Error", e));

        // 🆕 Start Simulation on Play
        this.audioPlayer.addEventListener('play', () => {
            console.log("🎧 Audio Played: Starting Lip Sync Simulation");
            this.startSimulationLoop();
        });

        this.audioPlayer.addEventListener('pause', () => this.stopSimulationLoop());

        // Global State Callback
        this.onAudioStateChange = null;

        // 🔓 Simple Autoplay Unlocker
        this.hasUnlocked = false;
        const unlocker = () => {
            if (this.hasUnlocked) return;
            this.hasUnlocked = true;
            // Play empty
            this.audioPlayer.play().catch(() => { });
            console.log("🔓 Audio Unlocked");
            document.removeEventListener('click', unlocker);
            document.removeEventListener('touchstart', unlocker);
            document.removeEventListener('keydown', unlocker);
        };
        document.addEventListener('click', unlocker);
        document.addEventListener('touchstart', unlocker);
        document.addEventListener('keydown', unlocker);
    }

    // 🆕 Simulation Loop REMOVED (Avatar has built-in mouth animation)
    // The voiceData system was causing unnecessary processing overhead.
    startSimulationLoop() {
        // DISABLED - Avatar handles mouth animation internally
    }

    stopSimulationLoop() {
        // DISABLED - No-op
    }

    // 🆕 Local Visuals Only (High Frequency) - No Backend Spam
    sendLocalVisual(cmd) {
        const wrapper = document.querySelector('#avatar-wrapper iframe');
        if (wrapper && wrapper.contentWindow) {
            wrapper.contentWindow.postMessage(cmd, '*');
        }
    }

    // ==========================================
    // CORE COMMANDS
    // ==========================================

    sendCommand(cmd) {
        // 1. Send to Backend (only infrequent commands)
        avatarService.send(cmd);

        // 2. Send to Iframe
        this.sendLocalVisual(cmd);
    }

    setMood(mood) {
        console.log(`🎭 Avatar Mood: ${mood}`);
        stateManager.set('avatarMood', mood);
        // Special case: map setMood to changeMood for iframe
        let msg = { type: 'changeMood', mood: mood };
        avatarService.send({ type: 'setMood', mood: mood }); // Send original type to backend
        this.sendLocalVisual(msg);
    }

    changeSkin(skinName) {
        if (!skinName) return;
        console.log(`👕 Changing Skin: ${skinName}`);
        this.sendLocalVisual({ type: 'changeSkin', skin: skinName });
    }

    // ==========================================
    // SPEECH
    // ==========================================

    speak(text, mood = 'normal', lang = 'th', onComplete = null, interrupt = true) {
        if (!text) return;

        if (text.length < 100) uiManager.updateSpeech(text);

        if (interrupt) {
            this.stop();
        }

        // ✅ Word Boundary Chunking - ตัดที่ ~200 ตัวอักษรแต่ไม่ตัดกลางคำ
        const chunks = this._splitTextIntoChunks(text, 200);
        console.log(`📦 [TTS] แบ่งข้อความเป็น ${chunks.length} ก้อน`);

        // Push each chunk to queue
        chunks.forEach((chunk, index) => {
            const isLast = (index === chunks.length - 1);
            this.audioQueue.push({
                text: chunk,
                lang: lang,
                mood: mood,
                onComplete: isLast ? onComplete : null // Only call onComplete on last chunk
            });
        });

        if (!this.isPlaying) {
            this.processQueue();
        }
    }

    /**
     * 📦 Word Boundary Chunking
     * แบ่งข้อความเป็นก้อนๆ โดยไม่ตัดกลางคำ
     * - ตัดเมื่อความยาวเกิน threshold
     * - แต่จะเดินหน้าต่อจนเจอ space/เว้นวรรค
     * 
     * @param {string} text - ข้อความที่จะแบ่ง
     * @param {number} threshold - ขนาดขั้นต่ำต่อ chunk (~200)
     * @returns {string[]} - Array ของ chunks
     */
    _splitTextIntoChunks(text, threshold = 200) {
        if (!text || text.length <= threshold) {
            return [text];
        }

        const chunks = [];
        let currentChunk = '';

        // Split by spaces (works for both Thai and English)
        const words = text.split(/(\s+)/); // Keep whitespace in result

        for (const word of words) {
            currentChunk += word;

            // Check if we've passed the threshold
            if (currentChunk.length >= threshold) {
                // Push current chunk and reset
                const trimmed = currentChunk.trim();
                if (trimmed) {
                    chunks.push(trimmed);
                }
                currentChunk = '';
            }
        }

        // Push remaining text
        const remaining = currentChunk.trim();
        if (remaining) {
            chunks.push(remaining);
        }

        return chunks.length > 0 ? chunks : [text];
    }

    stop() {
        this.audioPlayer.pause();
        this.audioPlayer.currentTime = 0;
        this.audioQueue = [];
        this.isPlaying = false;
        this.setMood('normal');
        stateManager.set('isSpeaking', false);
        this.stopSimulationLoop();
        if (this.onAudioStateChange) this.onAudioStateChange(false);
    }

    async processQueue() {
        if (this.isPlaying || this.audioQueue.length === 0) return;

        this.isPlaying = true;
        const item = this.audioQueue.shift();

        try {
            // Fetch TTS
            console.log(`📥 Fetching TTS...`);
            const blob = await this.fetchTTS(item.text, item.lang);

            if (!blob || blob.size < 100) {
                this.isPlaying = false;
                this.processQueue();
                return;
            }

            const url = URL.createObjectURL(blob);
            this.audioPlayer.src = url;

            try {
                await this.audioPlayer.play();
                this.setMood('speaking');
                stateManager.set('isSpeaking', true);
                if (this.onAudioStateChange) this.onAudioStateChange(true);
                this.currentItem = item;
            } catch (playError) {
                console.warn("⚠️ Audio Autoplay Blocked", playError);
                this.isPlaying = false;
                this.processQueue();
            }

        } catch (e) {
            console.error("TTS Error:", e);
            this.isPlaying = false;
            this.processQueue();
        }
    }

    onAudioEnded() {
        this.isPlaying = false;
        this.stopSimulationLoop();

        if (this.currentItem && this.currentItem.onComplete) {
            this.currentItem.onComplete();
        }

        if (this.audioQueue.length > 0) {
            this.processQueue();
        } else {
            stateManager.set('isSpeaking', false);
            if (stateManager.get('isProcessing')) {
                this.setMood('thinking');
            } else {
                this.setMood('normal');
                this.sendCommand({ type: 'resumeIdle' });
            }
            if (this.onAudioStateChange) this.onAudioStateChange(false);
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
