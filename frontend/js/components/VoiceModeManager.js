/**
 * # Voice Mode Manager
 * จัดการการสลับโหมดเสียงและการบันทึกเสียง
 * 
 * @example
 * import { voiceModeManager } from './components/VoiceModeManager.js';
 * voiceModeManager.init({
 *     onAudioSend: (audioBlob) => chatService.sendAudio(audioBlob),
 *     onStatusUpdate: (status) => console.log(status)
 * });
 */

import { $ } from '../utils/dom.js';
import { speechService } from '../services/speechService.js';

class VoiceModeManager {
    constructor() {
        this.isVoiceMode = false;
        this.isRecording = false;

        // Element references
        this.elements = {
            textMode: null,
            voiceMode: null,
            rainbowBtn: null,
            voiceStatus: null,
            stopBtn: null
        };

        // Callbacks
        this.callbacks = {
            onAudioSend: null,
            onStatusUpdate: null,
            onModeChange: null,
            showLoading: null,
            hideLoading: null
        };
    }

    /**
     * Initialize Voice Mode Manager
     * @param {Object} options - Configuration options
     */
    init(options = {}) {
        this.callbacks = { ...this.callbacks, ...options };
        this._cacheElements();
        this._bindEvents();
        console.log('🎤 VoiceModeManager: Initialized');
    }

    /**
     * Cache DOM elements
     * @private
     */
    _cacheElements() {
        this.elements = {
            textMode: $('#text-mode'),
            voiceMode: $('#voice-mode'),
            rainbowBtn: $('#btn-voice-toggle'),
            voiceStatus: $('.voice-status'),
            stopBtn: $('#btn-stop-voice')
        };
    }

    /**
     * Bind event listeners
     * @private
     */
    _bindEvents() {
        const { rainbowBtn, stopBtn } = this.elements;

        if (rainbowBtn) {
            rainbowBtn.addEventListener('click', () => this.toggle());
        }

        if (stopBtn) {
            stopBtn.addEventListener('click', () => this.stop());
        }
    }

    /**
     * Toggle voice mode on/off
     */
    toggle() {
        if (this.isVoiceMode) {
            this._switchToTextMode();
        } else {
            this._switchToVoiceMode();
        }
    }

    /**
     * Force stop voice mode
     */
    stop() {
        this._switchToTextMode();
    }

    /**
     * Pause recording (e.g. while AI is speaking)
     */
    pauseRecording() {
        if (this.isRecording) {
            console.log('⏸️ VoiceModeManager: Pausing VAD for TTS...');
            speechService.stopVAD(true); // Treat as interrupted so it doesn't process audio
            this.isRecording = false;
        }
    }

    /**
     * Resume recording (e.g. after AI finishes speaking)
     */
    resumeRecording() {
        if (this.isVoiceMode && !this.isRecording) {
            console.log('▶️ VoiceModeManager: Resuming VAD after TTS...');
            this._startRecording();
        }
    }

    /**
     * Switch to Text Mode
     * @private
     */
    _switchToTextMode() {
        const { textMode, voiceMode, rainbowBtn } = this.elements;

        if (textMode) textMode.classList.remove('hidden');
        if (voiceMode) voiceMode.classList.add('hidden');
        if (rainbowBtn) rainbowBtn.classList.remove('active');

        // Stop VAD
        speechService.stopVAD(true);
        this.isVoiceMode = false;
        this.isRecording = false;

        this._updateStatus('หยุดฟังแล้วค่ะ');
        this.callbacks.onModeChange?.('text');

        console.log('🔤 VoiceModeManager: Switched to Text Mode');
    }

    /**
     * Switch to Voice Mode and start recording
     * @private
     */
    async _switchToVoiceMode() {
        const { textMode, voiceMode, rainbowBtn } = this.elements;

        if (textMode) textMode.classList.add('hidden');
        if (voiceMode) voiceMode.classList.remove('hidden');
        if (rainbowBtn) rainbowBtn.classList.add('active');

        this.isVoiceMode = true;
        this.callbacks.onModeChange?.('voice');

        console.log('🎤 VoiceModeManager: Switched to Voice Mode');

        // Start recording
        await this._startRecording();
    }

    /**
     * Start voice recording with VAD
     * @private
     */
    async _startRecording() {
        this._updateStatus('กำลังเตรียมไมค์... 🎤');

        const success = await speechService.startVAD({
            onStatusUpdate: (status) => {
                this._updateStatus(status);
                this.callbacks.onStatusUpdate?.(status);
            },
            onSpeechEnd: async (audioBlob) => {
                await this._handleSpeechEnd(audioBlob);
            },
            onVolumeChange: (volume) => {
                // Optional: Update volume indicator
                this._updateVolumeIndicator(volume);
            }
        });

        if (success) {
            this.isRecording = true;
            this._updateStatus('กำลังฟัง... 👂');
        } else {
            this._updateStatus('เปิดไมค์ไม่ได้ค่ะ 🎤❌');
            // Switch back to text mode after error
            setTimeout(() => this._switchToTextMode(), 2000);
        }
    }

    /**
     * Handle when speech ends (silence detected)
     * @private
     * @param {Blob} audioBlob - Recorded audio
     */
    async _handleSpeechEnd(audioBlob) {
        this._updateStatus('กำลังประมวลผล... 🔄');
        this.callbacks.showLoading?.();

        try {
            if (this.callbacks.onAudioSend) {
                const result = await this.callbacks.onAudioSend(audioBlob);

                if (!result?.success) {
                    this._updateStatus('ขอโทษค่ะ ฟังไม่ทัน 😓');
                }
            }
        } catch (err) {
            console.error('VoiceModeManager: Error sending audio', err);
            this._updateStatus('มีปัญหาในการส่งเสียงค่ะ 🔌');
        } finally {
            this.callbacks.hideLoading?.();
            // 🛑 ไม่สลับกลับ Text Mode อัตโนมัติแล้ว เพื่อให้คุยต่อเนื่องได้
            // setTimeout(() => this._switchToTextMode(), 1500);

            // 🛑 REMOVED: Auto-resume immediately after text response.
            // We now wait for TTS to finish (handled by app.js callbacks).
            // if (this.isVoiceMode) {
            //     console.log('🎤 VoiceModeManager: Resuming listening...');
            //     this._startRecording();
            // }
        }
    }

    /**
     * Update voice status text
     * @private
     * @param {string} status - Status message
     */
    _updateStatus(status) {
        const { voiceStatus } = this.elements;
        if (voiceStatus) {
            voiceStatus.textContent = status;
        }
    }

    /**
     * Update volume indicator (optional)
     * @private
     * @param {number} volume - Volume level 0-1
     */
    _updateVolumeIndicator(volume) {
        // Could add visual volume indicator here
        // For now, just log high volume
        if (volume > 0.5) {
            console.log('🔊 High volume detected:', volume);
        }
    }

    /**
     * Check if currently in voice mode
     * @returns {boolean}
     */
    isActive() {
        return this.isVoiceMode;
    }

    /**
     * Check if currently recording
     * @returns {boolean}
     */
    isCurrentlyRecording() {
        return this.isRecording;
    }
}

// Export singleton instance
export const voiceModeManager = new VoiceModeManager();
export default voiceModeManager;
