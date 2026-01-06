/**
 * # Speech Service
 * 
 * บริการจัดการเสียง รองรับ 2 โหมด:
 * 1. Simple Mode - กดเริ่ม/กดหยุด (เหมาะกับ UI แบบปุ่ม toggle)
 * 2. VAD Mode - ตรวจจับเสียงอัตโนมัติ (หยุดเองเมื่อเงียบ)
 * 
 * @example
 * // Simple Mode
 * await speechService.startRecording();
 * const blob = await speechService.stopRecording();
 * 
 * // VAD Mode
 * speechService.startVAD({
 *     onSpeechEnd: (blob) => sendToBackend(blob),
 *     onStatusUpdate: (status) => updateUI(status)
 * });
 */

import { VoiceHandler } from './voiceHandler.js';

class SpeechService {
    constructor() {
        // Simple Mode
        this.mediaRecorder = null;
        this.audioChunks = [];
        this.stream = null;

        // VAD Mode
        this.voiceHandler = null;
        this.isVADMode = false;
    }

    // ==========================================
    // SIMPLE MODE (กดเริ่ม/กดหยุด)
    // ==========================================

    /**
     * เริ่มบันทึกเสียง (Simple Mode)
     */
    async startRecording() {
        try {
            this.stream = await navigator.mediaDevices.getUserMedia({
                audio: {
                    noiseSuppression: true,
                    echoCancellation: true
                }
            });

            // หา mimeType ที่รองรับ
            const mimeTypes = ['audio/webm;codecs=opus', 'audio/ogg;codecs=opus', 'audio/webm'];
            const supportedMimeType = mimeTypes.find(type => MediaRecorder.isTypeSupported(type)) || 'audio/webm';

            this.mediaRecorder = new MediaRecorder(this.stream, {
                mimeType: supportedMimeType
            });
            this.audioChunks = [];

            this.mediaRecorder.ondataavailable = (event) => {
                if (event.data.size > 0) {
                    this.audioChunks.push(event.data);
                }
            };

            this.mediaRecorder.start();
            console.log('🎤 SpeechService: Recording started (Simple Mode)');
            return true;
        } catch (err) {
            console.error('🎤 SpeechService: Microphone Error', err);
            return false;
        }
    }

    /**
     * หยุดบันทึกเสียง (Simple Mode)
     * @returns {Promise<Blob|null>}
     */
    stopRecording() {
        return new Promise((resolve) => {
            if (!this.mediaRecorder) return resolve(null);

            this.mediaRecorder.onstop = () => {
                const audioBlob = new Blob(this.audioChunks, { type: 'audio/webm' });
                this._stopStream();
                console.log(`🎤 SpeechService: Recording stopped. Size: ${audioBlob.size} bytes`);
                resolve(audioBlob);
            };

            this.mediaRecorder.stop();
        });
    }

    // ==========================================
    // VAD MODE (ตรวจจับเสียงอัตโนมัติ)
    // ==========================================

    /**
     * เริ่ม Voice Activity Detection
     * @param {Object} callbacks
     * @param {Function} callbacks.onSpeechEnd - เรียกเมื่อพูดจบ (ได้รับ audio blob)
     * @param {Function} callbacks.onStatusUpdate - เรียกเมื่อสถานะเปลี่ยน
     * @param {Function} callbacks.onVolumeChange - เรียกเมื่อระดับเสียงเปลี่ยน
     * @param {Object} options - VAD configuration
     */
    async startVAD(callbacks = {}, options = {}) {
        // สร้าง VoiceHandler ใหม่
        this.voiceHandler = new VoiceHandler(callbacks, options);
        this.isVADMode = true;

        const success = await this.voiceHandler.start();
        if (!success) {
            this.isVADMode = false;
            this.voiceHandler = null;
        }
        return success;
    }

    /**
     * หยุด VAD
     * @param {boolean} interrupted - true ถ้าต้องการยกเลิก (ไม่ส่ง audio)
     */
    stopVAD(interrupted = false) {
        if (this.voiceHandler) {
            this.voiceHandler.stop(interrupted);
            this.voiceHandler = null;
        }
        this.isVADMode = false;
    }

    /**
     * Get VAD state
     */
    getVADState() {
        if (this.voiceHandler) {
            return this.voiceHandler.getState();
        }
        return { isListening: false, isSpeaking: false, volume: 0 };
    }

    // ==========================================
    // HELPERS
    // ==========================================

    /**
     * หยุด media stream
     * @private
     */
    _stopStream() {
        if (this.stream) {
            this.stream.getTracks().forEach(track => track.stop());
            this.stream = null;
        }
    }

    /**
     * เล่นเสียงจาก URL
     */
    playAudio(url) {
        const audio = new Audio(url);
        return audio.play();
    }

    /**
     * เล่นเสียงจาก Blob
     */
    playBlob(blob) {
        const url = URL.createObjectURL(blob);
        const audio = new Audio(url);
        audio.onended = () => URL.revokeObjectURL(url);
        return audio.play();
    }

    /**
     * เช็คว่ากำลังบันทึกอยู่หรือไม่
     */
    isRecording() {
        if (this.isVADMode) {
            return this.voiceHandler?.isListening || false;
        }
        return this.mediaRecorder?.state === 'recording';
    }
}

export const speechService = new SpeechService();
export default speechService;
