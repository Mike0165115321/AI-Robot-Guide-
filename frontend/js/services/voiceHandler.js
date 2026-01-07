/**
 * # Voice Handler - Voice Activity Detection (VAD)
 * 
 * ระบบตรวจจับเสียงอัตโนมัติ
 * - ตรวจจับเมื่อผู้ใช้เริ่มพูด
 * - ตรวจจับเมื่อผู้ใช้หยุดพูด (silence detection)
 * - ส่ง audio blob กลับเมื่อพูดจบ
 * 
 * @example
 * const handler = new VoiceHandler({
 *     onStatusUpdate: (status) => console.log(status),
 *     onSpeechEnd: (blob) => sendToBackend(blob)
 * });
 * await handler.start();
 * // ... user speaks ...
 * // onSpeechEnd จะถูกเรียกอัตโนมัติเมื่อหยุดพูด
 */

// =============================================
// DEFAULT CONFIGURATION
// =============================================
const DEFAULT_CONFIG = {
    // Noise/Speech Detection
    NOISE_FLOOR: 0.01,            // เพิ่ม Noise floor ให้สูงขึ้นนิดหน่อย
    SPEECH_THRESHOLD: 0.25,       // ลด Threshold ลงให้พอดีกับ Amplification ใหม่
    AMPLIFICATION: 2.0,           // ⚠️ ลดลงจาก 45 เพื่อแก้ปัญหา volume สูงเกินไป
    SILENCE_DELAY_MS: 1500,       // รอ 1.5 วินาทีเพื่อให้แน่ใจว่าจบประโยคจริง
    SPEECH_CONFIRMATION_FRAMES: 5, // เพิ่ม frames เพื่อความชัวร์ว่าเป็นเสียงพูดจริง
    MIN_BLOB_SIZE_BYTES: 1000,    // ขนาดไฟล์เสียงขั้นต่ำ (ป้องกันเสียงสั้นเกินไป)
    SMOOTHING_FACTOR: 0.2,        // ค่า smoothing สำหรับ volume
    MAX_RECORDING_MS: 15000       // บันทึกได้สูงสุด 15 วินาที
};

// =============================================
// VOICE HANDLER CLASS
// =============================================
export class VoiceHandler {
    /**
     * @param {Object} callbacks - Callback functions
     * @param {Function} callbacks.onStatusUpdate - เรียกเมื่อสถานะเปลี่ยน
     * @param {Function} callbacks.onSpeechEnd - เรียกเมื่อพูดจบ (ได้รับ audio blob)
     * @param {Function} callbacks.onVolumeChange - เรียกเมื่อระดับเสียงเปลี่ยน (optional)
     * @param {Object} options - Configuration options
     */
    constructor(callbacks = {}, options = {}) {
        // Callbacks
        this.callbacks = {
            onStatusUpdate: () => { },
            onSpeechEnd: () => { },
            onVolumeChange: () => { },
            ...callbacks
        };

        // Configuration
        this.config = { ...DEFAULT_CONFIG, ...options };

        // State
        this.isListening = false;
        this.isSpeaking = false;
        this.smoothedVolume = 0.0;
        this.speechFrameCount = 0;
        this.wasInterrupted = false;

        // Audio Components
        this.audioContext = null;
        this.mediaStream = null;
        this.mediaRecorder = null;
        this.analyser = null;
        this.dataArray = null;
        this.audioChunks = [];

        // Timeouts
        this.silenceTimeout = null;
        this.recordingTimeout = null;

        // Debug
        this.lastLogTime = 0;
    }

    /**
     * เริ่มฟังเสียง
     */
    async start() {
        if (this.isListening) {
            console.warn('⚠️ VoiceHandler: Already listening');
            return false;
        }

        try {
            // Create or resume AudioContext
            if (!this.audioContext) {
                this.audioContext = new (window.AudioContext || window.webkitAudioContext)();
            }
            if (this.audioContext.state === 'suspended') {
                await this.audioContext.resume();
            }

            // Get microphone access
            this.mediaStream = await navigator.mediaDevices.getUserMedia({
                audio: {
                    noiseSuppression: true,
                    echoCancellation: true,
                    autoGainControl: true
                }
            });

            // Setup audio analyser for VAD
            const source = this.audioContext.createMediaStreamSource(this.mediaStream);
            this.analyser = this.audioContext.createAnalyser();
            this.analyser.fftSize = 256;
            source.connect(this.analyser);
            this.dataArray = new Uint8Array(this.analyser.frequencyBinCount);

            // Setup MediaRecorder
            const mimeTypes = ['audio/webm;codecs=opus', 'audio/ogg;codecs=opus', 'audio/webm'];
            const supportedMimeType = mimeTypes.find(type => MediaRecorder.isTypeSupported(type));

            if (!supportedMimeType) {
                this.callbacks.onStatusUpdate('เบราว์เซอร์ไม่รองรับการอัดเสียง');
                return false;
            }

            this.mediaRecorder = new MediaRecorder(this.mediaStream, {
                mimeType: supportedMimeType,
                audioBitsPerSecond: 128000
            });

            this._setupRecorderEvents();

            // Start listening
            this.isListening = true;
            this.smoothedVolume = 0.0;
            this.speechFrameCount = 0;
            this.callbacks.onStatusUpdate('กำลังฟัง...');
            this._runDetectionLoop();

            // Max recording timeout
            this.recordingTimeout = setTimeout(() => {
                console.log('⏱️ Max recording time reached');
                this.stop(false);
            }, this.config.MAX_RECORDING_MS);

            console.log('🎤 VoiceHandler: Started');
            return true;

        } catch (err) {
            console.error('🎤 VoiceHandler: Microphone access error', err);
            this.callbacks.onStatusUpdate('ไม่สามารถเข้าถึงไมโครโฟน');
            return false;
        }
    }

    /**
     * หยุดฟังเสียง
     * @param {boolean} interrupted - true ถ้าถูกขัดจังหวะ (ไม่ส่ง audio)
     */
    stop(interrupted = false) {
        if (!this.isListening) return;

        this.wasInterrupted = interrupted;
        this.isListening = false;

        // Stop recording
        if (this.mediaRecorder?.state === 'recording') {
            this.mediaRecorder.stop();
        }

        // Stop microphone
        this.mediaStream?.getTracks().forEach(track => track.stop());
        this.mediaStream = null;

        // Clear timeouts
        clearTimeout(this.silenceTimeout);
        this.silenceTimeout = null;
        clearTimeout(this.recordingTimeout);
        this.recordingTimeout = null;

        this.callbacks.onStatusUpdate('หยุดฟัง');
        console.log('🎤 VoiceHandler: Stopped');
    }

    /**
     * Setup MediaRecorder events
     * @private
     */
    _setupRecorderEvents() {
        this.mediaRecorder.ondataavailable = (event) => {
            if (event.data.size > 0) {
                this.audioChunks.push(event.data);
            }
        };

        this.mediaRecorder.onstop = () => {
            console.log('🛑 MediaRecorder Stopped');

            if (this.wasInterrupted) {
                console.warn('⚠️ Recording was interrupted (discarding)');
                this.audioChunks = [];
                this.wasInterrupted = false;
                return;
            }

            const audioBlob = new Blob(this.audioChunks, { type: 'audio/webm' });
            console.log(`📦 Audio Blob Created. Size: ${audioBlob.size} bytes`);
            this.audioChunks = [];

            if (audioBlob.size > this.config.MIN_BLOB_SIZE_BYTES) {
                console.log('✅ Sending valid audio blob...');
                this.callbacks.onSpeechEnd(audioBlob);
            } else {
                console.warn(`⚠️ Audio too short (${audioBlob.size} < ${this.config.MIN_BLOB_SIZE_BYTES})`);
                this.callbacks.onStatusUpdate('เสียงสั้นไป ลองใหม่นะคะ');
            }
        };
    }

    /**
     * VAD Detection Loop
     * @private
     */
    _runDetectionLoop() {
        if (!this.isListening) return;
        requestAnimationFrame(() => this._runDetectionLoop());

        this.analyser.getByteTimeDomainData(this.dataArray);

        // Calculate RMS volume
        let sumSquares = 0.0;
        for (const amplitude of this.dataArray) {
            const normalized = (amplitude / 128.0) - 1.0;
            sumSquares += normalized * normalized;
        }
        let rawVolume = Math.sqrt(sumSquares / this.dataArray.length);

        // Apply noise floor
        if (rawVolume < this.config.NOISE_FLOOR) rawVolume = 0;

        // Amplify and smooth
        const amplifiedVolume = rawVolume * this.config.AMPLIFICATION;
        this.smoothedVolume = this.smoothedVolume * this.config.SMOOTHING_FACTOR +
            amplifiedVolume * (1 - this.config.SMOOTHING_FACTOR);

        // Notify volume change (for UI visualization)
        this.callbacks.onVolumeChange(this.smoothedVolume);

        // Speech detection
        if (this.smoothedVolume > this.config.SPEECH_THRESHOLD) {
            this.speechFrameCount++;
            if (this.speechFrameCount >= this.config.SPEECH_CONFIRMATION_FRAMES) {
                if (!this.isSpeaking) {
                    this.isSpeaking = true;
                    console.log('🗣️ Speech STARTED');
                    if (this.mediaRecorder.state === 'inactive') {
                        this.mediaRecorder.start();
                    }
                    this.callbacks.onStatusUpdate('รับฟังอยู่...');
                }
                clearTimeout(this.silenceTimeout);
                this.silenceTimeout = null;
            }
        } else {
            this.speechFrameCount = 0;
            if (this.isSpeaking && this.silenceTimeout === null) {
                console.log('🤫 Silence detected, starting timeout...');
                this.silenceTimeout = setTimeout(() => {
                    if (this.mediaRecorder?.state === 'recording') {
                        this.mediaRecorder.stop();
                    }
                    this.isSpeaking = false;
                    this.silenceTimeout = null;
                    console.log('⏹️ Speech ENDED (Silence)');
                }, this.config.SILENCE_DELAY_MS);
            }
        }

        // Debug logging (throttled)
        this._debugLog();
    }

    /**
     * Debug log (throttled to every 200ms)
     * @private
     */
    _debugLog() {
        const now = Date.now();
        if (now - this.lastLogTime > 200) {
            const vol = this.smoothedVolume.toFixed(2);
            const barLength = Math.min(20, Math.floor(this.smoothedVolume * 10));
            const bar = '█'.repeat(barLength).padEnd(20, '░');
            console.log(`🎤 Level: [${bar}] ${vol} ${this.isSpeaking ? '(Speaking)' : ''}`);
            this.lastLogTime = now;
        }
    }

    /**
     * Get current state
     */
    getState() {
        return {
            isListening: this.isListening,
            isSpeaking: this.isSpeaking,
            volume: this.smoothedVolume
        };
    }
}

// =============================================
// FACTORY FUNCTION
// =============================================
export function createVoiceHandler(callbacks, options) {
    return new VoiceHandler(callbacks, options);
}

export default VoiceHandler;
