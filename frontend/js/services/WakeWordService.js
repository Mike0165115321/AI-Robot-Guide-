/**
 * # Wake Word Service
 * 
 * ฟังอยู่เบื้องหลังตลอดเวลา ตรวจจับ Wake Words เช่น "สวัสดีน้องน่าน"
 * เมื่อตรวจพบจะ emit event เพื่อให้ app.js รับและดำเนินการต่อ
 * 
 * @example
 * wakeWordService.start();
 * wakeWordService.on('detected', () => activateAssistant());
 */

class WakeWordService {
    constructor() {
        // Wake Words ที่รองรับ (lowercase)
        this.wakeWords = [
            'สวัสดีน้องน่าน',
            'น้องน่าน',
            'หนูน่าน',
            'hi nan',
            'hey nan',
            'hello nan'
        ];

        this.recognition = null;
        this.isListening = false;
        this.isPaused = false;  // หยุดชั่วคราวระหว่าง TTS พูด
        this.callbacks = {
            onDetected: null,
            onStatusChange: null,
            onError: null
        };

        // Check browser support
        this.isSupported = this._checkSupport();

        if (!this.isSupported) {
            console.warn('⚠️ WakeWordService: Web Speech API not supported in this browser');
        }
    }

    /**
     * ตรวจสอบว่า browser รองรับหรือไม่
     */
    _checkSupport() {
        return 'webkitSpeechRecognition' in window || 'SpeechRecognition' in window;
    }

    /**
     * สร้าง SpeechRecognition instance
     */
    _createRecognition() {
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        const recognition = new SpeechRecognition();

        recognition.continuous = true;       // ฟังตลอด
        recognition.interimResults = true;   // ได้ผลลัพธ์ระหว่างพูด
        recognition.lang = 'th-TH';          // ภาษาไทย
        recognition.maxAlternatives = 3;     // รับหลาย alternatives

        // Event: ได้ผลลัพธ์
        recognition.onresult = (event) => {
            if (this.isPaused) return; // ข้ามถ้า paused

            const results = event.results;
            const lastResult = results[results.length - 1];

            // รวบรวม transcript จากทุก alternatives
            for (let i = 0; i < lastResult.length; i++) {
                const transcript = lastResult[i].transcript.toLowerCase().trim();
                console.log(`🎤 [WakeWord] Heard: "${transcript}"`);

                // ตรวจสอบ wake word
                if (this._matchWakeWord(transcript)) {
                    console.log('✨ [WakeWord] DETECTED! Activating...');
                    this._onWakeWordDetected(transcript);
                    return;
                }
            }
        };

        // Event: Error
        recognition.onerror = (event) => {
            console.error('❌ [WakeWord] Error:', event.error);

            if (event.error === 'not-allowed') {
                this.callbacks.onError?.('microphone_denied');
                this.stop();
            } else if (event.error === 'network') {
                // Network error - retry
                setTimeout(() => this._restart(), 1000);
            }
        };

        // Event: หยุดทำงาน (อาจหยุดเองหลังเงียบนาน)
        recognition.onend = () => {
            console.log('🔇 [WakeWord] Recognition ended');

            // Auto-restart ถ้ายังอยู่ในโหมดฟัง
            if (this.isListening && !this.isPaused) {
                console.log('🔄 [WakeWord] Auto-restarting...');
                setTimeout(() => this._restart(), 100);
            }
        };

        recognition.onstart = () => {
            console.log('👂 [WakeWord] Listening for wake word...');
            this.callbacks.onStatusChange?.('listening');
        };

        return recognition;
    }

    /**
     * ตรวจสอบว่า transcript match wake word ไหม
     */
    _matchWakeWord(transcript) {
        return this.wakeWords.some(word => transcript.includes(word.toLowerCase()));
    }

    /**
     * เมื่อตรวจพบ wake word
     */
    _onWakeWordDetected(transcript) {
        this.pause(); // หยุดฟังชั่วคราว
        this.callbacks.onDetected?.(transcript);
    }

    /**
     * Restart recognition
     */
    _restart() {
        if (!this.isListening || this.isPaused) return;

        try {
            this.recognition?.start();
        } catch (e) {
            // Already running - ignore
        }
    }

    // ==========================================
    // PUBLIC API
    // ==========================================

    /**
     * เริ่มฟัง wake word
     */
    start() {
        if (!this.isSupported) {
            console.warn('⚠️ [WakeWord] Not supported');
            return false;
        }

        if (this.isListening) {
            console.log('ℹ️ [WakeWord] Already listening');
            return true;
        }

        this.recognition = this._createRecognition();
        this.isListening = true;
        this.isPaused = false;

        try {
            this.recognition.start();
            console.log('🟢 [WakeWord] Started');
            return true;
        } catch (e) {
            console.error('❌ [WakeWord] Start failed:', e);
            return false;
        }
    }

    /**
     * หยุดฟังถาวร
     */
    stop() {
        if (!this.recognition) return;

        this.isListening = false;
        this.isPaused = false;

        try {
            this.recognition.stop();
        } catch (e) {
            // Ignore
        }

        this.recognition = null;
        console.log('🔴 [WakeWord] Stopped');
        this.callbacks.onStatusChange?.('stopped');
    }

    /**
     * หยุดชั่วคราว (ระหว่าง TTS พูด)
     */
    pause() {
        if (!this.isListening) return;

        this.isPaused = true;

        try {
            this.recognition?.stop();
        } catch (e) {
            // Ignore
        }

        console.log('⏸️ [WakeWord] Paused');
        this.callbacks.onStatusChange?.('paused');
    }

    /**
     * กลับมาฟังต่อ (หลัง TTS พูดจบ)
     */
    resume() {
        if (!this.isListening) return;

        this.isPaused = false;
        this._restart();

        console.log('▶️ [WakeWord] Resumed');
    }

    /**
     * ลงทะเบียน callback
     */
    on(event, callback) {
        if (event === 'detected') {
            this.callbacks.onDetected = callback;
        } else if (event === 'status') {
            this.callbacks.onStatusChange = callback;
        } else if (event === 'error') {
            this.callbacks.onError = callback;
        }
    }

    /**
     * ตรวจสอบสถานะ
     */
    getStatus() {
        if (!this.isSupported) return 'unsupported';
        if (!this.isListening) return 'stopped';
        if (this.isPaused) return 'paused';
        return 'listening';
    }
}

// Export singleton
export const wakeWordService = new WakeWordService();
export default wakeWordService;
