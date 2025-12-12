// /frontend/assets/scripts/wake-word-handler.js
// 🎤 Wake Word Voice Mode - รอคำเรียก "น้องน่าน" แล้วเข้าโหมดฟังต่อเนื่อง

class WakeWordHandler {
    /**
     * @param {object} callbacks - ฟังก์ชัน Callback ต่างๆ
     * @param {function} callbacks.onStatusUpdate - อัปเดตสถานะ UI
     * @param {function} callbacks.onWakeWordDetected - เมื่อตรวจพบ Wake Word
     * @param {function} callbacks.onModeChange - เมื่อเปลี่ยนโหมด (wake/continuous)
     */
    constructor(callbacks) {
        this.callbacks = {
            onStatusUpdate: () => { },
            onWakeWordDetected: () => { },
            onModeChange: () => { },
            ...callbacks
        };

        this.recognition = null;
        this.isListening = false;
        this.isActive = false; // ระบบเปิดอยู่หรือไม่
        this.mode = 'wake'; // 'wake' = รอคำเรียก, 'continuous' = ฟังต่อเนื่อง
        this.silenceTimer = null;
        this.wakeRestartTimer = null;

        // ⏱️ ค่า Timeout
        this.WAKE_LISTEN_DURATION = 5000; // ฟัง 5 วิ ในโหมดรอคำเรียก
        this.SILENCE_TIMEOUT = 30000; // [FIX] เพิ่มจาก 10 เป็น 30 วิ กลับไปโหมดรอ

        // 🗣️ Wake Words (รองรับหลายรูปแบบ)
        this.wakeWords = [
            'น้องน่าน', 'นองน่าน', 'นองนาน', 'น่าน', 'นาน',
            'nongnan', 'nong nan', 'nan', 'น้อง น่าน',
            'หน่อยน่าน', 'หนองน่าน', 'น้องนาน', 'น่าน เจ้า',
            'น้องหน่าน', 'นานน่าน', 'น่านน่าน', 'น้องน่า', 'น้องนา', 
            'นา', 'น้อง', 'นาค', 'น้องนาค'
        ];

        this.initRecognition();
    }

    initRecognition() {
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        if (!SpeechRecognition) {
            console.warn('[WakeWord] Browser ไม่รองรับ Speech Recognition');
            return;
        }

        this.recognition = new SpeechRecognition();
        this.recognition.lang = 'th-TH';
        this.recognition.interimResults = true;
        this.recognition.continuous = false; // จะ control เอง per-cycle

        this.recognition.onstart = () => {
            this.isListening = true;
            console.log(`[WakeWord] 🎧 เริ่มฟัง (mode: ${this.mode})`);
        };

        this.recognition.onend = () => {
            this.isListening = false;

            // รีสตาร์ทถ้ายังทำงานอยู่และอยู่ในโหมดรอ
            if (this.isActive && this.mode === 'wake') {
                this.scheduleWakeRestart();
            }
        };

        this.recognition.onerror = (event) => {
            console.log(`[WakeWord] Error: ${event.error}`);
            if (event.error === 'no-speech' || event.error === 'aborted') {
                // ปกติ - ไม่ต้องทำอะไร จะ restart เอง
            } else if (event.error === 'not-allowed') {
                console.error('[WakeWord] ไม่ได้รับอนุญาตใช้ไมค์');
                this.stop();
            }
        };

        this.recognition.onresult = (event) => {
            let transcript = '';
            for (let i = event.resultIndex; i < event.results.length; ++i) {
                transcript += event.results[i][0].transcript;
            }

            const lowerTranscript = transcript.toLowerCase().trim();
            console.log(`[WakeWord] ได้ยิน: "${lowerTranscript}"`);

            // ตรวจสอบ Wake Word
            const wakeWordFound = this.wakeWords.some(word =>
                lowerTranscript.includes(word.toLowerCase())
            );

            if (wakeWordFound) {
                console.log('[WakeWord] ✅ ตรวจพบคำเรียก!');
                this.onWakeWordDetected();
            } else {
                this.callbacks.onStatusUpdate(`🎤 รอคำเรียก... "${transcript}"`);
            }
        };
    }

    // 🔥 เมื่อตรวจพบ Wake Word
    onWakeWordDetected() {
        // หยุดการฟังปัจจุบัน
        this.stopListening();
        this.clearAllTimers();

        this.mode = 'continuous';
        this.callbacks.onModeChange('continuous');
        this.callbacks.onWakeWordDetected();

        // เริ่ม silence timer
        this.startSilenceTimer();
    }

    // 🚀 เริ่มระบบ Wake Word
    start() {
        if (this.isActive) return;

        console.log('[WakeWord] 🎤 เริ่มระบบ Wake Word Mode');
        this.isActive = true;
        this.mode = 'wake';
        this.callbacks.onModeChange('wake');
        this.callbacks.onStatusUpdate('🎤 พูดว่า "น้องน่าน" เพื่อเริ่มสนทนา...');
        this.startListening();
    }

    // 🛑 หยุดระบบทั้งหมด
    stop() {
        console.log('[WakeWord] 🛑 หยุดระบบ');
        this.isActive = false;
        this.mode = 'wake';
        this.clearAllTimers();
        this.stopListening();
        this.callbacks.onModeChange('stopped');
    }

    // 🎧 เริ่มฟัง
    startListening() {
        if (!this.recognition || this.isListening) return;

        try {
            this.recognition.start();
        } catch (e) {
            console.error('[WakeWord] Start error:', e);
            setTimeout(() => this.startListening(), 500);
        }
    }

    // หยุดฟัง
    stopListening() {
        if (this.recognition && this.isListening) {
            try {
                this.recognition.stop();
            } catch (e) { }
        }
        this.isListening = false;
    }

    // ⏰ กำหนดเวลารีสตาร์ทในโหมดรอ
    scheduleWakeRestart() {
        if (!this.isActive || this.mode !== 'wake') return;

        this.wakeRestartTimer = setTimeout(() => {
            if (this.isActive && this.mode === 'wake') {
                console.log('[WakeWord] 🔄 รีสตาร์ทการฟังโหมดรอ...');
                this.startListening();
            }
        }, 500);
    }

    // ⏱️ Silence Timer - กลับโหมดรอถ้าเงียบ 10 วิ
    startSilenceTimer() {
        this.clearSilenceTimer();

        this.silenceTimer = setTimeout(() => {
            console.log('[WakeWord] 🔇 เงียบ 10 วิ กลับไปโหมดรอคำเรียก');
            this.returnToWakeMode();
        }, this.SILENCE_TIMEOUT);
    }

    // รีเซ็ต silence timer (เรียกเมื่อยังมีการโต้ตอบ)
    resetSilenceTimer() {
        if (this.mode === 'continuous' && this.isActive) {
            this.startSilenceTimer();
        }
    }

    clearSilenceTimer() {
        if (this.silenceTimer) {
            clearTimeout(this.silenceTimer);
            this.silenceTimer = null;
        }
    }

    // 🔙 กลับไปโหมดรอคำเรียก
    returnToWakeMode() {
        console.log('[WakeWord] 🔙 กลับโหมดรอคำเรียก');

        this.mode = 'wake';
        this.clearAllTimers();
        this.callbacks.onModeChange('wake');
        this.callbacks.onStatusUpdate('🎤 พูดว่า "น้องน่าน" เพื่อเริ่มสนทนา...');

        // เริ่มฟังใหม่ในโหมดรอ
        setTimeout(() => {
            if (this.isActive) {
                this.startListening();
            }
        }, 500);
    }

    clearAllTimers() {
        this.clearSilenceTimer();
        if (this.wakeRestartTimer) {
            clearTimeout(this.wakeRestartTimer);
            this.wakeRestartTimer = null;
        }
    }

    // สถานะปัจจุบัน
    getMode() {
        return this.mode;
    }

    isActiveMode() {
        return this.isActive;
    }

    // เช็คว่าอยู่ในโหมดฟังต่อเนื่องหรือไม่
    isContinuousMode() {
        return this.isActive && this.mode === 'continuous';
    }
}
