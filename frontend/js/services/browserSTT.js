/**
 * # Browser STT (Speech-to-Text) Service
 * 
 * ใช้ Web Speech API สำหรับแปลงเสียงเป็นข้อความ
 * รองรับ Chrome, Edge และ Safari
 * 
 * @example
 * browserSTT.start({
 *     onResult: (text) => console.log(text),
 *     onInterim: (text) => updateUI(text)
 * });
 */

class BrowserSTTService {
    constructor() {
        this.recognition = null;
        this.isListening = false;
        this.finalTranscript = '';
        this.isSupported = this._checkSupport();

        // Callbacks
        this.callbacks = {
            onStart: () => { },
            onEnd: () => { },
            onResult: () => { },
            onInterim: () => { },
            onError: () => { }
        };
    }

    /**
     * Check browser support
     * @private
     */
    _checkSupport() {
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        return !!SpeechRecognition;
    }

    /**
     * Initialize recognition
     * @private
     */
    _initRecognition() {
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        if (!SpeechRecognition) return null;

        const recognition = new SpeechRecognition();

        // Config for Thai language
        recognition.lang = 'th-TH';
        recognition.interimResults = true;
        recognition.continuous = true;
        recognition.maxAlternatives = 1;

        // Event handlers
        recognition.onstart = () => {
            this.isListening = true;
            this.finalTranscript = '';
            console.log('🎤 Browser STT: Started');
            this.callbacks.onStart();
        };

        recognition.onend = () => {
            this.isListening = false;
            console.log('🎤 Browser STT: Ended');
            this.callbacks.onEnd();

            // Send final result
            if (this.finalTranscript.trim()) {
                this.callbacks.onResult(this.finalTranscript.trim());
            }
        };

        recognition.onerror = (event) => {
            console.error('🎤 Browser STT: Error', event.error);

            // Don't report "no-speech" as error
            if (event.error !== 'no-speech') {
                this.callbacks.onError(this._getErrorMessage(event.error));
            }
        };

        recognition.onresult = (event) => {
            let interimTranscript = '';

            for (let i = event.resultIndex; i < event.results.length; i++) {
                if (event.results[i].isFinal) {
                    this.finalTranscript += event.results[i][0].transcript;
                } else {
                    interimTranscript += event.results[i][0].transcript;
                }
            }

            // Send interim update
            const fullText = this.finalTranscript + interimTranscript;
            if (fullText) {
                this.callbacks.onInterim(fullText);
            }
        };

        return recognition;
    }

    /**
     * Start listening
     */
    start(callbacks = {}) {
        if (!this.isSupported) {
            console.error('🎤 Browser STT: Not supported');
            callbacks.onError?.('เบราว์เซอร์นี้ไม่รองรับ Speech Recognition');
            return false;
        }

        if (this.isListening) {
            console.warn('🎤 Browser STT: Already listening');
            return true;
        }

        // Set callbacks
        this.callbacks = { ...this.callbacks, ...callbacks };

        // Initialize and start
        this.recognition = this._initRecognition();
        if (!this.recognition) return false;

        try {
            this.recognition.start();
            return true;
        } catch (e) {
            console.error('🎤 Browser STT: Start error', e);
            return false;
        }
    }

    /**
     * Stop listening
     */
    stop() {
        if (!this.recognition || !this.isListening) return;

        try {
            this.recognition.stop();
        } catch (e) {
            console.error('🎤 Browser STT: Stop error', e);
        }
    }

    /**
     * Abort listening (discard results)
     */
    abort() {
        if (!this.recognition) return;

        this.finalTranscript = ''; // Clear results
        try {
            this.recognition.abort();
        } catch (e) {
            console.error('🎤 Browser STT: Abort error', e);
        }
    }

    /**
     * Get error message in Thai
     * @private
     */
    _getErrorMessage(error) {
        const messages = {
            'not-allowed': 'ไม่อนุญาตให้ใช้ไมโครโฟน',
            'network': 'เกิดปัญหาเครือข่าย',
            'aborted': 'ถูกยกเลิก',
            'audio-capture': 'ไม่พบไมโครโฟน',
            'service-not-allowed': 'ไม่สามารถใช้งานได้'
        };
        return messages[error] || `เกิดข้อผิดพลาด: ${error}`;
    }

    /**
     * Get current state
     */
    getState() {
        return {
            isListening: this.isListening,
            isSupported: this.isSupported,
            currentTranscript: this.finalTranscript
        };
    }
}

// Singleton
export const browserSTT = new BrowserSTTService();
export default browserSTT;
