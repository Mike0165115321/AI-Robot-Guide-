class VoiceHandler {
    /**
     * @param {object} callbacks - ฟังก์ชัน Callback ต่างๆ
     * @param {function} callbacks.onStatusUpdate - อัปเดตสถานะ (เช่น "กำลังฟัง...", "รับฟังอยู่...")
     * @param {function} callbacks.onSpeechEnd - ส่ง Blob เสียงเมื่อพูดจบ
     * @param {object} options - การตั้งค่า VAD สำหรับปรับจูน
     */
    constructor(callbacks, options = {}) {
        this.callbacks = { onStatusUpdate: () => {}, onSpeechEnd: () => {}, ...callbacks };

        const defaults = {
            NOISE_FLOOR: 0.02,
            SPEECH_THRESHOLD: 0.05,
            AMPLIFICATION: 50,
            SILENCE_DELAY_MS: 800,
            SPEECH_CONFIRMATION_FRAMES: 4,
            MIN_BLOB_SIZE_BYTES: 8000,
            smoothingFactor: 0.4
        };
        Object.assign(this, defaults, options);

        this.smoothedVolume = 0.0;
        this.wasInterrupted = false;
        this.isListening = false;
        this.isSpeaking = false;
        this.silenceTimeout = null;
        this.audioChunks = [];
        this.speechFrameCount = 0;

        this.audioContext = null;
        this.mediaStream = null;
        this.mediaRecorder = null;
        this.analyser = null;
        this.dataArray = null;
    }

    _getAudioContext() {
        if (!this.audioContext || this.audioContext.state === 'closed') {
            this.audioContext = new (window.AudioContext || window.webkitAudioContext)();
        }
        return this.audioContext;
    }

    async start() {
        if (this.isListening) return;

        const audioContext = this._getAudioContext();
        if (audioContext.state === 'suspended') {
            await audioContext.resume();
        }

        try {
            this.mediaStream = await navigator.mediaDevices.getUserMedia({
                audio: {
                    noiseSuppression: true,
                    echoCancellation: true,
                    autoGainControl: true,
                }
            });

            const source = audioContext.createMediaStreamSource(this.mediaStream);
            this.analyser = audioContext.createAnalyser();
            this.analyser.fftSize = 256;
            source.connect(this.analyser);
            this.dataArray = new Uint8Array(this.analyser.frequencyBinCount);

            const options = { mimeType: 'audio/webm;codecs=opus', audioBitsPerSecond: 128000 };
            this.mediaRecorder = MediaRecorder.isTypeSupported(options.mimeType)
                ? new MediaRecorder(this.mediaStream, options)
                : new MediaRecorder(this.mediaStream);

            this.mediaRecorder.ondataavailable = (event) => {
                if (event.data.size > 0) this.audioChunks.push(event.data);
            };

            this.mediaRecorder.onstop = () => {
                if (this.wasInterrupted) {
                    this.audioChunks = [];
                    this.wasInterrupted = false;
                    return;
                }
                const audioBlob = new Blob(this.audioChunks, { type: 'audio/webm' });
                this.audioChunks = [];

                if (audioBlob.size > this.MIN_BLOB_SIZE_BYTES) {
                    this.callbacks.onSpeechEnd(audioBlob);
                } else {
                    console.log(`🎤 VAD: Discarding audio, too small (${audioBlob.size} bytes).`);
                }
                this.callbacks.onStatusUpdate("กำลังฟัง...");
            };

            this.isListening = true;
            this.smoothedVolume = 0.0;
            this.speechFrameCount = 0;
            this.callbacks.onStatusUpdate("กำลังฟัง...");
            this._runDetectionLoop();

        } catch (err) {
            console.error("VAD: Microphone access error:", err);
            this.callbacks.onStatusUpdate("ไม่สามารถเข้าถึงไมโครโฟน");
        }
    }

    stop(interrupted = false) {
        if (!this.isListening) return;

        this.wasInterrupted = interrupted;
        this.isListening = false;
        
        if (this.mediaRecorder && this.mediaRecorder.state === 'recording') {
            this.mediaRecorder.stop();
        }
        this.mediaStream?.getTracks().forEach(track => track.stop());
        this.mediaStream = null;
        clearTimeout(this.silenceTimeout);
        this.silenceTimeout = null;

        console.log("VAD: Stopped.");
        this.callbacks.onStatusUpdate("หยุดทำงาน");
    }

    _runDetectionLoop() {
        if (!this.isListening) return;
        requestAnimationFrame(() => this._runDetectionLoop());

        this.analyser.getByteTimeDomainData(this.dataArray);
        let sumSquares = 0.0;
        for (const amplitude of this.dataArray) {
            const normalizedAmplitude = (amplitude / 128.0) - 1.0;
            sumSquares += normalizedAmplitude * normalizedAmplitude;
        }
        let rawVolume = Math.sqrt(sumSquares / this.dataArray.length);

        if (rawVolume < this.NOISE_FLOOR) rawVolume = 0;
        
        const amplifiedVolume = rawVolume * this.AMPLIFICATION;
        this.smoothedVolume = this.smoothedVolume * this.smoothingFactor + amplifiedVolume * (1 - this.smoothingFactor);
        
        if (this.smoothedVolume > this.SPEECH_THRESHOLD) {
            this.speechFrameCount++;
            if (this.speechFrameCount >= this.SPEECH_CONFIRMATION_FRAMES) {
                if (!this.isSpeaking) {
                    this.isSpeaking = true;
                    if (this.mediaRecorder.state === 'inactive') this.mediaRecorder.start();
                    this.callbacks.onStatusUpdate("รับฟังอยู่...");
                }
                clearTimeout(this.silenceTimeout);
                this.silenceTimeout = null;
            }
        } else {
            this.speechFrameCount = 0;
            if (this.isSpeaking && this.silenceTimeout === null) {
                this.silenceTimeout = setTimeout(() => {
                    if (this.mediaRecorder.state === 'recording') {
                        this.mediaRecorder.stop();
                    }
                    this.isSpeaking = false;
                    this.silenceTimeout = null;
                }, this.SILENCE_DELAY_MS);
            }
        }
    }
}