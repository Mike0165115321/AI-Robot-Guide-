// /frontend/assets/scripts/voice-handler.js
// 🎙️ Voice Activity Detection (VAD) Handler (V2.0 - Cleaned)

class VoiceHandler {
    /**
     * @param {AudioContext} audioContext - Shared AudioContext from main controller
     * @param {object} callbacks - Callback functions
     * @param {function} callbacks.onStatusUpdate
     * @param {function} callbacks.onSpeechEnd
     * @param {object} options - VAD configuration
     */
    constructor(audioContext, callbacks, options = {}) {
        if (!audioContext) {
            throw new Error('VoiceHandler requires a valid AudioContext');
        }

        this.callbacks = {
            onStatusUpdate: () => { },
            onSpeechEnd: () => { },
            ...callbacks
        };

        // VAD Configuration
        const defaults = {
            NOISE_FLOOR: 0.003,
            SPEECH_THRESHOLD: 0.40, // Increased from 0.35
            AMPLIFICATION: 45,      // Reduced from 60 to filter noise
            SILENCE_DELAY_MS: 1500,
            SPEECH_CONFIRMATION_FRAMES: 3,
            MIN_BLOB_SIZE_BYTES: 1000, // Reduced from 5000 to allow short words
            smoothingFactor: 0.2,
            MAX_RECORDING_MS: 15000
        };
        Object.assign(this, defaults, options);

        // State
        this.smoothedVolume = 0.0;
        this.wasInterrupted = false;
        this.isListening = false;
        this.isSpeaking = false;
        this.silenceTimeout = null;
        this.recordingTimeout = null;
        this.audioChunks = [];
        this.speechFrameCount = 0;

        // Audio
        this.audioContext = audioContext;
        this.mediaStream = null;
        this.mediaRecorder = null;
        this.analyser = null;
        this.dataArray = null;

        // Debug
        this.lastLogTime = 0;
    }

    async start() {
        if (this.isListening) return;

        const audioContext = this.audioContext;
        if (audioContext.state === 'suspended') {
            await audioContext.resume();
        }

        try {
            this.mediaStream = await navigator.mediaDevices.getUserMedia({
                audio: {
                    noiseSuppression: true,
                    echoCancellation: true,
                    autoGainControl: true
                }
            });

            const source = audioContext.createMediaStreamSource(this.mediaStream);
            this.analyser = audioContext.createAnalyser();
            this.analyser.fftSize = 256;
            source.connect(this.analyser);
            this.dataArray = new Uint8Array(this.analyser.frequencyBinCount);

            // Setup MediaRecorder with compatible mimeType
            const mimeTypes = ['audio/webm;codecs=opus', 'audio/ogg;codecs=opus', 'audio/webm'];
            const supportedMimeType = mimeTypes.find(type => MediaRecorder.isTypeSupported(type));

            if (!supportedMimeType) {
                this.callbacks.onStatusUpdate('เบราว์เซอร์ไม่รองรับการอัดเสียง');
                return;
            }

            this.mediaRecorder = new MediaRecorder(this.mediaStream, {
                mimeType: supportedMimeType,
                audioBitsPerSecond: 128000
            });

            this.mediaRecorder.ondataavailable = (event) => {
                if (event.data.size > 0) {
                    this.audioChunks.push(event.data);
                }
            };

            this.mediaRecorder.onstop = () => {
                console.log("🛑 MediaRecorder Stopped");
                if (this.wasInterrupted) {
                    console.warn("⚠️ Recording was interrupted (discarding)");
                    this.audioChunks = [];
                    this.wasInterrupted = false;
                    return;
                }

                const audioBlob = new Blob(this.audioChunks, { type: 'audio/webm' });
                console.log(`📦 Audio Blob Created. Size: ${audioBlob.size} bytes`);
                this.audioChunks = [];

                if (audioBlob.size > this.MIN_BLOB_SIZE_BYTES) {
                    console.log("✅ Sending valid audio blob...");
                    this.callbacks.onSpeechEnd(audioBlob);
                } else {
                    console.warn(`⚠️ Audio too short/small (${audioBlob.size} < ${this.MIN_BLOB_SIZE_BYTES}). Discarding.`);
                    this.callbacks.onStatusUpdate('เสียงสั้นไป (ลองใหม่)');
                }
                // this.callbacks.onStatusUpdate('กำลังฟัง...'); // Don't reset generic status here, let logic handle it
            };

            this.isListening = true;
            this.smoothedVolume = 0.0;
            this.speechFrameCount = 0;
            this.callbacks.onStatusUpdate('กำลังฟัง...');
            this._runDetectionLoop();

            // Max recording timeout
            this.recordingTimeout = setTimeout(() => {
                this.stop(false);
            }, this.MAX_RECORDING_MS);

        } catch (err) {
            console.error('Microphone access error:', err);
            this.callbacks.onStatusUpdate('ไม่สามารถเข้าถึงไมโครโฟน');
        }
    }

    stop(interrupted = false) {
        if (!this.isListening) return;

        this.wasInterrupted = interrupted;
        this.isListening = false;

        if (this.mediaRecorder?.state === 'recording') {
            this.mediaRecorder.stop();
        }

        this.mediaStream?.getTracks().forEach(track => track.stop());
        this.mediaStream = null;

        clearTimeout(this.silenceTimeout);
        this.silenceTimeout = null;

        clearTimeout(this.recordingTimeout);
        this.recordingTimeout = null;

        this.callbacks.onStatusUpdate('หยุดทำงาน');
    }

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
        if (rawVolume < this.NOISE_FLOOR) rawVolume = 0;

        // Amplify and smooth
        const amplifiedVolume = rawVolume * this.AMPLIFICATION;
        this.smoothedVolume = this.smoothedVolume * this.smoothingFactor + amplifiedVolume * (1 - this.smoothingFactor);

        // Speech detection
        if (this.smoothedVolume > this.SPEECH_THRESHOLD) {
            this.speechFrameCount++;
            if (this.speechFrameCount >= this.SPEECH_CONFIRMATION_FRAMES) {
                if (!this.isSpeaking) {
                    this.isSpeaking = true;
                    console.log("🗣️ Speech STARTED (Volume > Threshold)");
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
                console.log("🤫 Silence Detected, starting timeout...");
                this.silenceTimeout = setTimeout(() => {
                    if (this.mediaRecorder?.state === 'recording') {
                        this.mediaRecorder.stop();
                    }
                    this.isSpeaking = false;
                    this.silenceTimeout = null;
                    console.log("⏹️ Speech ENDED (Silence Timeout)");
                }, this.SILENCE_DELAY_MS);
            }
        }

        // Throttle Log: Print volume every 200ms
        const now = Date.now();
        if (now - this.lastLogTime > 200) {
            const vol = this.smoothedVolume.toFixed(2);
            // Visual Bar
            const barLength = Math.min(20, Math.floor(this.smoothedVolume * 10));
            const bar = '█'.repeat(barLength).padEnd(20, '░');
            console.log(`🎤 Level: [${bar}] ${vol} ${this.isSpeaking ? '(Speaking)' : ''}`);
            this.lastLogTime = now;
        }
    }
}