class VoiceHandler {
    /**
     * @param {AudioContext} audioContext - [แก้ไข] รับ AudioContext ที่สร้างจากข้างนอก
     * @param {object} callbacks - ฟังก์ชัน Callback ต่างๆ
     * @param {function} callbacks.onStatusUpdate
     * @param {function} callbacks.onSpeechEnd
     * @param {object} options - การตั้งค่า VAD
     */
    constructor(audioContext, callbacks, options = {}) { // 👈 [แก้ไข] เพิ่ม audioContext
        if (!audioContext) {
            throw new Error("VoiceHandler requires a valid AudioContext to be provided.");
        }

        this.callbacks = { onStatusUpdate: () => { }, onSpeechEnd: () => { }, ...callbacks };

        const defaults = {
            NOISE_FLOOR: 0.02,
            SPEECH_THRESHOLD: 0.05,
            AMPLIFICATION: 50,
            SILENCE_DELAY_MS: 1000,          // [FIX] เพิ่มจาก 800 เป็น 1000 - รอให้ user พูดจบแน่ๆ
            SPEECH_CONFIRMATION_FRAMES: 6,    // [FIX] เพิ่มจาก 4 เป็น 6 - ลดความไวในการเริ่มรับเสียง
            MIN_BLOB_SIZE_BYTES: 8000,
            smoothingFactor: 0.4,
            MAX_RECORDING_MS: 10000 // 10 วินาที (ตามที่เราตั้งไว้)
        };
        Object.assign(this, defaults, options);

        this.smoothedVolume = 0.0;
        this.wasInterrupted = false;
        this.isListening = false;
        this.isSpeaking = false;
        this.silenceTimeout = null;
        this.recordingTimeout = null;
        this.audioChunks = [];
        this.speechFrameCount = 0;

        this.audioContext = audioContext; // 👈 [แก้ไข] ใช้ Context ที่รับเข้ามา
        this.mediaStream = null;
        this.mediaRecorder = null;
        this.analyser = null;
        this.dataArray = null;
    }

    _getAudioContext() {
        // 👈 [ลบ] ฟังก์ชันนี้ไม่จำเป็นอีกต่อไป เพราะเรารับ audioContext จาก constructor แล้ว
        // เราจะปล่อยฟังก์ชัน start() ให้ใช้ this.audioContext โดยตรง
        return this.audioContext;
    }

    async start() {
        if (this.isListening) return;

        // 👈 [แก้ไข] ใช้ this.audioContext โดยตรง
        const audioContext = this.audioContext;
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

            // [แก้ไข] การันตี MimeType
            const mimeTypes = [
                'audio/webm;codecs=opus', 'audio/ogg;codecs=opus', 'audio/webm'
            ];
            const supportedMimeType = mimeTypes.find(type => MediaRecorder.isTypeSupported(type));

            if (!supportedMimeType) {
                console.error("VAD: ไม่มี MimeType ที่รองรับ (webm/ogg) สำหรับการอัดเสียง");
                this.callbacks.onStatusUpdate("เบราว์เซอร์ไม่รองรับการอัดเสียง");
                return;
            }

            console.log("VAD: Using supported mimeType:", supportedMimeType);
            const options = { mimeType: supportedMimeType, audioBitsPerSecond: 128000 };
            this.mediaRecorder = new MediaRecorder(this.mediaStream, options);


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

            // [เพิ่ม] เริ่มจับเวลาอัดสูงสุด
            this.recordingTimeout = setTimeout(() => {
                console.warn(`VAD: Max recording time reached (${this.MAX_RECORDING_MS / 1000}s). Forcing stop.`);
                this.stop(false);
            }, this.MAX_RECORDING_MS);

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

        clearTimeout(this.recordingTimeout); // [เพิ่ม] เคลียร์ตัวจับเวลาสูงสุด
        this.recordingTimeout = null;      // [เพิ่ม]

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