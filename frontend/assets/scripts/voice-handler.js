class VoiceHandler {
    constructor(callbacks) {
        this.callbacks = { onStatusUpdate: () => {}, onSpeechEnd: () => {}, ...callbacks };
        this.NOISE_FLOOR = 0.05;           
        this.SPEECH_THRESHOLD = 0.2;            
        this.AMPLIFICATION = 25;            
        this.SILENCE_DELAY_MS = 1000;          
        this.SPEECH_CONFIRMATION_FRAMES = 5;    
        this.speechFrameCount = 0;
        this.MIN_BLOB_SIZE_BYTES = 10000;   
        this.smoothingFactor = 0.3;
        this.smoothedVolume = 0.0;
        this.wasInterrupted = false;
        this.isListening = false;
        this.isSpeaking = false;
        this.silenceTimeout = null;
        this.audioChunks = [];
        this.audioContext = null;
        this.mediaStream = null;
        this.mediaRecorder = null;
        this.analyser = null;
        this.dataArray = null;
    }

    async start(audioContext) {
        if (this.isListening) return;
        this.audioContext = audioContext;
        try {
            this.mediaStream = await navigator.mediaDevices.getUserMedia({
                audio: {
                    noiseSuppression: true,
                    echoCancellation: true,
                    autoGainControl: true
                }
            });

            const source = this.audioContext.createMediaStreamSource(this.mediaStream);
            this.analyser = this.audioContext.createAnalyser();
            source.connect(this.analyser);
            this.dataArray = new Uint8Array(this.analyser.frequencyBinCount);
            
            const options = { mimeType: 'audio/webm;codecs=opus', audioBitsPerSecond: 128000 };
            this.mediaRecorder = MediaRecorder.isTypeSupported(options.mimeType) 
                ? new MediaRecorder(this.mediaStream, options) 
                : new MediaRecorder(this.mediaStream);
            
            console.log(`VAD: MediaRecorder initialized with mimeType: ${this.mediaRecorder.mimeType}`);

            this.mediaRecorder.ondataavailable = (event) => {
                if (event.data.size > 0) this.audioChunks.push(event.data);
            };

            this.mediaRecorder.onstop = () => {
                if (this.wasInterrupted) {
                    console.log("VAD: Recording interrupted. Discarding audio chunk.");
                    this.audioChunks = [];
                    this.wasInterrupted = false;
                    return;
                }
                const audioBlob = new Blob(this.audioChunks, { type: 'audio/webm' });
                this.audioChunks = [];
                
                if (audioBlob.size > this.MIN_BLOB_SIZE_BYTES) {
                    this.callbacks.onSpeechEnd(audioBlob);
                } else {
                    console.log(`🎤 VAD: Discarding audio blob, too small (${audioBlob.size} bytes). Not speech.`);
                }
            };

            this.isListening = true;
            this.smoothedVolume = 0.0;
            this.speechFrameCount = 0;
            this.callbacks.onStatusUpdate("กำลังฟัง...");
            this.runDetectionLoop();
        } catch (err) {
            console.error("VAD: Microphone access error:", err);
            this.callbacks.onStatusUpdate("ไม่สามารถเข้าถึงไมโครโฟน");
        }
    }

    stop(interrupted = false) {
        if (interrupted) {
            this.wasInterrupted = true;
        }
        if (!this.isListening) return;
        this.isListening = false;

        if (this.mediaRecorder && this.mediaRecorder.state === 'recording') {
            this.mediaRecorder.stop();
        }
        this.mediaStream?.getTracks().forEach(track => track.stop());
        this.mediaStream = null;
        clearTimeout(this.silenceTimeout);
    }

    runDetectionLoop() {
        if (!this.isListening) return;
        requestAnimationFrame(() => this.runDetectionLoop());

        this.analyser.getByteFrequencyData(this.dataArray);
        let rawVolume = this.dataArray.reduce((a, b) => a + b) / this.dataArray.length / 128.0;
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
                    if (this.mediaRecorder.state === 'recording') this.mediaRecorder.stop();
                    this.isSpeaking = false;
                    this.silenceTimeout = null;
                }, this.SILENCE_DELAY_MS);
            }
        }
    }
}

