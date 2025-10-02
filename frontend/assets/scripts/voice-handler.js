// /frontend/assets/scripts/voice-handler.js (The Correct, New Version)

class VoiceHandler {
    constructor(mainController, onSpeechEndCallback) {
        // รับ Main Controller เข้ามาเพื่อใช้ควบคุม UI (เช่น status text)
        this.mainController = mainController; 
        this.onSpeechEnd = onSpeechEndCallback;

        // ค่าที่ปรับจูนแล้วของคุณ
        this.NOISE_FLOOR = 0.1;
        this.SPEECH_THRESHOLD = 0.01;
        this.AMPLIFICATION = 70;
        this.SILENCE_DELAY_MS = 400;

        // ตัวแปรสถานะภายใน
        this.isListening = false;
        this.isSpeaking = false;
        this.silenceTimeout = null;
        this.audioChunks = [];
        this.audioContext = null;
        this.mediaStream = null;
        this.mediaRecorder = null;
        this.analyser = null;
        this.dataArray = null;
        this.debugInterval = null;
    }

    async start(audioContext) {
        if (this.isListening) return;
        this.audioContext = audioContext;
        try {
            this.mediaStream = await navigator.mediaDevices.getUserMedia({ audio: true });
            const source = this.audioContext.createMediaStreamSource(this.mediaStream);
            this.analyser = this.audioContext.createAnalyser();
            source.connect(this.analyser);
            this.dataArray = new Uint8Array(this.analyser.frequencyBinCount);
            this.mediaRecorder = new MediaRecorder(this.mediaStream, { mimeType: 'audio/webm' });
            
            this.mediaRecorder.ondataavailable = (event) => {
                if (event.data.size > 0) this.audioChunks.push(event.data);
            };

            this.mediaRecorder.onstop = () => {
                const audioBlob = new Blob(this.audioChunks, { type: 'audio/webm' });
                this.audioChunks = [];
                if (audioBlob.size > 2000) { this.onSpeechEnd(audioBlob); }
            };

            this.isListening = true;
            this.mainController.setStatus("กำลังฟัง...");
            this.runDetectionLoop();
        } catch (err) {
            console.error("Microphone access error:", err);
            this.mainController.setStatus("ไม่สามารถเข้าถึงไมโครโฟน");
        }
    }

    stop() {
        this.isListening = false;
        if (this.mediaRecorder && this.mediaRecorder.state === 'recording') {
            this.mediaRecorder.stop();
        }
        this.mediaStream?.getTracks().forEach(track => track.stop());
        this.mediaStream = null;
        clearTimeout(this.silenceTimeout);
        if (this.debugInterval) {
            clearInterval(this.debugInterval);
            this.debugInterval = null;
        }
    }

    runDetectionLoop() {
        if (!this.isListening) return;
        requestAnimationFrame(() => this.runDetectionLoop());

        this.analyser.getByteFrequencyData(this.dataArray);
        let rawVolume = this.dataArray.reduce((a, b) => a + b) / this.dataArray.length / 128.0;
        if (rawVolume < this.NOISE_FLOOR) {
            rawVolume = 0;
        }
        const amplifiedVolume = rawVolume * this.AMPLIFICATION;

        if (!this.debugInterval) {
            this.debugInterval = setInterval(() => {
                console.log(`Raw Vol: ${rawVolume.toFixed(4)} | Amplified Vol: ${amplifiedVolume.toFixed(4)} | Threshold: ${this.SPEECH_THRESHOLD}`);
            }, 200);
        }

        if (amplifiedVolume > this.SPEECH_THRESHOLD) {
            if (!this.isSpeaking) {
                this.isSpeaking = true;
                if (this.mediaRecorder.state === 'inactive') this.mediaRecorder.start();
                this.mainController.setStatus("รับฟังอยู่...");
            }
            clearTimeout(this.silenceTimeout);
            this.silenceTimeout = null;
        } else {
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