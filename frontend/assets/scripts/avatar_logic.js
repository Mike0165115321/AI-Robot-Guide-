document.addEventListener('DOMContentLoaded', () => {

    // --- UI Elements & Controller ---
    const faceV1 = document.getElementById('avatar-face');
    const statusText = document.getElementById('status-text');

    const uiController = {
        setEmotion(emotion) {
            const activeAvatar = window.currentAvatar || 'v1'; 
            if (activeAvatar === 'v1') {
                faceV1.className = 'face';
                if (emotion) faceV1.classList.add(emotion);
            } else if (activeAvatar === 'v2') {
                switch (emotion) {
                    case 'talking':
                        if (typeof playNeutral === 'function') playNeutral();
                        break;
                    case 'thinking':
                        if (typeof playThinking === 'function') playThinking();
                        break;
                    default:
                        if (typeof playNeutral === 'function') playNeutral();
                        break;
                }
            }
        },
        setStatus(text) {
            if (statusText) {
                statusText.textContent = text;
            }
        }
    };

    // --- AudioProcessor Class ---
    class AudioProcessor {
        constructor(onSpeechEndCallback) {
            this.onSpeechEnd = onSpeechEndCallback;
            this.NOISE_FLOOR = 0.1;
            this.SPEECH_THRESHOLD = 0.05;
            this.AMPLIFICATION = 50;
            this.SILENCE_DELAY_MS = 400;
            this.smoothingFactor = 0.3;
            this.smoothedVolume = 0.0;
            this.wasInterrupted = false; // Flag สำหรับ Barge-in Fix
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
                    // [Barge-in Fix] ตรวจสอบก่อนว่าถูกขัดจังหวะหรือไม่
                    if (this.wasInterrupted) {
                        console.log("Recording interrupted by AI response. Discarding audio chunk.");
                        this.audioChunks = []; // ล้างเสียงที่บันทึกไว้ทิ้ง
                        this.wasInterrupted = false; // รีเซ็ตสถานะ
                        return; // ไม่ต้องทำอะไรต่อ
                    }

                    const audioBlob = new Blob(this.audioChunks, { type: 'audio/webm' });
                    this.audioChunks = [];
                    if (audioBlob.size > 4000) { 
                        this.onSpeechEnd(audioBlob); 
                    }
                };
                this.isListening = true;
                this.smoothedVolume = 0.0;
                uiController.setStatus("กำลังฟัง...");
                this.runDetectionLoop();
            } catch (err) {
                console.error("Microphone access error:", err);
                uiController.setStatus("ไม่สามารถเข้าถึงไมโครโฟน");
            }
        }

        // [Barge-in Fix] ปรับปรุง stop ให้รับ parameter ได้
        stop(interrupted = false) {
            if (interrupted) {
                this.wasInterrupted = true;
            }
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
            if (!this.isListening || !this.analyser || !this.dataArray) { return; }
            requestAnimationFrame(() => this.runDetectionLoop());
            
            this.analyser.getByteFrequencyData(this.dataArray);
            let rawVolume = this.dataArray.reduce((a, b) => a + b) / this.dataArray.length / 128.0;
            if (rawVolume < this.NOISE_FLOOR) { rawVolume = 0; }
            const amplifiedVolume = rawVolume * this.AMPLIFICATION;
            this.smoothedVolume = this.smoothedVolume * this.smoothingFactor + amplifiedVolume * (1 - this.smoothingFactor);

            if (!this.debugInterval) {
                this.debugInterval = setInterval(() => { 
                    console.log(`Amp Vol: ${amplifiedVolume.toFixed(4)} | Smooth Vol: ${this.smoothedVolume.toFixed(4)} | Threshold: ${this.SPEECH_THRESHOLD}`); 
                }, 200);
            }

            if (this.smoothedVolume > this.SPEECH_THRESHOLD) {
                if (!this.isSpeaking) {
                    this.isSpeaking = true;
                    if (this.mediaRecorder.state === 'inactive') { this.mediaRecorder.start(); }
                    uiController.setStatus("รับฟังอยู่...");
                }
                clearTimeout(this.silenceTimeout);
                this.silenceTimeout = null;
            } else {
                if (this.isSpeaking && this.silenceTimeout === null) {
                    this.silenceTimeout = setTimeout(() => {
                        if (this.mediaRecorder.state === 'recording') { this.mediaRecorder.stop(); }
                        this.isSpeaking = false; this.silenceTimeout = null;
                    }, this.SILENCE_DELAY_MS);
                }
            }
        }
    }

    let websocket = null;
    let isPlayingAudio = false;
    let mainAudioContext = null;
    let currentAudioSource = null;
    let idleTimeout = null;
    const IDLE_TIME_MS = 60000;

    function startIdleTimer() {
        clearTimeout(idleTimeout);
        idleTimeout = setTimeout(() => {
            console.log("Idle timer triggered. Asking backend for a prompt.");
            audioProcessor.stop(); 
            if (websocket?.readyState === WebSocket.OPEN) {
                websocket.send(JSON.stringify({ action: "idle_prompt" }));
                uiController.setEmotion('thinking');
            }
        }, IDLE_TIME_MS);
        console.log(`Idle timer (re)started for ${IDLE_TIME_MS / 1000} seconds.`);
    }

    const audioProcessor = new AudioProcessor((audioBlob) => {
        if (websocket?.readyState === WebSocket.OPEN) {
            clearTimeout(idleTimeout); 
            console.log("User spoke, idle timer paused.");
            console.log("Sending audio blob to backend...");
            uiController.setEmotion('thinking');
            websocket.send(audioBlob);
        }
    });

    async function startAudioSystem() {
        if (mainAudioContext) return;
        try {
            mainAudioContext = new (window.AudioContext || window.webkitAudioContext)();
            await mainAudioContext.resume();
            console.log("AudioContext is active and running!");
            audioProcessor.start(mainAudioContext);
        } catch (e) {
            console.error("Failed to initialize audio system.", e);
            uiController.setStatus("เกิดข้อผิดพลาดในการเริ่มระบบเสียง");
        }
    }

    function connectWebSocket() {
        if (typeof API_HOST === 'undefined' || typeof API_PORT === 'undefined') {
            console.error("API_HOST or API_PORT is not defined in config.js");
            uiController.setStatus("ตั้งค่า API ไม่ถูกต้อง");
            return;
        }
        const wsUrl = `ws://${API_HOST}:${API_PORT}/api/chat/ws/avatar_chat`;
        websocket = new WebSocket(wsUrl);
        websocket.binaryType = 'arraybuffer';

        websocket.onopen = () => {
            console.log("WebSocket connected.");
            uiController.setStatus("โปรดคลิกเพื่อเริ่มการสนทนา");
            document.body.addEventListener('click', startAudioSystem, { once: true });
            startIdleTimer();
        };

        websocket.onmessage = async (event) => {
            clearTimeout(idleTimeout);
            
            if (typeof event.data === 'string') {
                try {
                    const data = JSON.parse(event.data);
                    if (data.emotion) {
                        uiController.setEmotion(data.emotion);
                        if (data.emotion === 'thinking') {
                            uiController.setStatus("กำลังประมวลผล...");
                        }
                    }
                } catch (e) { console.error("Failed to parse JSON message:", e); }
            }
            else if (event.data instanceof ArrayBuffer) {
                if (!mainAudioContext) {
                    await startAudioSystem(); 
                    if (!mainAudioContext) return;
                }
                
                // [Barge-in Fix] เรียก stop พร้อมบอกว่า "นี่คือการขัดจังหวะ"
                audioProcessor.stop(true);

                try {
                    if (currentAudioSource) {
                        currentAudioSource.stop();
                        currentAudioSource.disconnect();
                    }
                    const audioData = event.data;
                    const audioBuffer = await mainAudioContext.decodeAudioData(audioData);
                    const newSource = mainAudioContext.createBufferSource();
                    newSource.buffer = audioBuffer;
                    newSource.connect(mainAudioContext.destination);
                    newSource.start(0);
                    currentAudioSource = newSource;
                    isPlayingAudio = true;
                    uiController.setEmotion('talking');
                    uiController.setStatus(" ");

                    newSource.onended = () => {
                        if (currentAudioSource === newSource) {
                            isPlayingAudio = false;
                            currentAudioSource = null;
                            uiController.setEmotion(null);
                            uiController.setStatus("กำลังฟัง...");
                            audioProcessor.start(mainAudioContext);
                            startIdleTimer();
                        }
                    };
                } catch (e) {
                    console.error("Error playing received audio:", e);
                    isPlayingAudio = false;
                    currentAudioSource = null;
                    uiController.setEmotion(null);
                    uiController.setStatus("กำลังฟัง...");
                    audioProcessor.start(mainAudioContext);
                    startIdleTimer();
                }
            }
        };

        websocket.onclose = () => {
            clearTimeout(idleTimeout);
            uiController.setStatus("การเชื่อมต่อถูกตัด");
            audioProcessor.stop();
            if(currentAudioSource) currentAudioSource.stop();
            console.warn("WebSocket disconnected. Attempting to reconnect in 5 seconds...");
            setTimeout(connectWebSocket, 5000);
        };
        websocket.onerror = (err) => {
            console.error("WebSocket Error:", err);
            uiController.setStatus("เกิดข้อผิดพลาดในการเชื่อมต่อ");
            websocket.close();
        };
    }

    connectWebSocket();
});