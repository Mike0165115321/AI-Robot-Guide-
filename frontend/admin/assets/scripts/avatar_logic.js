// /frontend/assets/scripts/avatar_logic.js
// 🤖 Avatar Controller V6.0 - Silero VAD Edition
// Clean rewrite with Neural Network VAD

document.addEventListener('DOMContentLoaded', () => {
    console.log("🤖 Avatar Logic V6.0 - Silero VAD");

    // ==========================================
    // 1. DOM References
    // ==========================================
    const UI = {
        status: document.getElementById('status-text'),
        micBtn: document.getElementById('mic-btn'),
        stopBtn: document.getElementById('stop-speech-btn'),
        startOverlay: document.getElementById('start-overlay'),
        startBtn: document.getElementById('start-btn'),
        inputForm: document.getElementById('text-query-form'),
        inputField: document.getElementById('text-query-input')
    };

    // ==========================================
    // 2. State & Components
    // ==========================================
    let fsm = null;           // AvatarStateManager
    let animator = null;      // window.avatarAnimator
    let vadInstance = null;           // Silero VAD Instance
    let ws = null;            // WebSocket
    let audioCtx = null;      // AudioContext
    let gainNode = null;      // Volume Control
    let isVADActive = false;  // VAD running state
    let audioQueue = [];      // Audio playback queue
    let isPlaying = false;    // Audio playing state

    const updateStatus = (msg) => {
        console.log(`[Status] ${msg}`);
        if (UI.status) UI.status.textContent = msg;
    };

    // ==========================================
    // 3. Start Button (User Interaction Required)
    // ==========================================
    if (UI.startBtn) {
        UI.startBtn.addEventListener('click', async () => {
            console.log("👆 User Clicked Start");
            UI.startBtn.innerHTML = '<i class="fas fa-spinner fa-spin"></i> กำลังโหลด...';

            try {
                await initSystem();
                // Hide overlay
                UI.startOverlay.style.opacity = '0';
                setTimeout(() => UI.startOverlay.remove(), 500);
            } catch (err) {
                console.error("Init Failed:", err);
                updateStatus(`❌ เกิดข้อผิดพลาด: ${err.message}`);
                UI.startBtn.innerHTML = '<i class="fas fa-redo"></i> ลองใหม่';
            }
        });
    }

    // ==========================================
    // 4. System Initialization
    // ==========================================
    async function initSystem() {
        updateStatus("กำลังโหลดระบบแสดงผล...");

        // 4.1 Wait for Animator
        let attempts = 0;
        while (!window.avatarAnimator && attempts < 50) {
            await new Promise(r => setTimeout(r, 100));
            attempts++;
        }
        animator = window.avatarAnimator;
        if (!animator) throw new Error("Animator ไม่พร้อม");
        console.log("✅ Animator Ready");

        // 4.2 Init Audio Context
        updateStatus("กำลังเริ่มระบบเสียง...");
        const AC = window.AudioContext || window.webkitAudioContext;
        audioCtx = new AC();
        gainNode = audioCtx.createGain();
        gainNode.connect(audioCtx.destination);

        if (audioCtx.state === 'suspended') {
            await audioCtx.resume();
        }
        console.log("✅ Audio Context Ready");

        // 4.3 Init FSM
        if (window.AvatarStateManager) {
            fsm = new AvatarStateManager(getFsmCallbacks());
            console.log("✅ FSM Ready");
        }

        // 4.4 Init Silero VAD
        updateStatus("กำลังโหลด AI ตรวจจับเสียง...");
        if (vadInstance !== null) {
            console.log("⚠️ VAD already exists, skipping init");
        } else {
            await initSileroVAD();
        }

        // 4.5 Connect WebSocket
        updateStatus("กำลังเชื่อมต่อเซิร์ฟเวอร์...");
        connectWebSocket();

        // 4.6 Ready!
        setTimeout(() => {
            fsm?.transitionTo('IDLE');
            if (UI.micBtn) UI.micBtn.style.display = 'flex';
            console.log("🚀 System Ready!");
        }, 300);
    }

    // ==========================================
    // 5. Silero VAD Setup
    // ==========================================
    async function initSileroVAD() {
        if (!window.vad || !window.vad.MicVAD) {
            throw new Error("Silero VAD Library not loaded");
        }

        try {
            vadInstance = await window.vad.MicVAD.new({
                // 🔧 VAD Config
                positiveSpeechThreshold: 0.8,  // ความมั่นใจที่เป็นเสียงพูด
                negativeSpeechThreshold: 0.3,  // ความมั่นใจที่เป็นเสียงเงียบ
                redemptionFrames: 5,           // จำนวน Frame ที่ต้องเงียบก่อนจบ (~0.5s at 10fps)
                frameSamples: 1536,            // 96ms per frame at 16kHz
                preSpeechPadFrames: 3,         // Buffer ก่อนพูด
                minSpeechFrames: 3,            // ต้องพูดอย่างน้อย 3 frames

                // 📢 Callbacks
                onSpeechStart: () => {
                    console.log("🎤 Speech Started");
                    if (fsm?.getState() === 'LISTENING') {
                        updateStatus("กำลังฟัง... 👂");
                        animator?.setEmotion('listening');
                    }
                },

                onSpeechEnd: async (audio) => {
                    console.log("🛑 Speech Ended, Audio received");

                    if (fsm?.getState() !== 'LISTENING') {
                        console.log("⚠️ Not in LISTENING state, ignoring");
                        return;
                    }

                    // Transition to THINKING
                    fsm.transitionTo('THINKING', { text: 'กำลังประมวลผล...' });
                    animator?.setEmotion('thinking');

                    // Convert Float32Array to WAV Blob
                    const audioBlob = float32ToWav(audio, 16000);
                    console.log(`📦 Audio Blob: ${audioBlob.size} bytes`);

                    // Send to Backend
                    if (ws?.readyState === WebSocket.OPEN) {
                        ws.send(audioBlob);
                        updateStatus("กำลังคิด... 🧠");
                    } else {
                        updateStatus("❌ ไม่ได้เชื่อมต่อเซิร์ฟเวอร์");
                        fsm.transitionTo('IDLE');
                    }
                },

                onVADMisfire: () => {
                    console.log("⚡ VAD Misfire (false positive)");
                }
            });

            console.log("✅ Silero VAD Ready");
        } catch (err) {
            console.error("VAD Init Error:", err);
            throw new Error(`VAD ไม่พร้อม: ${err.message}`);
        }
    }

    // ==========================================
    // 6. WebSocket Connection
    // ==========================================
    let reconnectCount = 0;

    function connectWebSocket() {
        if (!window.API_HOST) {
            console.error("❌ config.js not loaded");
            updateStatus("❌ ไม่พบ Config");
            return;
        }

        const url = `ws://${window.API_HOST}:${window.API_PORT}/api/avatar/ws`;
        console.log(`🔌 Connecting to ${url}...`);

        ws = new WebSocket(url);
        ws.binaryType = 'arraybuffer';

        ws.onopen = () => {
            console.log("🟢 WebSocket Connected");
            reconnectCount = 0;
        };

        ws.onmessage = async (event) => {
            // Text JSON
            if (typeof event.data === 'string') {
                try {
                    const data = JSON.parse(event.data);
                    handleJsonMessage(data);
                } catch (e) {
                    console.error("JSON Parse Error:", e);
                }
            }
            // Binary Audio
            else if (event.data instanceof ArrayBuffer) {
                if (fsm?.getState() !== 'SPEAKING') {
                    fsm?.transitionTo('SPEAKING');
                }
                await playAudio(event.data);
            }
        };

        ws.onclose = () => {
            console.warn("🔴 WebSocket Disconnected");
            const delay = Math.min(1000 * Math.pow(2, reconnectCount++), 10000);
            setTimeout(connectWebSocket, delay);
        };

        ws.onerror = (e) => {
            console.error("WebSocket Error:", e);
        };
    }

    function handleJsonMessage(data) {
        console.log("📩 Message:", data);

        // Show Presentation
        if (data.answer || data.image_url) {
            animator?.enterPresentationMode(data);
        }

        // Update Emotion
        if (data.emotion) {
            animator?.setEmotion(data.emotion);
        }

        // Handle stream end
        if (data.action === 'AUDIO_STREAM_END') {
            // Audio will naturally end and trigger IDLE via onended
        }
    }

    // ==========================================
    // 7. Audio Playback
    // ==========================================
    async function playAudio(arrayBuffer) {
        if (audioCtx.state === 'suspended') {
            await audioCtx.resume();
        }

        try {
            const audioBuffer = await audioCtx.decodeAudioData(arrayBuffer.slice(0));
            const source = audioCtx.createBufferSource();
            source.buffer = audioBuffer;
            source.connect(gainNode);

            source.onended = () => {
                console.log("🔊 Audio chunk ended");
                if (audioQueue.length > 0) {
                    playNextInQueue();
                } else {
                    isPlaying = false;
                    // Return to IDLE after speaking
                    if (fsm?.getState() === 'SPEAKING') {
                        fsm.transitionTo('IDLE');
                    }
                }
            };

            if (isPlaying) {
                audioQueue.push(source);
            } else {
                isPlaying = true;
                source.start(0);
                animator?.startSpeaking();
            }
        } catch (e) {
            console.error("Audio Decode Error:", e);
        }
    }

    function playNextInQueue() {
        const source = audioQueue.shift();
        if (source) {
            source.start(0);
        }
    }

    // ==========================================
    // 8. FSM Callbacks
    // ==========================================
    function getFsmCallbacks() {
        return {
            onIdle: () => {
                updateStatus("กดปุ่มไมค์ 🎙️ เพื่อคุยกับน้องน่าน");
                animator?.setEmotion('normal');
                animator?.stopSpeaking();

                // Stop VAD if running
                if (isVADActive) {
                    vadInstance?.pause();
                    isVADActive = false;
                }

                // Show Mic Button
                if (UI.micBtn) {
                    UI.micBtn.style.display = 'flex';
                    UI.micBtn.classList.remove('active-recording');
                }

                // Hide Stop Button
                if (UI.stopBtn) UI.stopBtn.style.display = 'none';

                // Clear audio queue
                audioQueue = [];
                isPlaying = false;
            },

            onListening: () => {
                updateStatus("พูดได้เลยเจ้า... 🎤");
                animator?.setEmotion('listening');

                // Start VAD
                if (vadInstance && !isVADActive) {
                    vadInstance?.start();
                    isVADActive = true;
                    console.log("🎙️ VAD Started");
                }

                // Visual feedback
                if (UI.micBtn) {
                    UI.micBtn.classList.add('active-recording');
                    gsap?.to(UI.micBtn, { scale: 1.1, duration: 0.2 });
                }

                // Show Stop Button
                if (UI.stopBtn) UI.stopBtn.style.display = 'flex';
            },

            onThinking: (text) => {
                updateStatus(text || "กำลังคิด... 🧠");
                animator?.setEmotion('thinking');

                // Stop VAD
                if (isVADActive) {
                    vadInstance?.pause();
                    isVADActive = false;
                }

                // Reset mic button
                if (UI.micBtn) {
                    UI.micBtn.classList.remove('active-recording');
                    gsap?.to(UI.micBtn, { scale: 1, duration: 0.2 });
                }
            },

            onSpeaking: () => {
                updateStatus("น้องน่านกำลังตอบ... 🗣️");
                animator?.setEmotion('speaking');
                animator?.startSpeaking();

                // Hide mic during speaking
                if (UI.micBtn) UI.micBtn.style.display = 'none';
                // Show stop button so user can interrupt
                if (UI.stopBtn) UI.stopBtn.style.display = 'flex';
            },

            onError: (msg) => {
                console.error("FSM Error:", msg);
                updateStatus(`❌ ${msg}`);
            },

            onForceStop: () => {
                console.log("🛑 Force Stop");
                audioQueue = [];
                isPlaying = false;
                if (isVADActive) {
                    vadInstance?.pause();
                    isVADActive = false;
                }
            }
        };
    }

    // ==========================================
    // 9. Event Listeners
    // ==========================================

    // Mic Button - Toggle Listen
    if (UI.micBtn) {
        UI.micBtn.addEventListener('click', () => {
            const state = fsm?.getState();

            if (state === 'IDLE') {
                console.log("🎙️ Start Listening");
                fsm.transitionTo('LISTENING');
            } else if (state === 'LISTENING') {
                console.log("🎙️ Manual Stop");
                fsm.transitionTo('IDLE');
            } else if (state === 'SPEAKING') {
                console.log("🛑 Stop Speaking");
                fsm.transitionTo('IDLE');
            }
        });
    }

    // Stop Button
    if (UI.stopBtn) {
        UI.stopBtn.addEventListener('click', () => {
            console.log("🛑 Stop Button Clicked");
            fsm?.forceReset();
        });
    }

    // Text Input
    if (UI.inputForm) {
        UI.inputForm.addEventListener('submit', (e) => {
            e.preventDefault();
            const text = UI.inputField.value.trim();
            if (text && ws?.readyState === WebSocket.OPEN) {
                // Stop any current activity
                fsm?.transitionTo('THINKING', { text: 'กำลังคิด...' });

                // Send text query
                ws.send(JSON.stringify({ query: text, ai_mode: 'fast' }));
                UI.inputField.value = '';
            }
        });
    }

    // ==========================================
    // 10. Utility Functions
    // ==========================================

    // Convert Float32Array to WAV Blob
    function float32ToWav(float32Array, sampleRate) {
        const numChannels = 1;
        const bytesPerSample = 2; // 16-bit
        const blockAlign = numChannels * bytesPerSample;
        const byteRate = sampleRate * blockAlign;
        const dataSize = float32Array.length * bytesPerSample;
        const buffer = new ArrayBuffer(44 + dataSize);
        const view = new DataView(buffer);

        // WAV Header
        const writeString = (offset, str) => {
            for (let i = 0; i < str.length; i++) {
                view.setUint8(offset + i, str.charCodeAt(i));
            }
        };

        writeString(0, 'RIFF');
        view.setUint32(4, 36 + dataSize, true);
        writeString(8, 'WAVE');
        writeString(12, 'fmt ');
        view.setUint32(16, 16, true); // Subchunk1Size
        view.setUint16(20, 1, true);  // AudioFormat (PCM)
        view.setUint16(22, numChannels, true);
        view.setUint32(24, sampleRate, true);
        view.setUint32(28, byteRate, true);
        view.setUint16(32, blockAlign, true);
        view.setUint16(34, bytesPerSample * 8, true); // BitsPerSample
        writeString(36, 'data');
        view.setUint32(40, dataSize, true);

        // Convert samples
        let offset = 44;
        for (let i = 0; i < float32Array.length; i++) {
            const sample = Math.max(-1, Math.min(1, float32Array[i]));
            view.setInt16(offset, sample < 0 ? sample * 0x8000 : sample * 0x7FFF, true);
            offset += 2;
        }

        return new Blob([buffer], { type: 'audio/wav' });
    }

    // ==========================================
    // 11. Public API
    // ==========================================
    window.AvatarController = {
        getState: () => fsm?.getState(),
        forceStop: () => fsm?.forceReset()
    };
});