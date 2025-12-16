// /frontend/assets/scripts/avatar_logic.js (V-FSM Stable V2.1 - Audio Fix + Mute)

document.addEventListener('DOMContentLoaded', () => {

    const statusText = document.getElementById('status-text');
    const stopSpeechButton = document.getElementById('stop-speech-btn');
    const musicControls = document.getElementById('music-controls');
    const textQueryForm = document.getElementById('text-query-form');

    // Mute Button Injection
    let muteButton = document.getElementById('mute-btn');
    if (!muteButton) {
        // Create it dynamically if missing
        muteButton = document.createElement('button');
        muteButton.id = 'mute-btn';
        muteButton.className = 'stop-speech-button'; // Re-use style
        muteButton.style.cssText = 'left: calc(50% + 70px); background: rgba(59, 130, 246, 0.9); display: none;'; // Position to right
        muteButton.innerHTML = '<i class="fas fa-volume-up"></i>';
        muteButton.title = 'ปิดเสียง';
        document.body.appendChild(muteButton);
    }

    // 🌐 Globals
    let websocket = null;
    let mainAudioContext = null;
    let mainGainNode = null; // 🎚️ Master Volume
    let currentAudioSource = null;
    let isMuted = false;

    // Handlers
    let musicHandler = null;
    let voiceHandler = null;
    let wakeWordHandler = null;
    let aiModeManager = null;

    // 🔊 Audio Queue
    let audioQueue = [];
    let isPlayingAudio = false;
    let isServerResponseComplete = false;

    // 🤖 State Manager initialized later
    let stateManager = null;

    // =========================================================================
    // 🛠️ Core Utilities
    // =========================================================================

    const uiController = {
        setEmotion(emotion) {
            if (window.avatarAnimator?.setEmotion) window.avatarAnimator.setEmotion(emotion);
        },
        setStatus(text) {
            if (statusText) statusText.textContent = text;
        },
        enterPresentation(data) {
            if (window.avatarAnimator?.enterPresentationMode) window.avatarAnimator.enterPresentationMode(data);
        },
        exitPresentation() {
            if (window.avatarAnimator?.exitPresentationMode) window.avatarAnimator.exitPresentationMode();
        },
        toggleMuteUI(muted) {
            if (muteButton) {
                muteButton.innerHTML = muted ? '<i class="fas fa-volume-mute"></i>' : '<i class="fas fa-volume-up"></i>';
                muteButton.style.background = muted ? 'rgba(107, 114, 128, 0.9)' : 'rgba(59, 130, 246, 0.9)';
                muteButton.title = muted ? 'เปิดเสียง' : 'ปิดเสียง';
            }
        }
    };

    async function waitForAnimator() {
        let attempts = 0;
        while (!window.avatarAnimator && attempts < 50) {
            await new Promise(r => setTimeout(r, 100));
            attempts++;
        }
        return window.avatarAnimator;
    }

    // 🔥 Hard Audio Reset (The "Kill Switch")
    async function hardResetAudio() {
        console.warn("💀 Hard Resetting Audio System...");

        // 1. Stop Current Logic
        if (currentAudioSource) {
            try { currentAudioSource.stop(); } catch (e) { }
            currentAudioSource = null;
        }
        audioQueue = [];
        isPlayingAudio = false;

        // 2. Kill Context if possible (or just suspend/resume to clear buffers)
        if (mainAudioContext) {
            try {
                await mainAudioContext.suspend();
                await mainAudioContext.resume();
            } catch (e) {
                console.error("Failed to cycle AudioContext:", e);
            }
        }

        // 3. Reset Handlers
        if (wakeWordHandler) wakeWordHandler.stop();
        if (voiceHandler) voiceHandler.stop(true);
        if (window.avatarAnimator) window.avatarAnimator.stopSpeaking();
    }

    // =========================================================================
    // 🧠 Finite State Machine Callbacks
    // =========================================================================

    const fsmCallbacks = {
        onIdle: () => {
            console.log("[State] IDLE");
            uiController.setStatus("🎤 พูดว่า \"น้องน่าน\" เพื่อเริ่ม...");
            uiController.setEmotion('normal');

            // Logic: WakeWord=ON, Voice=OFF, Audio=OFF
            stopCurrentAudio();
            if (voiceHandler) voiceHandler.stop(true);

            if (wakeWordHandler) {
                wakeWordHandler.start(); // Wait for "Nong Nan"
            }

            // Reset Buttons
            if (stopSpeechButton) stopSpeechButton.classList.remove('visible');
            if (muteButton) muteButton.style.display = 'none';
        },

        onListening: () => {
            console.log("[State] LISTENING");
            uiController.setStatus("ฟังอยู่จ้าว... (พูดได้เลย)");
            uiController.setEmotion('listening');

            // Logic: WakeWord=OFF, Voice=ON, Audio=OFF
            if (wakeWordHandler) wakeWordHandler.stopListening();
            stopCurrentAudio();

            setTimeout(() => {
                if (stateManager.getState() === 'LISTENING' && voiceHandler) {
                    voiceHandler.start();
                }
            }, 200);

            if (stopSpeechButton) stopSpeechButton.classList.add('visible');
            if (muteButton) muteButton.style.display = 'none'; // Hide mute when listening
        },

        onThinking: (text) => {
            console.log("[State] THINKING");
            uiController.setStatus(text || "กำลังประมวลผล...");
            uiController.setEmotion('thinking');

            // Logic: ALL OFF
            if (wakeWordHandler) wakeWordHandler.stopListening(); // Stay stopped
            if (voiceHandler) voiceHandler.stop(false); // Stop recording
            stopCurrentAudio();

            if (stopSpeechButton) stopSpeechButton.classList.add('visible');
            // Allow mute button during thinking/speaking
            if (muteButton) {
                muteButton.style.display = 'flex';
                muteButton.classList.add('visible', 'popIn');
            }
        },

        onSpeaking: () => {
            console.log("[State] SPEAKING");
            uiController.setEmotion('speaking');

            // Logic: ALL INPUTS OFF
            if (wakeWordHandler) wakeWordHandler.stopListening();
            if (voiceHandler) voiceHandler.stop(true);

            if (stopSpeechButton) stopSpeechButton.classList.add('visible');
            if (muteButton) {
                muteButton.style.display = 'flex';
                muteButton.classList.add('visible', 'popIn');
            }
        },

        onError: (msg) => {
            console.error("[FSM Error]", msg);
            uiController.setStatus(msg);
            uiController.setEmotion('normal');
        },

        onForceStop: () => {
            hardResetAudio();
        }
    };

    // Initialize State Manager
    if (window.AvatarStateManager) {
        stateManager = new AvatarStateManager(fsmCallbacks);
    } else {
        console.error("AvatarStateManager not found!");
    }

    // =========================================================================
    // 🔌 WebSocket Logic
    // =========================================================================

    function connectWebSocket() {
        if (typeof API_HOST === 'undefined') return;
        websocket = new WebSocket(`ws://${API_HOST}:${API_PORT}/api/avatar/ws`);
        websocket.binaryType = 'arraybuffer';

        websocket.onopen = () => {
            console.log("WS Connected");
            if (aiModeManager) {
                const currentMode = aiModeManager.getMode();
                websocket.send(JSON.stringify({ "action": "SET_MODE", "ai_mode": currentMode }));
            }
        };

        websocket.onmessage = async (event) => {
            // 1. Text / JSON Message
            if (typeof event.data === 'string') {
                const data = JSON.parse(event.data);

                if (data.action === "AUDIO_STREAM_END") {
                    console.log("[Stream] EOS Received");
                    isServerResponseComplete = true;
                    if (audioQueue.length === 0 && !isPlayingAudio) {
                        handlePlaybackFinished();
                    }
                    return;
                }

                if (musicHandler && musicHandler.handleMessage(data)) {
                    stateManager.transitionTo('IDLE');
                    return;
                }

                const hasVisual = data.image_url || data.image_gallery?.length || data.sources?.length || (data.action === 'SHOW_MAP_EMBED');
                const shouldEnterPresentation = hasVisual || (data.answer && data.answer.length > 0);

                if (shouldEnterPresentation) {
                    uiController.enterPresentation(data);
                    // Update status strictly to short description
                    uiController.setStatus("กำลังนำเสนอข้อมูล...");
                } else if (!data.answer && data.emotion) {
                    // Only update emotional status if no text answer (e.g. "Thinking")
                }

                // [FIX] Do NOT dump raw answer into Status Text
                // if (data.answer) uiController.setStatus(data.answer);

                if (data.emotion && stateManager.getState() !== 'SPEAKING') {
                    uiController.setEmotion(data.emotion);
                }

            }
            // 2. Audio Binary Chunk
            else if (event.data instanceof ArrayBuffer) {
                if (stateManager.getState() !== 'SPEAKING') {
                    stateManager.transitionTo('SPEAKING');
                }

                // 🛑 DISABLED LEGACY AudioStreamManager to prevent double audio
                // if (window.audioStreamManager) audioStreamManager.enqueue(event.data); 

                queueAudio(event.data);
            }
        };

        websocket.onclose = () => setTimeout(connectWebSocket, 3000);
    }

    function sendAudioToBackend(audioBlob) {
        if (websocket && websocket.readyState === WebSocket.OPEN) {
            websocket.send(audioBlob);
        } else {
            uiController.setStatus("Error: No Connection");
            stateManager.transitionTo('IDLE');
        }
    }

    function sendQuery(query, intent = null, additionalData = {}) {
        if (websocket && websocket.readyState === WebSocket.OPEN) {
            hardResetAudio();

            const mode = aiModeManager ? aiModeManager.getMode() : 'fast';
            const payload = { "query": query, "ai_mode": mode, ...additionalData };
            if (intent) payload.intent = intent;

            websocket.send(JSON.stringify(payload));

            stateManager.transitionTo('THINKING', { text: "กำลังค้นหา..." });
        }
    }

    // =========================================================================
    // 🔊 Audio Playback Logic (Streaming + Mute Support)
    // =========================================================================

    function stopCurrentAudio() {
        if (currentAudioSource) {
            try { currentAudioSource.stop(); } catch (e) { }
            currentAudioSource = null;
        }
        if (window.avatarAnimator) window.avatarAnimator.stopSpeaking();
    }

    async function queueAudio(audioData) {
        if (stateManager.getState() !== 'SPEAKING' && stateManager.getState() !== 'THINKING') {
            return;
        }
        audioQueue.push(audioData);
        processAudioQueue();
    }

    async function processAudioQueue() {
        if (isPlayingAudio || audioQueue.length === 0) return;

        isPlayingAudio = true;
        const audioData = audioQueue.shift();

        try {
            await playAudioChunk(audioData);
        } catch (e) {
            console.error("Play Error:", e);
        } finally {
            isPlayingAudio = false;
            processAudioQueue();
        }
    }

    async function playAudioChunk(audioData) {
        if (!mainAudioContext) return;

        // Ensure state is Speaking
        if (stateManager.getState() !== 'SPEAKING') stateManager.transitionTo('SPEAKING');

        try {
            const buffer = await (audioData instanceof Blob ? audioData.arrayBuffer() : audioData);
            const audioBuffer = await mainAudioContext.decodeAudioData(buffer);

            const source = mainAudioContext.createBufferSource();
            source.buffer = audioBuffer;

            // 🎚️ GAIN NODE Logic (Volume/Mute)
            if (!mainGainNode) {
                mainGainNode = mainAudioContext.createGain();
                mainGainNode.connect(mainAudioContext.destination);
            }

            // Apply Mute State
            mainGainNode.gain.value = isMuted ? 0 : 1;

            // Connect Source -> Gain -> Destination
            source.connect(mainGainNode);

            // Lip Sync Trigger (Even if muted, we might want lips to move? 
            // The user said "Mute... slide continues... maybe lips stop or not".
            // Let's keep lips moving for realism, or stop them if muted?
            // "Mute: reduce volume to 0 but do NOT stop playback".
            // So lips SHOULD move technically, or we can disable them.
            // Let's keep them moving as per plan "Animation ปากอาจขยับต่อ").
            if (window.avatarAnimator && !window.avatarAnimator.isSpeaking) {
                window.avatarAnimator.startSpeaking();
            }

            return new Promise((resolve) => {
                source.onended = () => {
                    currentAudioSource = null;
                    if (audioQueue.length === 0) {
                        if (isServerResponseComplete) {
                            handlePlaybackFinished();
                        }
                    }
                    resolve();
                };
                source.start(0);
                currentAudioSource = source;
            });
        } catch (e) {
            return Promise.resolve();
        }
    }

    function handlePlaybackFinished() {
        console.log("[Stream] Playback Finished.");
        setTimeout(() => {
            if (stateManager.getState() === 'SPEAKING') {
                stateManager.transitionTo('LISTENING');
            }
        }, 500);
        isServerResponseComplete = false;
    }


    // =========================================================================
    // 🏗️ Initialization & Event Wiring
    // =========================================================================

    async function initializeAudioSystem() {
        if (!mainAudioContext) {
            const AudioContext = window.AudioContext || window.webkitAudioContext;
            mainAudioContext = new AudioContext();
        }
        if (mainAudioContext.state === 'suspended') {
            try {
                // Attempt to resume, but don't crash if blocked by policy
                await mainAudioContext.resume().catch(e => console.warn("Audio Autoplay Blocked (Will resume on click):", e));
            } catch (e) {
                console.warn("Audio Context Resume Error:", e);
            }
        }

        // Create Master Gain
        if (!mainGainNode) {
            mainGainNode = mainAudioContext.createGain();
            mainGainNode.connect(mainAudioContext.destination);
        }

        // 1. Wake Handlers
        if (!wakeWordHandler && typeof WakeWordHandler !== 'undefined') {
            wakeWordHandler = new WakeWordHandler({
                onStatusUpdate: () => { },
                onWakeWordDetected: () => {
                    console.log("[WakeWord] Detected!");
                    stateManager.transitionTo('LISTENING');
                }
            });
        }

        // 2. Voice Handler
        if (!voiceHandler && typeof VoiceHandler !== 'undefined') {
            voiceHandler = new VoiceHandler(mainAudioContext, {
                onStatusUpdate: (msg) => {
                    if (stateManager.getState() === 'LISTENING') uiController.setStatus(msg);
                },
                onSpeechEnd: (audioBlob) => {
                    if (stateManager.getState() === 'LISTENING') {
                        stateManager.transitionTo('THINKING', { text: "กำลังฟัง..." });
                        sendAudioToBackend(audioBlob);
                    }
                }
            });
        }

        // 3. Music (Legacy)
        if (!musicHandler && typeof AvatarMusicHandler !== 'undefined') {
            const timerManager = { clearPresentationTimeout: () => { }, startPresentationTimeout: () => { } };
            musicHandler = new AvatarMusicHandler(websocket, uiController, voiceHandler, timerManager, {
                musicControls: musicControls,
                stopAISpeechAudio: stopCurrentAudio,
                stopSpeechButton: stopSpeechButton,
                resetToListeningState: () => stateManager.transitionTo('LISTENING')
            });
        }
    }

    window.AvatarController = {
        setActive: async (active) => {
            if (active) {
                console.log("🟢 Avatar Controller Activated");
                await waitForAnimator();
                await initializeAudioSystem();
                stateManager.transitionTo('IDLE');
            }
        }
    };

    // UI Event Listeners
    if (stopSpeechButton) {
        stopSpeechButton.addEventListener('click', () => {
            stateManager.forceReset();
        });
    }

    // Mute Button Listener
    if (muteButton) {
        muteButton.addEventListener('click', () => {
            isMuted = !isMuted;
            if (mainGainNode) {
                mainGainNode.gain.value = isMuted ? 0 : 1;
            }
            uiController.toggleMuteUI(isMuted);
        });
    }

    if (textQueryForm) {
        textQueryForm.addEventListener('submit', (e) => {
            e.preventDefault();
            const input = document.getElementById('text-query-input');
            const text = input.value.trim();
            if (text) {
                sendQuery(text);
                input.value = "";
            }
        });
    }

    // AI Mode Setup
    if (window.NanApp && window.NanApp.AIModeManager) {
        aiModeManager = new window.NanApp.AIModeManager();
        const aiModeBtn = document.getElementById('ai-mode-toggle');

        const updateUI = (mode) => {
            if (aiModeBtn) {
                const icon = aiModeBtn.querySelector('i');
                const span = aiModeBtn.querySelector('span');
                if (mode === 'fast') {
                    icon.className = 'fas fa-bolt'; span.textContent = 'คิดเร็ว';
                    aiModeBtn.style.borderColor = '#10b981';
                } else {
                    icon.className = 'fas fa-brain'; span.textContent = 'คิดละเอียด';
                    aiModeBtn.style.borderColor = '#3b82f6';
                }
            }
        };
        updateUI(aiModeManager.getMode());

        if (aiModeBtn) {
            aiModeBtn.addEventListener('click', () => {
                const newMode = aiModeManager.toggle();
                updateUI(newMode);
                if (websocket) websocket.send(JSON.stringify({ "action": "SET_MODE", "ai_mode": newMode }));
            });
        }
    }

    // Fab Manager Setup
    if (window.FabManager) {
        new FabManager({
            isAvatarMode: true,
            buttons: {
                music: 'avatar-music-btn',
                faq: 'avatar-faq-btn',
                calc: 'avatar-calc-btn',
                nav: 'avatar-nav-btn'
            },
            callbacks: {
                sendMessage: (text, intent, extra) => sendQuery(text, intent, extra),
                onMusicAction: () => {
                    const infoDisplay = document.getElementById('info-display');
                    if (infoDisplay) {
                        infoDisplay.innerHTML = '';
                        const fm = new FabManager({ callbacks: { sendMessage: (t, i, d) => sendQuery(t, i, d) } });
                        infoDisplay.appendChild(fm.createMusicWidget());
                        if (window.avatarAnimator) window.avatarAnimator.enterPresentationMode({ html_is_pre_rendered: true });
                        stateManager.transitionTo('LISTENING');
                    }
                },
                onFaqAction: () => {
                    const infoDisplay = document.getElementById('info-display');
                    if (infoDisplay) {
                        infoDisplay.innerHTML = '';
                        const fm = new FabManager({ callbacks: { sendMessage: (t, i, d) => sendQuery(t, i, d) } });
                        infoDisplay.appendChild(fm.createFAQWidget());
                        if (window.avatarAnimator) window.avatarAnimator.enterPresentationMode({ html_is_pre_rendered: true });
                        stateManager.transitionTo('LISTENING');
                    }
                },
                onNavAction: () => {
                    const infoDisplay = document.getElementById('info-display');
                    if (infoDisplay) {
                        infoDisplay.innerHTML = '';
                        const fm = new FabManager({ callbacks: { sendMessage: (t, i, d) => sendQuery(t, i, d) } });
                        infoDisplay.appendChild(fm.createNavigationWidget());
                        if (window.avatarAnimator) window.avatarAnimator.enterPresentationMode({ html_is_pre_rendered: true });
                        stateManager.transitionTo('LISTENING');
                    }
                },
                onCalcAction: () => {
                    const infoDisplay = document.getElementById('info-display');
                    if (infoDisplay) {
                        infoDisplay.innerHTML = '';
                        const fm = new FabManager({ callbacks: { sendMessage: (t, i, d) => sendQuery(t, i, d) } });
                        infoDisplay.appendChild(fm.createCalculatorWidget());
                        if (window.avatarAnimator) window.avatarAnimator.enterPresentationMode({ html_is_pre_rendered: true });
                        stateManager.transitionTo('LISTENING');
                    }
                }
            }
        });
    }

    connectWebSocket();
});