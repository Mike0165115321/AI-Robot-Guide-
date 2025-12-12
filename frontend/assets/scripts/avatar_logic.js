// /frontend/assets/scripts/avatar_logic.js (V-Separate Page Mode)

document.addEventListener('DOMContentLoaded', () => {

    const statusText = document.getElementById('status-text');
    const stopSpeechButton = document.getElementById('stop-speech-btn');
    const musicControls = document.getElementById('music-controls');
    const textQueryForm = document.getElementById('text-query-form');

    let websocket = null;
    let mainAudioContext = null;
    let currentAudioSource = null;
    let conversationLoopActive = false; // เริ่มต้นยังไม่ฟัง จนกว่า setActive จะถูกเรียก

    let musicHandler = null;
    let voiceHandler = null;

    async function waitForAnimator() {
        let attempts = 0;
        while (!window.avatarAnimator && attempts < 50) {
            await new Promise(r => setTimeout(r, 100));
            attempts++;
        }
        return window.avatarAnimator;
    }

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
        }
    };

    // API
    window.AvatarController = {
        setActive: async (active) => {
            if (active) {
                console.log("🟢 Avatar Start Sequence...");
                await waitForAnimator();

                if (!mainAudioContext) await initializeAudioSystem();
                if (mainAudioContext?.state === 'suspended') await mainAudioContext.resume();

                setTimeout(() => {
                    startConversationLoop();
                }, 1000);
            }
        }
    };

    function startConversationLoop() {
        conversationLoopActive = true;
        stopCurrentAudio();

        uiController.setStatus("ฟังอยู่จ้าว... (พูดได้เลย)");
        uiController.setEmotion('listening');

        if (stopSpeechButton) stopSpeechButton.classList.add('visible');
        if (voiceHandler && !voiceHandler.isListening) voiceHandler.start();
    }

    function stopConversationLoop(forceStop = false) {
        if (forceStop) conversationLoopActive = false;
        stopCurrentAudio();
        if (voiceHandler) voiceHandler.stop(true);

        uiController.setEmotion('normal');
        uiController.setStatus("พักผ่อน... (พิมพ์หรือกดเริ่มใหม่)");

        if (forceStop && musicHandler) musicHandler.reset();
        if (forceStop && stopSpeechButton) stopSpeechButton.classList.remove('visible');
    }

    function stopCurrentAudio() {
        if (currentAudioSource) {
            try { currentAudioSource.stop(); } catch (e) { }
            currentAudioSource = null;
        }
        if (window.avatarAnimator) window.avatarAnimator.stopSpeaking();
    }

    async function playAudio(audioData) {
        if (!mainAudioContext) return;
        stopCurrentAudio();

        try {
            const buffer = await (audioData instanceof Blob ? audioData.arrayBuffer() : audioData);
            const audioBuffer = await mainAudioContext.decodeAudioData(buffer);
            const source = mainAudioContext.createBufferSource();
            source.buffer = audioBuffer;
            source.connect(mainAudioContext.destination);

            if (window.avatarAnimator) window.avatarAnimator.startSpeaking();
            uiController.setEmotion('speaking');

            return new Promise((resolve) => {
                source.onended = () => {
                    if (window.avatarAnimator) window.avatarAnimator.stopSpeaking();
                    currentAudioSource = null;
                    resolve();
                };
                source.start(0);
                currentAudioSource = source;
            });
        } catch (e) {
            console.error("Audio Play Error:", e);
            if (conversationLoopActive) startConversationLoop();
        }
    }

    async function initializeAudioSystem() {
        try {
            if (!mainAudioContext) mainAudioContext = new (window.AudioContext || window.webkitAudioContext)();

            if (!voiceHandler) {
                voiceHandler = new VoiceHandler(mainAudioContext, {
                    onStatusUpdate: (text) => {
                        // Update status only if needed
                    },
                    onSpeechEnd: (audioBlob) => {
                        uiController.setEmotion('thinking');
                        uiController.setStatus("กำลังคิด...");

                        // [Fix] Stop listening immediately to prevent feedback loop
                        if (voiceHandler) voiceHandler.stop(false);

                        if (websocket?.readyState === WebSocket.OPEN) websocket.send(audioBlob);
                    }
                });
            }
            if (!musicHandler) {
                musicHandler = new AvatarMusicHandler(websocket, uiController, voiceHandler, null, {
                    stopSpeechButton: stopSpeechButton,
                    musicControls: musicControls,
                    stopAISpeechAudio: stopCurrentAudio,
                    resetToListeningState: () => { if (conversationLoopActive) startConversationLoop(); }
                });
            }
        } catch (e) { console.error(e); }
    }

    function connectWebSocket() {
        if (typeof API_HOST === 'undefined') return;
        websocket = new WebSocket(`ws://${API_HOST}:${API_PORT}/api/avatar/ws`);
        websocket.binaryType = 'arraybuffer';

        websocket.onopen = () => {
            console.log("WS Connected");
            if (aiModeManager) {
                const currentMode = aiModeManager.getMode();
                websocket.send(JSON.stringify({
                    "action": "SET_MODE",
                    "ai_mode": currentMode
                }));
                console.log("Synced AI Mode on Connect:", currentMode);
            }
        };

        websocket.onmessage = async (event) => {
            if (typeof event.data === 'string') {
                const data = JSON.parse(event.data);
                if (musicHandler && musicHandler.handleMessage(data)) {
                    conversationLoopActive = false;
                    return;
                }
                const hasVisual = data.image_url || data.image_gallery?.length || data.sources?.length || (data.action === 'SHOW_MAP_EMBED');

                // Trigger presentation if there are visuals OR if there is a text answer
                const shouldEnterPresentation = hasVisual || (data.answer && data.answer.length > 0);

                if (shouldEnterPresentation) {
                    uiController.enterPresentation(data);
                } else if (data.answer) {
                    uiController.setStatus(data.answer);
                }

                uiController.setEmotion(data.emotion || 'talking');

            } else if (event.data instanceof ArrayBuffer) {
                if (voiceHandler) voiceHandler.stop(false);
                await playAudio(event.data);

                // [FIX] เพิ่ม delay ก่อนเริ่มฟังใหม่ - ป้องกัน AI พูดแทรก/จับเสียงสะท้อน
                if (conversationLoopActive && !musicHandler.isPlaying()) {
                    setTimeout(() => {
                        if (conversationLoopActive) {
                            startConversationLoop();
                        }
                    }, 1000); // รอ 1 วินาทีก่อนเริ่มฟังใหม่
                }
            }
        };

        websocket.onclose = () => setTimeout(connectWebSocket, 3000);
    }

    // Listeners
    if (stopSpeechButton) {
        stopSpeechButton.addEventListener('click', () => stopConversationLoop(true));
    }

    if (textQueryForm) {
        textQueryForm.addEventListener('submit', (e) => {
            e.preventDefault();
            const input = document.getElementById('text-query-input');
            const text = input.value.trim();
            if (text && websocket) {
                stopCurrentAudio();
                if (voiceHandler) voiceHandler.stop(false);
                websocket.send(JSON.stringify({ "query": text }));
                input.value = "";
                uiController.setEmotion('thinking');
                uiController.setStatus("กำลังอ่าน...");
                conversationLoopActive = true;
            }
        });
    }

    // FAB Panel Toggle
    const fabToggle = document.getElementById('fab-toggle');
    const fabActions = document.getElementById('fab-actions');

    if (fabToggle && fabActions) {
        fabToggle.addEventListener('click', () => {
            fabToggle.classList.toggle('active');
            fabActions.classList.toggle('open');
        });

        // ปิด FAB เมื่อคลิกที่ปุ่ม action
        fabActions.querySelectorAll('.fab-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                fabToggle.classList.remove('active');
                fabActions.classList.remove('open');
            });
        });
    }

    // FAB Button Handlers
    const avatarMusicBtn = document.getElementById('avatar-music-btn');
    const avatarFaqBtn = document.getElementById('avatar-faq-btn');
    const avatarCalcBtn = document.getElementById('avatar-calc-btn');

    // Definitions
    const aiModeBtn = document.getElementById('ai-mode-toggle');

    // AI Mode Manager
    let aiModeManager = null;
    if (window.NanApp && window.NanApp.AIModeManager) {
        aiModeManager = new window.NanApp.AIModeManager();
        updateAIModeUI(aiModeManager.getMode());
    }

    function sendQuery(query, intent = null, additionalData = {}) {
        if (websocket && websocket.readyState === WebSocket.OPEN) {
            stopCurrentAudio();
            if (voiceHandler) voiceHandler.stop(false);

            const mode = aiModeManager ? aiModeManager.getMode() : 'fast';

            // Base Payload
            const payload = {
                "query": query,
                "ai_mode": mode,
                ...additionalData
            };

            // Add intent if available
            if (intent) {
                payload.intent = intent;
            }

            websocket.send(JSON.stringify(payload));

            uiController.setEmotion('thinking');
            uiController.setStatus("กำลังค้นหา...");
            conversationLoopActive = true;
        }
    }

    // AI Mode UI Handlers
    if (aiModeBtn && aiModeManager) {
        aiModeBtn.addEventListener('click', () => {
            const newMode = aiModeManager.toggle();
            updateAIModeUI(newMode);

            // [FIX] Sync mode with backend immediately for Voice interactions
            if (websocket && websocket.readyState === WebSocket.OPEN) {
                websocket.send(JSON.stringify({
                    "action": "SET_MODE",
                    "ai_mode": newMode
                }));
                console.log("Updated AI Mode to:", newMode);
            }
        });
    }

    function updateAIModeUI(mode) {
        if (!aiModeBtn) return;
        const icon = aiModeBtn.querySelector('i');
        const text = aiModeBtn.querySelector('span');

        if (mode === 'fast') {
            aiModeBtn.style.background = 'rgba(16, 185, 129, 0.2)'; // Green tint
            aiModeBtn.style.borderColor = '#10b981';
            if (icon) icon.className = 'fas fa-bolt';
            if (text) text.textContent = 'คิดเร็ว';
        } else {
            aiModeBtn.style.background = 'rgba(59, 130, 246, 0.2)'; // Blue tint
            aiModeBtn.style.borderColor = '#3b82f6';
            if (icon) icon.className = 'fas fa-brain';
            if (text) text.textContent = 'คิดละเอียด';
        }
    }

    // 🚀 Initialize FabManager for handling Tool Buttons
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
                // Shared sendMessage implementation
                sendMessage: (text, intent, additionalData) => sendQuery(text, intent, additionalData),

                // Music: Custom logic for Avatar
                onMusicAction: () => {
                    if (musicHandler) {
                        musicHandler.handleMessage({ action: 'PROMPT_FOR_SONG_INPUT' });
                    } else {
                        console.error("MusicHandler not ready");
                    }
                },

                // FAQ: Use default sendMessage (via callback)
                // FabManager calls sendMessage("...", INTENTS.FAQ)

                // Calc: Use default sendMessage (via callback)
                // FabManager calls sendMessage("...", INTENTS.CALCULATOR)

                // Nav: Custom Avatar Logic
                // Nav: Custom Avatar Logic using FabManager Widget
                onNavAction: () => {
                    const resultText = document.getElementById('result-text');
                    const infoDisplay = document.getElementById('info-display');

                    if (resultText && infoDisplay) {
                        // Clear old content
                        resultText.innerHTML = '';
                        infoDisplay.innerHTML = '';

                        // Create Widget from FabManager
                        // Note: FabManager inside logic calls fetch/api directly.
                        const fabManager = window.NanApp?.fabManager || new FabManager({
                            callbacks: { sendMessage: (t, i, d) => sendQuery(t, i, d) } // temp instance if needed, but best if reused
                        });

                        // Creating a fresh widget instance 
                        const widget = fabManager.createNavigationWidget();

                        // Append to info display
                        infoDisplay.appendChild(widget);

                        // Enter presentation mode
                        if (window.avatarAnimator) {
                            window.avatarAnimator.enterPresentationMode({ html_is_pre_rendered: true });
                        }
                        if (window.uiController) {
                            window.uiController.setEmotion('listening');
                            window.uiController.setStatus("รอเลือกสถานที่...");
                        }
                    }
                }
            }
        });
    }

    connectWebSocket();
});