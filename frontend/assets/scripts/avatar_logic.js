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

        websocket.onopen = () => console.log("WS Connected");

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

                if (conversationLoopActive && !musicHandler.isPlaying()) {
                    startConversationLoop();
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

    connectWebSocket();
});