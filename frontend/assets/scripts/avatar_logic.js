// /frontend/assets/scripts/avatar_logic.js (V-Separate Page Mode + Wake Word)

document.addEventListener('DOMContentLoaded', () => {

    const statusText = document.getElementById('status-text');
    const stopSpeechButton = document.getElementById('stop-speech-btn');
    const musicControls = document.getElementById('music-controls');
    const textQueryForm = document.getElementById('text-query-form');

    let websocket = null;
    // 🎧 ตัวจัดการเสียงและสถานะการสนทนา
    let mainAudioContext = null;
    let currentAudioSource = null;
    let conversationLoopActive = false; // เริ่มต้นยังไม่ฟัง จนกว่า setActive จะถูกเรียก

    let musicHandler = null;
    let voiceHandler = null;
    let wakeWordHandler = null; // 🎤 Wake Word Handler



    function sendAudioToBackend(audioBlob) {
        if (websocket && websocket.readyState === WebSocket.OPEN) {
            console.log(`📤 Sending Audio: ${audioBlob.size} bytes`);
            websocket.send(audioBlob);
        } else {
            console.error("❌ WebSocket not connected");
            uiController.setStatus("ขาดการเชื่อมต่อ");
        }
    }

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
                console.log("🟢 Avatar Start Sequence (Wake Word Mode)...");
                // ⏳ รอให้ AvatarAnimator พร้อมใช้งาน (โหลดโมเดล 3D เสร็จ)
                await waitForAnimator();

                if (!mainAudioContext) await initializeAudioSystem();
                if (mainAudioContext?.state === 'suspended') await mainAudioContext.resume();

                setTimeout(() => {
                    // 🎤 เริ่มด้วย Wake Word Mode แทน Continuous
                    startWakeWordMode();
                }, 1000);
            }
        }
    };

    // 🎤 เริ่มโหมด Wake Word (รอคำเรียก)
    function startWakeWordMode() {
        conversationLoopActive = true;
        stopCurrentAudio();

        if (stopSpeechButton) stopSpeechButton.classList.add('visible');

        // หยุด VoiceHandler ถ้าทำงานอยู่
        if (voiceHandler && voiceHandler.isListening) voiceHandler.stop(true);

        // เริ่ม Wake Word Handler
        if (wakeWordHandler) {
            wakeWordHandler.start();
        }

        uiController.setStatus("🎤 พูดว่า \"น้องน่าน\" เพื่อเริ่มสนทนา...");
        uiController.setEmotion('normal');
    }

    // 🎙️ เริ่มโหมดฟังต่อเนื่อง (หลังตรวจพบ Wake Word)
    function startConversationLoop() {
        console.log('🔵 [DEBUG] startConversationLoop() called');
        console.log('🔵 [DEBUG] voiceHandler exists:', !!voiceHandler);
        console.log('🔵 [DEBUG] voiceHandler.isListening:', voiceHandler?.isListening);
        console.log('🔵 [DEBUG] wakeWordHandler.isListening:', wakeWordHandler?.isListening);

        conversationLoopActive = true;
        stopCurrentAudio();

        // 🔥 [FIX] หยุด WakeWordHandler ก่อน เพื่อปล่อยไมค์ให้ VoiceHandler
        if (wakeWordHandler && wakeWordHandler.isListening) {
            console.log('🔵 [DEBUG] Stopping WakeWordHandler...');
            wakeWordHandler.stopListening();
        }

        uiController.setStatus("ฟังอยู่จ้าว... (พูดได้เลย)");
        uiController.setEmotion('listening');

        if (stopSpeechButton) stopSpeechButton.classList.add('visible');

        // เพิ่ม delay เล็กน้อยเพื่อให้ไมค์พร้อมใช้งาน
        setTimeout(() => {
            console.log('🔵 [DEBUG] After 300ms delay:');
            console.log('🔵 [DEBUG] voiceHandler exists:', !!voiceHandler);
            console.log('🔵 [DEBUG] voiceHandler.isListening:', voiceHandler?.isListening);
            console.log('🔵 [DEBUG] conversationLoopActive:', conversationLoopActive);

            if (voiceHandler && !voiceHandler.isListening && conversationLoopActive) {
                console.log('[Avatar] 🎙️ Starting VoiceHandler...');
                console.log('🟢 [DEBUG] Starting VoiceHandler NOW!');
                voiceHandler.start();
            } else {
                console.log('🔴 [DEBUG] VoiceHandler NOT started because:');
                if (!voiceHandler) console.log('   - voiceHandler is null');
                if (voiceHandler?.isListening) console.log('   - voiceHandler already listening');
                if (!conversationLoopActive) console.log('   - conversationLoopActive is false');
            }
        }, 300);
    }

    function clearAudioQueue() {
        audioQueue = []; // Clear pending chunks
        isPlayingAudio = false;
        isServerResponseComplete = false;
        stopCurrentAudio(); // Stop currently playing sound
    }

    function stopConversationLoop(forceStop = false) {
        if (forceStop) {
            conversationLoopActive = false;
            // 🛑 Clear queue immediately to prevent next chunk from playing
            clearAudioQueue();
        }
        stopCurrentAudio();
        if (voiceHandler) voiceHandler.stop(true);
        if (wakeWordHandler) wakeWordHandler.stop();

        // Send Interrupted Signal to Backend (Optional but good practice)
        // if (websocket && websocket.readyState === WebSocket.OPEN && forceStop) {
        //    websocket.send(JSON.stringify({ action: "STOP" }));
        // }

        uiController.setEmotion('normal');
        uiController.setStatus("พักผ่อน... (พิมพ์หรือกดเริ่มใหม่)");

        if (forceStop && musicHandler) musicHandler.reset();

        // เปลี่ยนปุ่มหยุดเป็นปุ่มเริ่มใหม่
        if (forceStop && stopSpeechButton) {
            stopSpeechButton.innerHTML = '<i class="fas fa-play"></i>';
            stopSpeechButton.style.backgroundColor = 'rgba(16, 185, 129, 0.9)'; // สีเขียว
            stopSpeechButton.title = 'เริ่มใหม่';
            stopSpeechButton.dataset.mode = 'start';
        }
    }

    function restartListening() {
        // เปลี่ยนปุ่มกลับเป็นปุ่มหยุด
        if (stopSpeechButton) {
            stopSpeechButton.innerHTML = '<i class="fas fa-stop"></i>';
            stopSpeechButton.style.backgroundColor = 'rgba(239, 68, 68, 0.9)'; // สีแดง
            stopSpeechButton.title = 'หยุด';
            stopSpeechButton.dataset.mode = 'stop';
        }
        // เริ่มโหมด Wake Word ใหม่
        startWakeWordMode();
    }

    function stopCurrentAudio() {
        if (currentAudioSource) {
            try { currentAudioSource.stop(); } catch (e) { }
            currentAudioSource = null;
        }
        if (window.avatarAnimator) window.avatarAnimator.stopSpeaking();
    }

    // 🔊 Audio Queue System for Streaming
    let audioQueue = [];
    let isPlayingAudio = false;
    let isServerResponseComplete = false; // 🔴 Track Server EOS Status

    async function queueAudio(audioData) {
        // 🛑 Drop chunks if we are stopped (User pressed Stop)
        if (!conversationLoopActive && !isPlayingAudio) {
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
            console.error("Error playing chunk:", e);
        } finally {
            isPlayingAudio = false;
            // Process next chunk immediately
            processAudioQueue();
        }
    }

    // ฟังก์ชันจบการเล่นทั้งหมดและเปิดไมค์
    function handlePlaybackFinished() {
        console.log("[Stream] All done. Resuming mic.");

        // Debounce เล็กน้อยกันเสียงท้ายประโยคตีกับไมค์
        setTimeout(() => {
            if (conversationLoopActive && !musicHandler.isPlaying()) {
                startConversationLoop();
            }
            isServerResponseComplete = false; // Reset for next turn
        }, 800);
    }

    // 🔊 ฟังก์ชันเล่นเสียง Chunk เดียว (Internal)
    async function playAudioChunk(audioData) {
        if (!mainAudioContext) return;

        try {
            const buffer = await (audioData instanceof Blob ? audioData.arrayBuffer() : audioData);
            const audioBuffer = await mainAudioContext.decodeAudioData(buffer);
            const source = mainAudioContext.createBufferSource();
            source.buffer = audioBuffer;
            source.connect(mainAudioContext.destination);

            // 🟢 [Trigger] สั่งให้ Avatar เริ่มขยับปาก ถ้ายังไม่ได้ขยับ
            if (window.avatarAnimator && !window.avatarAnimator.isSpeaking) {
                window.avatarAnimator.startSpeaking();
                uiController.setEmotion('speaking');
            }

            return new Promise((resolve) => {
                source.onended = () => {
                    currentAudioSource = null;

                    // Check if queue is empty to stop animation
                    if (audioQueue.length === 0) {
                        // Stop speaking animation
                        if (window.avatarAnimator) window.avatarAnimator.stopSpeaking();

                        // 🔍 CRITICAL CHECK: Have we received the "END" signal from server?
                        if (isServerResponseComplete) {
                            handlePlaybackFinished();
                        } else {
                            console.log("[Stream] Queue empty, waiting for more chunks (or END signal)...");
                        }
                    }
                    resolve();
                };
                source.start(0);
                currentAudioSource = source;
            });
        } catch (e) {
            console.error("Audio Play Error:", e);
            // If error, just resolve to continue queue
            return Promise.resolve();
        }
    }

    // Legacy wrapper
    async function playAudio(audioData) {
        clearAudioQueue();
        queueAudio(audioData);
    }

    // 🔊 Initialize Audio System (Complete)
    async function initializeAudioSystem() {
        console.log("🔊 Initializing Audio System...");

        if (!mainAudioContext) {
            const AudioContext = window.AudioContext || window.webkitAudioContext;
            mainAudioContext = new AudioContext();
        }
        if (mainAudioContext.state === 'suspended') {
            await mainAudioContext.resume();
        }

        // Initialize Wake Word Handler
        if (!wakeWordHandler && typeof WakeWordHandler !== 'undefined') {
            wakeWordHandler = new WakeWordHandler({
                onStatusUpdate: (msg) => { },
                onWakeWordDetected: async () => {
                    console.log("🎤 Wake Word Detected!");
                    if (voiceHandler) voiceHandler.stop(true);
                    startConversationLoop();
                }
            });
        }

        // Initialize Voice Handler (Speech-to-Text)
        if (!voiceHandler && typeof VoiceHandler !== 'undefined') {
            voiceHandler = new VoiceHandler(mainAudioContext, {
                onStatusUpdate: (msg) => {
                    if (!currentAudioSource && conversationLoopActive) uiController.setStatus(msg);
                },
                onSpeechEnd: (audioBlob) => {
                    if (conversationLoopActive) {
                        uiController.setStatus("กำลังประมวลผล...");
                        uiController.setEmotion('thinking');
                        sendAudioToBackend(audioBlob);
                    }
                }
            });
        }

        // 🎵 Initialize Music Handler
        if (!musicHandler && typeof AvatarMusicHandler !== 'undefined') {
            const timerManager = {
                clearPresentationTimeout: () => { },
                startPresentationTimeout: () => { }
            };

            musicHandler = new AvatarMusicHandler(
                websocket,
                uiController,
                voiceHandler,
                timerManager,
                {
                    musicControls: document.getElementById('music-controls'),
                    stopAISpeechAudio: () => stopCurrentAudio(),
                    stopSpeechButton: document.getElementById('stop-speech-btn'),
                    resetToListeningState: () => restartListening()
                }
            );
            console.log("🎵 Avatar Music Handler Connected");
        }

        console.log("✅ Audio System Ready");
    }

    // 📤 Send Query - RESET Audio Manager
    function sendQuery(query, intent = null, additionalData = {}) {
        if (websocket && websocket.readyState === WebSocket.OPEN) {
            // 🧠 Reset Audio Manager on new Query
            if (window.audioStreamManager) audioStreamManager.reset();

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
                // 🔴 Handle EOS Signal
                if (data.action === "AUDIO_STREAM_END") {
                    console.log("[Stream] Server sent END signal.");
                    isServerResponseComplete = true;
                    return;
                }

                if (musicHandler && musicHandler.handleMessage(data)) {
                    conversationLoopActive = false;
                    return;
                }
                const hasVisual = data.image_url || data.image_gallery?.length || data.sources?.length || (data.action === 'SHOW_MAP_EMBED');
                const shouldEnterPresentation = hasVisual || (data.answer && data.answer.length > 0);

                if (shouldEnterPresentation) {
                    uiController.enterPresentation(data);
                } else if (data.answer) {
                    uiController.setStatus(data.answer);
                }

                uiController.setEmotion(data.emotion || 'talking');

            } else if (event.data instanceof ArrayBuffer) {
                // 🧠 Smart Audio Manager deals with chunks
                if (window.audioStreamManager) {
                    audioStreamManager.enqueue(event.data);
                }

                if (voiceHandler) voiceHandler.stop(false);
                if (wakeWordHandler) wakeWordHandler.resetSilenceTimer();
            }
        };

        websocket.onclose = () => setTimeout(connectWebSocket, 3000);
    }

    // Listeners
    if (stopSpeechButton) {
        stopSpeechButton.addEventListener('click', () => {
            if (stopSpeechButton.dataset.mode === 'start') {
                restartListening();
            } else {
                stopConversationLoop(true);
            }
        });
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
    const avatarNavBtn = document.getElementById('avatar-nav-btn');

    // Note: FabManager at the bottom of the file handles binding click events for these buttons.


    // Definitions
    const aiModeBtn = document.getElementById('ai-mode-toggle');

    // AI Mode Manager
    let aiModeManager = null;
    if (window.NanApp && window.NanApp.AIModeManager) {
        aiModeManager = new window.NanApp.AIModeManager();
        updateAIModeUI(aiModeManager.getMode());
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

                // Music: Custom Avatar Logic
                onMusicAction: () => {
                    const infoDisplay = document.getElementById('info-display');
                    if (infoDisplay) {
                        infoDisplay.innerHTML = '';
                        const fabManager = window.NanApp?.fabManager || new FabManager({
                            callbacks: { sendMessage: (t, i, d) => sendQuery(t, i, d) }
                        });
                        const widget = fabManager.createMusicWidget();
                        infoDisplay.appendChild(widget);

                        if (window.avatarAnimator) {
                            window.avatarAnimator.enterPresentationMode({ html_is_pre_rendered: true });
                        }
                        uiController.setEmotion('listening');
                        uiController.setStatus("🎵 ฟังเพลงอะไรดีคะ?");
                    }
                },

                // FAQ: Custom Avatar Logic
                onFaqAction: () => {
                    const infoDisplay = document.getElementById('info-display');
                    if (infoDisplay) {
                        infoDisplay.innerHTML = '';
                        const fabManager = window.NanApp?.fabManager || new FabManager({
                            callbacks: { sendMessage: (t, i, d) => sendQuery(t, i, d) }
                        });
                        const widget = fabManager.createFAQWidget();
                        infoDisplay.appendChild(widget);

                        if (window.avatarAnimator) {
                            window.avatarAnimator.enterPresentationMode({ html_is_pre_rendered: true });
                        }
                        uiController.setEmotion('listening');
                        uiController.setStatus("❓ คำถามที่พบบ่อย");
                    }
                },

                // Calc: Use default sendMessage (via callback)
                // FabManager calls sendMessage("...", INTENTS.CALCULATOR)

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
                },

                // Calc: Custom Avatar Logic using FabManager Widget
                onCalcAction: () => {
                    const resultText = document.getElementById('result-text');
                    const infoDisplay = document.getElementById('info-display');

                    if (resultText && infoDisplay) {
                        // Clear old content
                        resultText.innerHTML = '';
                        infoDisplay.innerHTML = '';

                        // Create Widget from FabManager
                        const fabManager = window.NanApp?.fabManager || new FabManager({
                            callbacks: { sendMessage: (t, i, d) => sendQuery(t, i, d) }
                        });

                        // Creating a fresh calculator widget instance 
                        const widget = fabManager.createCalculatorWidget();

                        // Append to info display
                        infoDisplay.appendChild(widget);

                        // Enter presentation mode
                        if (window.avatarAnimator) {
                            window.avatarAnimator.enterPresentationMode({ html_is_pre_rendered: true });
                        }
                        uiController.setEmotion('listening');
                        uiController.setStatus("🔢 เครื่องคิดเลขพร้อมใช้งานแล้วค่ะ");
                    }
                }
            }
        });
    }

    connectWebSocket();
});