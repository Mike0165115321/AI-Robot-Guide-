// /assets/scripts/avatar_logic.js (V-Music Mod 7.2 - Fixed Music Controls)

document.addEventListener('DOMContentLoaded', () => {

    const statusText = document.getElementById('status-text');
    const stopSpeechButton = document.getElementById('stop-speech-btn');
    
    const musicControls = document.getElementById('music-controls');
    const playPauseBtn = document.getElementById('play-pause-btn');
    const stopMusicBtn = document.getElementById('stop-btn'); 

    const uiController = {
        setEmotion(emotion) {
            if (window.avatarAnimator && typeof window.avatarAnimator.setEmotion === 'function') {
                window.avatarAnimator.setEmotion(emotion);
            }
        },
        setStatus(text) {
            if (statusText) statusText.textContent = text;
        },
        enterPresentation(data) {
            if (window.avatarAnimator && typeof window.avatarAnimator.enterPresentationMode === 'function') {
                window.avatarAnimator.enterPresentationMode(data);
            }
        },
        exitPresentation() {
            if (window.avatarAnimator && typeof window.avatarAnimator.exitPresentationMode === 'function') {
                window.avatarAnimator.exitPresentationMode();
            }
        }
    };

    let websocket = null;
    let mainAudioContext = null; 
    let currentAudioSource = null;
    let idleTimeout = null;
    
    let musicHandler = null; 
    let voiceHandler = null; 

    const IDLE_TIME_MS = 60000;
    const PRESENTATION_VIEW_TIME_MS = 20000;

    let isAITalking = false;
    let lastMessageWasIdle = false;
    let presentationHideTimeout = null;
    
    function stopAISpeechAudio() {
        if (currentAudioSource) {
            console.log("🛑 INTERRUPT: Stopping current AI speech audio.");
            currentAudioSource.onended = null; 
            currentAudioSource.stop();
            currentAudioSource = null;
        }
    }

    function interruptAISpeech() {
        stopAISpeechAudio(); 
        if (stopSpeechButton) stopSpeechButton.classList.remove('visible');
    }


    function resetToListeningOverPresentation() {
        interruptAISpeech();
        if (voiceHandler) voiceHandler.stop(true); 
        if (stopSpeechButton) stopSpeechButton.classList.remove('visible');

        console.log("State => Listening (over Presentation)");
        isAITalking = false;
        uiController.setEmotion('normal'); 
        uiController.setStatus("กำลังฟัง... (สามารถพูดแทรกได้)");

        if (mainAudioContext && mainAudioContext.state === 'running' && voiceHandler) { 
            voiceHandler.start();
        }
        timerManager.start();
    }
    
    function resetToListeningState() {
        interruptAISpeech();
        if (voiceHandler) voiceHandler.stop(true); 
        if (stopSpeechButton) stopSpeechButton.classList.remove('visible');
        if (musicControls) musicControls.style.display = 'none'; // 🚀 [แก้ไข 2/5] ซ่อนปุ่มเพลงเมื่อรีเซ็ต
        
        uiController.exitPresentation();

        if (musicHandler) musicHandler.reset(); 

        console.log("State => Listening");
        isAITalking = false;
        uiController.setEmotion('normal');
        uiController.setStatus("กำลังฟัง...");

        if (mainAudioContext && mainAudioContext.state === 'running' && voiceHandler) { 
            voiceHandler.start();
        }
        timerManager.start();
    }

    async function playAudio(audioData) {
        if (!mainAudioContext) return Promise.reject("AudioContext not ready");
        stopAISpeechAudio(); 

        try {
            let bufferToDecode = audioData;
            if (audioData instanceof Blob) {
                bufferToDecode = await audioData.arrayBuffer();
            } else if (!(audioData instanceof ArrayBuffer)){
                console.error("Invalid audio data type received:", typeof audioData);
                return Promise.reject("Invalid audio data type");
            }

            if (bufferToDecode.byteLength < 100) {
                console.warn("Received very small audio buffer, skipping playback.");
                return Promise.resolve();
            }

            const audioBuffer = await mainAudioContext.decodeAudioData(bufferToDecode);
            const newSource = mainAudioContext.createBufferSource();
            newSource.buffer = audioBuffer;
            newSource.connect(mainAudioContext.destination);

            if (stopSpeechButton) stopSpeechButton.classList.add('visible');

            return new Promise((resolve, reject) => {
                newSource.onended = () => {
                    if (window.avatarAnimator && typeof window.avatarAnimator.stopSpeaking === 'function') {
                        window.avatarAnimator.stopSpeaking();
                    }
                    if (currentAudioSource === newSource) currentAudioSource = null;
                    if (stopSpeechButton) stopSpeechButton.classList.remove('visible');
                    resolve();
                };
                try {
                    if (window.avatarAnimator && typeof window.avatarAnimator.startSpeaking === 'function') {
                        window.avatarAnimator.startSpeaking();
                    }
                    newSource.start(0);
                    currentAudioSource = newSource;
                } catch (startError){
                    console.error("Error starting audio source:", startError);
                    if (stopSpeechButton) stopSpeechButton.classList.remove('visible');
                    currentAudioSource = null;
                    reject(startError);
                }
            });

        } catch (e) {
            console.error("Audio decoding/playing error:", e);
            if (stopSpeechButton) stopSpeechButton.classList.remove('visible');
            currentAudioSource = null;
            return Promise.reject(e);
        }
    }

    const timerManager = {
        start: () => {
            if (isAITalking || (musicHandler && musicHandler.isPlaying()) || (musicHandler && musicHandler.isWaiting())) {
                console.log(`[Timer] Idle timer start skipped (isAITalking: ${isAITalking}, isPlayingMusic: ${musicHandler?.isPlaying()}, isAwaitingUserInput: ${musicHandler?.isWaiting()})`);
                return;
            }

            timerManager.clear();
            idleTimeout = setTimeout(() => {
                console.log("⏰ Idle timer triggered.");
                if (voiceHandler) voiceHandler.stop(true); 
                clearTimeout(presentationHideTimeout);
                uiController.exitPresentation();

                if (websocket?.readyState === WebSocket.OPEN) {
                    websocket.send(JSON.stringify({ action: "idle_prompt" }));
                    uiController.setEmotion('thinking');
                }
            }, IDLE_TIME_MS);
        },
        clear: () => {
            clearTimeout(idleTimeout);
            idleTimeout = null;
        },
        clearPresentationTimeout: () => {
            clearTimeout(presentationHideTimeout);
        }
    };


    async function initializeAndStart() {
        document.body.removeEventListener('click', initializeAndStart);
        uiController.setStatus("กำลังเริ่มต้นระบบเสียง...");

        try {
            if (!mainAudioContext) {
                mainAudioContext = new (window.AudioContext || window.webkitAudioContext)();
            }
            if (mainAudioContext.state === 'suspended') {
                await mainAudioContext.resume();
            }
            console.log("AudioContext is active and running.");

            if (!voiceHandler) {
                voiceHandler = new VoiceHandler(mainAudioContext, { 
                    onStatusUpdate: (text) => {
                        uiController.setStatus(text);
                        if (text === "กำลังฟัง..." || text === "รับฟังอยู่...") {
                            uiController.setEmotion('listening');
                        }
                    },
                    onSpeechEnd: (audioBlob) => {
                        timerManager.clear();
                        clearTimeout(presentationHideTimeout);
                        uiController.exitPresentation();
                        if (musicHandler) musicHandler.reset();

                        if (websocket?.readyState === WebSocket.OPEN) {
                            if (audioBlob && audioBlob.size > 1000) {
                                console.log(`User spoke, sending audio (${(audioBlob.size / 1024).toFixed(1)} KB).`);
                                uiController.setEmotion('thinking');
                                uiController.setStatus("กำลังประมวลผล...");
                                websocket.send(audioBlob);
                            } else {
                                console.log("Audio blob too small or invalid, not sending.");
                                resetToListeningState();
                            }
                        } else {
                            console.error("WebSocket not open, cannot send audio.");
                            resetToListeningState();
                        }
                    }
                });
            }
            
            if (!musicHandler) {
                // 🚀 [แก้ไข 3/5] ส่งปุ่ม musicControls เข้าไปเป็น callback
                musicHandler = new AvatarMusicHandler(websocket, uiController, voiceHandler, timerManager, {
                    stopSpeechButton: stopSpeechButton,
                    musicControls: musicControls, // 👈 ส่งปุ่มที่ถูกต้องเข้าไป
                    stopAISpeechAudio: stopAISpeechAudio,
                    resetToListeningState: resetToListeningState
                });
            }

            resetToListeningState();

        } catch (e) {
            console.error("Failed to initialize or resume AudioContext.", e);
            uiController.setStatus("ข้อผิดพลาดระบบเสียง: โปรดรีเฟรชหรืออนุญาตไมโครโฟน");
        }
    }


    function connectWebSocket() {
        // ... (โค้ดส่วนนี้เหมือนเดิม) ...
        if (typeof API_HOST === 'undefined' || typeof API_PORT === 'undefined') {
            console.error("API configuration (API_HOST/API_PORT in config.js) is missing!");
            uiController.setStatus("ข้อผิดพลาด: การตั้งค่า API หายไป");
            return;
        }
        const wsUrl = `ws://${API_HOST}:${API_PORT}/api/avatar/ws`;
        if (websocket && (websocket.readyState === WebSocket.OPEN || websocket.readyState === WebSocket.CONNECTING)) {
            return;
        }

        console.log(`Attempting to connect to WebSocket: ${wsUrl}`);
        uiController.setStatus("กำลังเชื่อมต่อ...");
        websocket = new WebSocket(wsUrl);
        websocket.binaryType = 'arraybuffer';

        websocket.onopen = () => {
            console.log("WebSocket connected successfully.");
            uiController.setStatus("โปรดคลิกเพื่อเริ่มการสนทนา");
            document.body.addEventListener('click', initializeAndStart, { once: true });
        };

        websocket.onmessage = async (event) => {
            // ... (โค้ดส่วนนี้เหมือนเดิม) ...
            timerManager.clear();

            if (typeof event.data === 'string') {
                try {
                    const data = JSON.parse(event.data);
                    console.log("Received JSON:", data);

                    isAITalking = true;
                    if (voiceHandler) voiceHandler.stop(true); 
                    
                    lastMessageWasIdle = data.isIdlePrompt || false;
                    if (lastMessageWasIdle) {
                        uiController.setEmotion(data.emotion || 'talking');
                        return; 
                    }
                    
                    if (musicHandler && musicHandler.handleMessage(data)) {
                        return; 
                    }
                    const hasVisualContent = data.image_url || (data.image_gallery && data.image_gallery.length > 0) || (data.sources && data.sources.length > 0);
                    
                    if (hasVisualContent) {
                        console.log("Message has visual content, entering presentation mode.");
                        uiController.setStatus("กำลังประมวลผล..."); 
                        uiController.enterPresentation(data);
                    } else {
                        console.log("Message has no visual content, will show text in status.");
                        if (data.answer) {
                            uiController.setStatus(data.answer); 
                        }
                        uiController.setEmotion(data.emotion || 'talking');
                    }

                } catch (e) {
                    console.error("Error processing received JSON:", e, "Data:", event.data);
                    resetToListeningState();
                }
            }
            else if (event.data instanceof ArrayBuffer) {
                console.log(`Received Audio Buffer (${event.data.byteLength} bytes)`);
                try {
                    await playAudio(event.data);
                    console.log("Audio playback finished.");

                    if (musicHandler && musicHandler.isWaiting()) {
                        console.log("[Input Wait] AI speech finished, but awaiting user input. Not resetting state.");
                        resetToListeningOverPresentation();
                        return; 
                    }

                    if (musicHandler && musicHandler.isPlaying()) { 
                        console.log("[Music] Audio finished, but music is playing. Keeping presentation open.");
                        musicHandler.goToMusicIdleState();
                    } else { 
                        clearTimeout(presentationHideTimeout);
                        presentationHideTimeout = setTimeout(() => {
                            console.log("Presentation view time expired, exiting presentation.");
                            uiController.exitPresentation();
                        }, PRESENTATION_VIEW_TIME_MS);
                        
                        resetToListeningState();
                    }

                } catch (e) {
                    console.error("Failed to play received audio.", e);
                    resetToListeningState();
                }
            } else {
                console.warn("Received unexpected data type:", typeof event.data);
            }
        }; 

        websocket.onclose = (event) => {
            // ... (โค้ดส่วนนี้เหมือนเดิม) ...
            console.warn(`WebSocket disconnected. Code: ${event.code}, Reason: ${event.reason}. Reconnecting in 5s...`);
            isAITalking = false;
            if (musicHandler) musicHandler.reset(); 
            timerManager.clear();
            clearTimeout(presentationHideTimeout);
            uiController.setStatus("การเชื่อมต่อถูกตัด กำลังพยายามเชื่อมต่อใหม่...");
            if (voiceHandler) voiceHandler.stop(true); 
            interruptAISpeech();
            document.body.removeEventListener('click', initializeAndStart);
            websocket = null;
            
            voiceHandler = null;
            musicHandler = null;
            mainAudioContext = null; 
            
            setTimeout(connectWebSocket, 5000);
        };
        websocket.onerror = (error) => {
            console.error("WebSocket Error:", error);
            uiController.setStatus("เกิดข้อผิดพลาดในการเชื่อมต่อ");
        };
    }

    const textQueryForm = document.getElementById('text-query-form');
    const textQueryInput = document.getElementById('text-query-input');
    if (textQueryForm && textQueryInput) {
        textQueryForm.addEventListener('submit', (e) => {
            e.preventDefault();
            const queryText = textQueryInput.value.trim();
            if (!queryText) return;

            if (websocket && websocket.readyState === WebSocket.OPEN) {
                console.log(`⌨️ [Text Input] Sending query: ${queryText}`);
                
                if (musicHandler) musicHandler.reset();
                
                interruptAISpeech();
                if (voiceHandler) voiceHandler.stop(true); 

                uiController.exitPresentation();
                websocket.send(JSON.stringify({ "query": queryText }));
                textQueryInput.value = "";
                uiController.setEmotion('thinking');
                uiController.setStatus("กำลังประมวลผล...");

            } else {
                console.error("WebSocket is not connected. Cannot send text query.");
                uiController.setStatus("ไม่ได้เชื่อมต่อ ไม่สามารถส่งข้อความได้");
            }
        });
    }

    if (stopSpeechButton) {
        stopSpeechButton.addEventListener('click', () => {
            console.log("🔘 [Stop Button] Clicked. Stopping AI speech and returning to listen state.");
            if (musicHandler) musicHandler.reset();  
            resetToListeningState();
        });
    }

    if (playPauseBtn) {
        playPauseBtn.addEventListener('click', () => {
            alert("ฟังก์ชัน Play/Pause ยังไม่รองรับค่ะ (กรุณากดปุ่ม Stop เพื่อหยุดเพลง)");
        });
    }
    
    if (stopMusicBtn) {
        stopMusicBtn.addEventListener('click', () => {
            console.log("🔘 [Music Stop Button] Clicked. Stopping music and returning to listen state.");
            if (musicHandler) musicHandler.reset();  
            resetToListeningState(); 
        });
    }


    connectWebSocket();
});