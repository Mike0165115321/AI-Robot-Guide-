// /frontend/assets/scripts/chat.js (Improved Image & Content Rendering)

document.addEventListener('DOMContentLoaded', () => {
    const messageArea = document.getElementById('message-area');
    const userInput = document.getElementById('user-input');
    const sendButton = document.getElementById('send-button-icon');
    const micButton = document.getElementById('mic-button');
    const newChatBtn = document.getElementById('new-chat-btn');
    const faqButton = document.getElementById('faq-button'); // เพิ่ม FAQ button
    const mapOverlay = document.getElementById('map-overlay'); // สำหรับ Travel Mode
    const closeMapBtn = document.getElementById('close-map-btn'); // สำหรับ Travel Mode

    let recognition;
    let isListening = false;
    let messageCounter = 0; // เพื่อให้แต่ละข้อความมี ID ไม่ซ้ำกัน

    // --- Voice Handler Integration ---
    let mainAudioContext = new (window.AudioContext || window.webkitAudioContext)();
    let websocket;
    let voiceHandler;

    // Connect WebSocket for chat (only once)
    function connectChatWebSocket() {
        if (typeof API_HOST === 'undefined' || typeof API_PORT === 'undefined') {
            console.error("API_HOST or API_PORT is not defined in config.js");
            return;
        }

        // ปิดการเชื่อมต่อเดิมถ้ามี
        if (websocket && websocket.readyState === WebSocket.OPEN) {
            websocket.close();
        }

        websocket = new WebSocket(`ws://${API_HOST}:${API_PORT}/api/chat/ws`);
        websocket.binaryType = 'arraybuffer'; // สำหรับรับข้อมูลเสียง

        websocket.onopen = () => {
            console.log("Chat WS Connected.");
            // ส่งข้อความต้อนรับครั้งแรกเมื่อเชื่อมต่อ
            sendSystemMessage("สวัสดีค่ะ น้องน่าน AI ยินดีให้บริการค่ะ มีอะไรให้น้องน่านช่วยแนะนำการท่องเที่ยว หรือข้อมูลวัฒนธรรมประเพณีของน่านไหมคะ? ว่ามาได้เลยเจ้า!");
        };

        websocket.onmessage = async (event) => {
            if (typeof event.data === 'string') {
                const data = JSON.parse(event.data);
                displayMessage(data.answer || "ขออภัยค่ะ ไม่เข้าใจคำถาม", 'ai', data.image_url, data.image_gallery, data.emotion, data.sources, data.action);
            } else if (event.data instanceof ArrayBuffer) {
                // สำหรับการเล่นเสียงตอบกลับของ AI
                await playAudio(event.data);
            }
        };

        websocket.onclose = (event) => {
            console.log("Chat WS Closed:", event);
            // พยายามเชื่อมต่อใหม่หากปิดโดยไม่ตั้งใจ
            if (!event.wasClean) {
                console.log("Attempting to reconnect Chat WS...");
                setTimeout(connectChatWebSocket, 3000);
            }
        };

        websocket.onerror = (error) => {
            console.error("Chat WS Error:", error);
            websocket.close();
        };
    }

    // Initialize VoiceHandler for microphone input
    function initializeVoiceHandler() {
        if (!voiceHandler) {
            voiceHandler = new VoiceHandler(mainAudioContext, {
                onStatusUpdate: (text) => {
                    if (isListening) updateMicButtonState(text);
                },
                onSpeechEnd: (audioBlob) => {
                    if (websocket && websocket.readyState === WebSocket.OPEN) {
                        websocket.send(audioBlob);
                        updateMicButtonState("กำลังประมวลผล...");
                    }
                    isListening = false;
                }
            });
        }
    }

    // Play audio from AI response
    async function playAudio(audioData) {
        try {
            const audioBuffer = await mainAudioContext.decodeAudioData(audioData);
            const source = mainAudioContext.createBufferSource();
            source.buffer = audioBuffer;
            source.connect(mainAudioContext.destination);
            source.start(0);
        } catch (e) {
            console.error("Error playing audio:", e);
        }
    }

    // --- Message Display Logic ---
    function displayMessage(text, sender, imageUrl = null, imageGallery = [], emotion = 'normal', sources = [], action = null) {
        messageCounter++;
        const messageRow = document.createElement('div');
        messageRow.classList.add('message-row', sender);
        messageRow.id = `msg-${messageCounter}`;

        const bubble = document.createElement('div');
        bubble.classList.add('bubble', sender);

        let contentHtml = marked.parse(text);

        // Handle image gallery
        if (imageGallery && imageGallery.length > 0) {
            contentHtml += `<div class="image-gallery-grid">`;
            imageGallery.forEach(img => {
                contentHtml += `<img src="${img.url || img}" alt="${img.alt || 'รูปภาพประกอบ'}" class="responsive-image">`;
            });
            contentHtml += `</div>`;
        }
        // Handle single image
        else if (imageUrl) {
            contentHtml += `<div class="single-image-container">
                                <img src="${imageUrl}" alt="รูปภาพประกอบ" class="responsive-image">
                            </div>`;
        }

        // Handle sources
        if (sources && sources.length > 0) {
            contentHtml += `<div class="sources-container"><h4>แหล่งข้อมูล:</h4><ul>`;
            sources.forEach(source => {
                contentHtml += `<li><a href="${source.url}" target="_blank">${source.title || source.url}</a></li>`;
            });
            contentHtml += `</ul></div>`;
        }

        bubble.innerHTML = contentHtml;
        messageRow.appendChild(bubble);
        messageArea.appendChild(messageRow);
        messageArea.scrollTop = messageArea.scrollHeight;

        // Auto-scroll on image load to prevent cut-off issues
        messageRow.querySelectorAll('img').forEach(img => {
            img.onload = () => {
                messageArea.scrollTop = messageArea.scrollHeight;
            };
        });

        // Handle specific actions (e.g., show map)
        if (action === 'SHOW_MAP_EMBED' && data.map_embed_url) {
            showMapEmbed(data.map_embed_url, data.map_title || "แผนที่นำทาง");
        }
    }

    function sendSystemMessage(text) {
        displayMessage(text, 'system');
    }

    // --- User Input & Mic Control ---
    function sendMessage() {
        const text = userInput.value.trim();
        if (text && websocket && websocket.readyState === WebSocket.OPEN) {
            displayMessage(text, 'user');
            websocket.send(JSON.stringify({ query: text }));
            userInput.value = '';
            // ถ้าไมค์กำลังฟัง ให้หยุด
            if (voiceHandler && voiceHandler.isListening) {
                voiceHandler.stop(true);
                isListening = false;
                updateMicButtonState("แตะเพื่อพูด");
            }
        }
    }

    function toggleMic() {
        if (isListening) {
            voiceHandler.stop(true);
            isListening = false;
            updateMicButtonState("แตะเพื่อพูด");
        } else {
            // ขออนุญาตใช้ไมค์ก่อน
            navigator.mediaDevices.getUserMedia({ audio: true })
                .then(stream => {
                    voiceHandler.start();
                    isListening = true;
                    updateMicButtonState("กำลังฟัง...");
                    stream.getTracks().forEach(track => track.stop()); // ปิด stream ทันทีหลังได้สิทธิ์
                })
                .catch(err => {
                    console.error("Error accessing microphone:", err);
                    alert("ไม่สามารถเข้าถึงไมโครโฟนได้ กรุณาอนุญาตการเข้าถึงไมโครโฟนในเบราว์เซอร์ของคุณ");
                    updateMicButtonState("ไมค์ไม่พร้อม");
                });
        }
    }

    function updateMicButtonState(status) {
        if (isListening) {
            micButton.classList.add('listening');
            micButton.querySelector('i').className = 'fa-solid fa-microphone-lines';
            micButton.setAttribute('title', status);
        } else {
            micButton.classList.remove('listening');
            micButton.querySelector('i').className = 'fa-solid fa-microphone';
            micButton.setAttribute('title', status);
        }
    }

    // --- Event Listeners ---
    sendButton.addEventListener('click', sendMessage);
    userInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            sendMessage();
        }
    });
    micButton.addEventListener('click', toggleMic);

    newChatBtn.addEventListener('click', () => {
        if (confirm("คุณแน่ใจหรือไม่ที่ต้องการเริ่มการสนทนาใหม่?")) {
            messageArea.innerHTML = '';
            connectChatWebSocket(); // Reconnect to get fresh welcome message
            if (voiceHandler && voiceHandler.isListening) {
                voiceHandler.stop(true);
                isListening = false;
                updateMicButtonState("แตะเพื่อพูด");
            }
        }
    });

    // Handle FAQ button click
    faqButton.addEventListener('click', () => {
        // Assume FAQ content is a predefined text or fetched from an API
        const faqContent = `
### คำถามที่พบบ่อยเกี่ยวกับการท่องเที่ยวจังหวัดน่าน

น้องน่าน AI สามารถช่วยคุณวางแผนการเดินทางและให้ข้อมูลสถานที่ท่องเที่ยวที่น่าสนใจในจังหวัดน่านได้ค่ะ

- **"แนะนำที่เที่ยวน่านหน่อย?"**
- **"วัดสำคัญในน่านมีที่ไหนบ้าง?"**
- **"ประเพณีของคนน่านมีอะไรบ้าง?"**
- **"ร้านอาหารพื้นเมืองที่ห้ามพลาด?"**
- **"โรงแรมที่พักในเมืองน่าน?"**

คุณสามารถพิมพ์คำถามเหล่านี้หรือคำถามอื่นๆ ที่เกี่ยวข้องได้เลยนะคะ
        `;
        displayMessage(faqContent, 'system');
    });

    // --- Initialization ---
    connectChatWebSocket();
    initializeVoiceHandler(); // Setup voice handler once
    updateMicButtonState("แตะเพื่อพูด"); // Initial state for mic button
});