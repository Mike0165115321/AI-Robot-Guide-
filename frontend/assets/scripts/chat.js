// /frontend/assets/scripts/chat.js (Improved Image & Content Rendering + Browser STT)

// 🚀 [เพิ่ม 1/3] สร้าง Class สำหรับไมค์เบราว์เซอร์ (เฉพาะกิจสำหรับไฟล์นี้)
class BrowserMicHandler {
    constructor(callbacks) {
        this.callbacks = callbacks;
        this.recognition = this.createRecognition();
        this.isListening = false;
        this.finalTranscript = '';
    }

    createRecognition() {
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        if (!SpeechRecognition) {
            this.callbacks.onError('เบราว์เซอร์นี้ไม่รองรับระบบเสียงพูดค่ะ (โปรดใช้ Chrome หรือ Edge)');
            return null;
        }

        const recognition = new SpeechRecognition();
        recognition.lang = 'th-TH'; // ภาษาไทย
        recognition.interimResults = true; // ขอผลลัพธ์ระหว่างพูด
        recognition.continuous = true; // พูดต่อได้เรื่อยๆ

        recognition.onstart = () => {
            this.isListening = true;
            this.finalTranscript = '';
            this.callbacks.onStartRecording();
        };

        recognition.onend = () => {
            this.isListening = false;
            this.callbacks.onStopRecording();
            // [สำคัญ] ส่งข้อความที่ได้ทั้งหมดกลับไป
            if (this.finalTranscript.trim()) {
                this.callbacks.onFinalTranscript(this.finalTranscript.trim());
            }
        };

        recognition.onerror = (event) => {
            if (event.error === 'no-speech') {
                // ไม่พูดอะไร ไม่ต้องทำอะไร
            } else {
                this.callbacks.onError(event.error);
            }
        };

        recognition.onresult = (event) => {
            let interimTranscript = '';
            for (let i = event.resultIndex; i < event.results.length; ++i) {
                if (event.results[i].isFinal) {
                    this.finalTranscript += event.results[i][0].transcript;
                } else {
                    interimTranscript += event.results[i][0].transcript;
                }
            }
            // อัปเดตข้อความระหว่างพูด (ถ้าต้องการ)
            this.callbacks.onInterimTranscript(this.finalTranscript + interimTranscript);
        };

        return recognition;
    }

    start() {
        if (this.isListening || !this.recognition) return;
        try {
            this.recognition.start();
        } catch (e) {
            console.error("Mic start error:", e);
        }
    }

    stop() {
        if (!this.isListening || !this.recognition) return;
        try {
            this.recognition.stop();
        } catch (e) {
            console.error("Mic stop error:", e);
        }
    }
}

document.addEventListener('DOMContentLoaded', () => {
    const messageArea = document.getElementById('message-area');
    const userInput = document.getElementById('user-input');
    const sendButton = document.getElementById('send-button-icon');
    const micButton = document.getElementById('mic-button');
    const newChatBtn = document.getElementById('new-chat-btn');
    const faqButton = document.getElementById('faq-button');

    let messageCounter = 0;

    // 🚀 [เพิ่ม 2/3] ประกาศตัวแปรสำหรับระบบเสียง (Browser)
    let audioContext = null;
    let browserMicHandler = null;
    let websocket;

    // Connect WebSocket for chat (only once)
    function connectChatWebSocket() {
        if (typeof API_HOST === 'undefined' || typeof API_PORT === 'undefined') {
            console.error("API_HOST or API_PORT is not defined in config.js");
            return;
        }

        if (websocket && websocket.readyState === WebSocket.OPEN) {
            websocket.close();
        }

        websocket = new WebSocket(`ws://${API_HOST}:${API_PORT}/api/chat/ws`);
        websocket.binaryType = 'arraybuffer';

        websocket.onopen = () => {
            console.log("Chat WS Connected.");
            sendSystemMessage("สวัสดีค่ะ น้องน่าน AI ยินดีให้บริการค่ะ มีอะไรให้น้องน่านช่วยแนะนำการท่องเที่ยว หรือข้อมูลวัฒนธรรมประเพณีของน่านไหมคะ? ว่ามาได้เลยเจ้า!");
        };

        websocket.onmessage = async (event) => {
            if (typeof event.data === 'string') {
                const data = JSON.parse(event.data);
                displayMessage(data.answer || "ขออภัยค่ะ ไม่เข้าใจคำถาม", 'ai', data.image_url, data.image_gallery, data.emotion, data.sources, data.action, data.action_payload);
            } else if (event.data instanceof ArrayBuffer) {
                await playAudio(event.data);
            }
        };

        websocket.onclose = (event) => {
            console.log("Chat WS Closed:", event);
            if (!event.wasClean) {
                setTimeout(connectChatWebSocket, 3000);
            }
        };

        websocket.onerror = (error) => {
            console.error("Chat WS Error:", error);
            websocket.close();
        };
    }

    // Play audio from AI response
    async function playAudio(audioData) {
        try {
            if (!audioContext) {
                audioContext = new (window.AudioContext || window.webkitAudioContext)();
            }
            if (audioContext.state === 'suspended') {
                await audioContext.resume();
            }
            const audioBuffer = await audioContext.decodeAudioData(audioData);
            const source = audioContext.createBufferSource();
            source.buffer = audioBuffer;
            source.connect(audioContext.destination);
            source.start(0);
        } catch (e) {
            console.error("Error playing audio:", e);
        }
    }

    // --- Message Display Logic ---
    function showMapEmbed(embedUrl, title) {
        const lastMessage = messageArea.lastElementChild;
        if (!lastMessage) return;

        const bubble = lastMessage.querySelector('.bubble');
        if (!bubble) return;

        const mapContainer = document.createElement('div');
        mapContainer.className = 'map-embed-container mt-4 rounded-lg overflow-hidden border border-glass-border';
        mapContainer.innerHTML = `
            <div class="bg-black/50 p-2 flex justify-between items-center">
                <span class="text-xs text-accent font-bold"><i class="fa-solid fa-map-location-dot mr-2"></i>${title}</span>
                <a href="${embedUrl}" target="_blank" class="text-xs text-primary hover:text-white transition"><i class="fa-solid fa-external-link-alt"></i> เปิดเต็มจอ</a>
            </div>
            <iframe src="${embedUrl}" width="100%" height="250" style="border:0;" allowfullscreen="" loading="lazy" referrerpolicy="no-referrer-when-downgrade"></iframe>
        `;

        bubble.appendChild(mapContainer);
        messageArea.scrollTop = messageArea.scrollHeight;
    }

    function displayMessage(text, sender, imageUrl = null, imageGallery = [], emotion = 'normal', sources = [], action = null, actionPayload = null, suggestedQuestions = []) {
        messageCounter++;
        const messageRow = document.createElement('div');
        messageRow.classList.add('message-row', sender);
        messageRow.id = `msg-${messageCounter}`;

        const bubble = document.createElement('div');
        bubble.classList.add('bubble', sender);

        let contentHtml = marked.parse(text);

        if (imageGallery && imageGallery.length > 0) {
            contentHtml += `<div class="image-gallery-grid">`;
            imageGallery.forEach(img => {
                contentHtml += `<img src="${img.url || img}" alt="${img.alt || 'รูปภาพประกอบ'}" class="responsive-image">`;
            });
            contentHtml += `</div>`;
        } else if (imageUrl) {
            contentHtml += `<div class="single-image-container"><img src="${imageUrl}" alt="รูปภาพประกอบ" class="responsive-image"></div>`;
        }

        if (sources && sources.length > 0) {
            contentHtml += `<div class="sources-container"><h4>แหล่งข้อมูล:</h4><ul>`;
            sources.forEach(source => {
                contentHtml += `<li><a href="${source.url}" target="_blank">${source.title || source.url}</a></li>`;
            });
            contentHtml += `</ul></div>`;
        }

        bubble.innerHTML = contentHtml;

        if (suggestedQuestions && suggestedQuestions.length > 0) {
            const questionsContainer = document.createElement('div');
            questionsContainer.className = 'suggested-questions-container';
            suggestedQuestions.forEach(q => {
                const btn = document.createElement('button');
                btn.className = 'suggestion-chip';
                btn.textContent = q;
                btn.onclick = () => sendMessage(q);
                questionsContainer.appendChild(btn);
            });
            bubble.appendChild(questionsContainer);
        }

        messageRow.appendChild(bubble);
        messageArea.appendChild(messageRow);
        messageArea.scrollTop = messageArea.scrollHeight;

        messageRow.querySelectorAll('img').forEach(img => {
            img.onload = () => { messageArea.scrollTop = messageArea.scrollHeight; };
        });

        if (action === 'SHOW_MAP_EMBED' && actionPayload && actionPayload.embed_url) {
            showMapEmbed(actionPayload.embed_url, actionPayload.destination_name || "แผนที่นำทาง");
        }
    }

    function sendSystemMessage(text) {
        displayMessage(text, 'system');
    }

    // --- User Input & Mic Control ---
    function sendMessage(text = null) {
        if (!text) text = userInput.value.trim();

        if (text && websocket && websocket.readyState === WebSocket.OPEN) {
            displayMessage(text, 'user');
            websocket.send(JSON.stringify({ query: text }));
            userInput.value = '';

            // Stop mic if listening
            if (browserMicHandler && browserMicHandler.isListening) {
                browserMicHandler.stop();
            }
        }
    }

    // 🚀 [แก้ไข 3/3] "รื้อ" micButton Event Listener เพื่อใช้ BrowserMicHandler
    micButton.addEventListener('click', async () => {
        // 1. สร้าง Context (จำเป็นต้องมี 1 ครั้ง)
        if (!audioContext) {
            try {
                audioContext = new (window.AudioContext || window.webkitAudioContext)();
                await audioContext.resume();
            } catch (e) {
                console.error("Could not create/resume AudioContext", e);
                alert('ขออภัยค่ะ ไม่สามารถเปิดใช้งานระบบเสียงได้');
                return;
            }
        }

        // 2. สร้าง Handler (ถ้ายังไม่มี)
        if (!browserMicHandler) {
            try {
                browserMicHandler = new BrowserMicHandler({
                    onStartRecording: () => {
                        micButton.classList.add('listening');
                        micButton.querySelector('i').className = 'fa-solid fa-microphone-lines';
                        userInput.setAttribute('placeholder', 'กำลังฟัง...');
                    },
                    onStopRecording: () => {
                        micButton.classList.remove('listening');
                        micButton.querySelector('i').className = 'fa-solid fa-microphone';
                        userInput.setAttribute('placeholder', 'ถามน้องน่านได้เลยเจ้า...');
                    },
                    onInterimTranscript: (text) => {
                        userInput.value = text; // อัปเดตข้อความระหว่างพูด
                    },
                    onFinalTranscript: (text) => {
                        // เมื่อพูดจบ ให้เติมข้อความ และ "ไม่ส่ง"
                        userInput.value = text;
                        userInput.focus(); // ย้ายเคอร์เซอร์ไปที่ช่องแชท
                    },
                    onError: (error) => {
                        console.error("Mic Error:", error);
                        alert(`เกิดข้อผิดพลาดกับระบบเสียง: ${error}`);
                    }
                });
            } catch (e) {
                console.error("Failed to initialize BrowserMicHandler", e);
                return;
            }
        }

        // 3. สั่งเริ่ม/หยุด การฟัง
        if (browserMicHandler.isListening) {
            browserMicHandler.stop();
        } else {
            browserMicHandler.start();
        }
    });

    // --- Event Listeners ---
    sendButton.addEventListener('click', () => sendMessage());
    userInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            sendMessage();
        }
    });

    newChatBtn.addEventListener('click', () => {
        if (confirm("คุณแน่ใจหรือไม่ที่ต้องการเริ่มการสนทนาใหม่?")) {
            messageArea.innerHTML = '';
            connectChatWebSocket();
            if (browserMicHandler && browserMicHandler.isListening) {
                browserMicHandler.stop();
            }
        }
    });

    // Handle FAQ button click
    faqButton.addEventListener('click', () => {
        const faqText = "### คำถามที่พบบ่อยเกี่ยวกับการท่องเที่ยวจังหวัดน่าน\n\nน้องน่าน AI สามารถช่วยคุณวางแผนการเดินทางและให้ข้อมูลสถานที่ท่องเที่ยวที่น่าสนใจในจังหวัดน่านได้ค่ะ ลองเลือกคำถามด้านล่างได้เลยนะคะ:";
        const questions = [
            "แนะนำที่เที่ยวน่านหน่อย?",
            "วัดสำคัญในน่านมีที่ไหนบ้าง?",
            "ประเพณีของคนน่านมีอะไรบ้าง?",
            "ร้านอาหารพื้นเมืองที่ห้ามพลาด?",
            "โรงแรมที่พักในเมืองน่าน?"
        ];
        displayMessage(faqText, 'system', null, [], 'normal', [], null, null, questions);
    });

    // --- Initialization ---
    connectChatWebSocket();
});