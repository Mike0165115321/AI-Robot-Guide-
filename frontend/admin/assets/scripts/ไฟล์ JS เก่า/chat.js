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

// ========================================
// Toast Notification Manager
// - Non-blocking notifications
// - Expands to form on click
// - Reusable for events/promotions
// ========================================
class ToastManager {
    constructor() {
        this.container = null; // Lazy init after DOM ready
        this.messageCount = 0;
        this.triggerAfterMessages = 3; // แสดง toast หลังข้อความที่ N
        this.hasShownWelcome = localStorage.getItem('nan_welcome_shown') === 'true';
        this.provinceSubmitted = localStorage.getItem('nan_province_submitted') === 'true';
    }

    // ดึง container (lazy init)
    getContainer() {
        if (!this.container) {
            this.container = document.getElementById('toast-container');
        }
        return this.container;
    }

    // เรียกเมื่อมีข้อความใหม่ (AI ตอบกลับ)
    onNewMessage() {
        this.messageCount++;
        console.log(`🔔 ToastManager: Message #${this.messageCount}`);

        // แสดง welcome toast ทุกๆ N ข้อความ (ไม่ต้องเช็ค localStorage)
        if (this.messageCount % this.triggerAfterMessages === 0) {
            // ตรวจสอบว่ามี toast อยู่แล้วหรือไม่ - ถ้ามีไม่ต้องแสดงซ้ำ
            if (!document.getElementById('welcome-toast')) {
                console.log('🎉 ToastManager: Showing welcome toast!');
                this.showWelcomeToast();
            }
        }
    }

    showWelcomeToast() {
        // ตรวจสอบว่ามี toast อยู่แล้วหรือไม่
        if (document.getElementById('welcome-toast')) return;

        const template = document.getElementById('welcome-toast-template');
        if (!template) return;

        const clone = template.content.cloneNode(true);
        const container = this.getContainer();
        if (!container) return;
        container.appendChild(clone);

        localStorage.setItem('nan_welcome_shown', 'true');

        // Setup province select change handler
        const select = document.getElementById('toast-province-select');
        const customGroup = document.getElementById('toast-custom-input-group');

        if (select) {
            select.addEventListener('change', () => {
                if (select.value === 'other_thai' || select.value === 'foreign') {
                    customGroup.style.display = 'block';
                } else {
                    customGroup.style.display = 'none';
                }
            });
        }
    }

    expandToast(element) {
        if (!element.classList.contains('expanded')) {
            element.classList.add('expanded');
        }
    }

    closeToast(toastId) {
        const toast = document.getElementById(toastId);
        if (toast) {
            toast.classList.add('closing');
            setTimeout(() => toast.remove(), 300);
        }
    }

    skipWelcome() {
        localStorage.setItem('nan_province_submitted', 'true');
        this.provinceSubmitted = true;
        this.closeToast('welcome-toast');
    }

    async submitWelcome() {
        const select = document.getElementById('toast-province-select');
        const customInput = document.getElementById('toast-custom-input');

        let province = select?.value || '';
        let origin = 'Thailand';

        // ถ้าเลือก "อื่นๆ" หรือ "ต่างประเทศ" ให้ใช้ค่าจาก input
        if (province === 'other_thai' || province === 'foreign') {
            province = customInput?.value?.trim() || '';
            if (province === 'foreign' && customInput?.value) {
                origin = customInput.value.trim();
                province = null;
            }
        }

        if (!province && !origin) {
            this.skipWelcome();
            return;
        }

        try {
            // ส่งข้อมูลไป backend
            await fetch(`${API_BASE_URL}/api/chat/welcome-data`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    session_id: sessionStorage.getItem('session_id') || 'anonymous',
                    user_province: province,
                    user_origin: origin
                })
            });

            console.log('✅ Province data submitted:', province || origin);
        } catch (error) {
            console.error('Failed to submit province data:', error);
        }

        localStorage.setItem('nan_province_submitted', 'true');
        this.provinceSubmitted = true;
        this.closeToast('welcome-toast');
    }

    // สำหรับ event toast ในอนาคต
    showEventToast(title, subtitle, icon = '🎉') {
        const html = `
            <div class="toast-notification" onclick="window.ToastManager.expandToast(this)">
                <button class="toast-close" onclick="event.stopPropagation(); this.parentElement.remove()">
                    <i class="fa-solid fa-xmark"></i>
                </button>
                <div class="toast-header">
                    <span class="toast-icon">${icon}</span>
                    <div>
                        <div class="toast-title">${title}</div>
                        <div class="toast-subtitle clickable-hint">${subtitle}</div>
                    </div>
                </div>
            </div>
        `;
        const container = this.getContainer();
        if (container) container.insertAdjacentHTML('beforeend', html);
    }
}

// สร้างและ expose ไว้ global สำหรับ HTML onclick handlers
window.ToastManager = new ToastManager();

// ========================================
// AI Mode Manager - UI Extension
// ใช้ NanApp.getAIModeManager() จาก main.js เป็นหลัก
// ========================================
function initAIModeUI() {
    const modeManager = window.NanApp ? window.NanApp.getAIModeManager() : null;
    if (!modeManager) {
        console.warn('⚠️ NanApp.AIModeManager not found');
        return;
    }

    const button = document.getElementById('ai-mode-btn');
    const indicator = document.getElementById('mode-indicator');

    if (!button) return;

    // Initial UI update
    updateAIModeUI(modeManager.getMode());

    // Click handler
    button.addEventListener('click', () => {
        const newMode = modeManager.toggle();
        updateAIModeUI(newMode);
        showModeIndicator(newMode);
    });

    function updateAIModeUI(mode) {
        // Update button
        button.classList.remove('fast-mode', 'detailed-mode');
        button.classList.add(`${mode}-mode`);

        // Update icon
        const icon = button.querySelector('i');
        if (icon) {
            icon.className = mode === 'fast'
                ? 'fa-solid fa-bolt'
                : 'fa-solid fa-brain';
        }

        // Update indicator
        if (indicator) {
            indicator.classList.remove('fast', 'detailed');
            indicator.classList.add(mode);
            indicator.textContent = mode === 'fast'
                ? '⚡ คิดเร็ว (Llama)'
                : '🧠 คิดละเอียด (Gemini)';
        }
    }

    function showModeIndicator(mode) {
        if (!indicator) return;
        indicator.classList.add('show');
        setTimeout(() => indicator.classList.remove('show'), 2000);
    }
}

document.addEventListener('DOMContentLoaded', () => {
    const messageArea = document.getElementById('message-area');
    const userInput = document.getElementById('user-input');
    const sendButton = document.getElementById('send-button-icon');
    const micButton = document.getElementById('mic-button');
    const musicButton = document.getElementById('music-button');
    const newChatBtn = document.getElementById('new-chat-btn');
    const faqButton = document.getElementById('faq-button');

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

    // Init AI Mode Manager - ใช้ function จากส่วนบน
    initAIModeUI();

    let messageCounter = 0;

    // 🚀 [เพิ่ม 2/3] ประกาศตัวแปรสำหรับระบบเสียง (Browser)
    let audioContext = null;
    let browserMicHandler = null;
    let websocket;
    let reconnectAttempts = 0;
    const maxReconnectDelay = 30000; // ไม่เกิน 30 วินาที

    // Connect WebSocket for chat (only once)
    function connectChatWebSocket() {
        if (typeof API_HOST === 'undefined' || typeof API_PORT === 'undefined') {
            console.error("API_HOST or API_PORT is not defined in config.js");
            return;
        }

        if (websocket && websocket.readyState === WebSocket.OPEN) {
            websocket.close();
        }

        // ใช้ WS_BASE_URL ถ้ามี, ไม่งั้นใช้ API_HOST/PORT
        const wsUrl = typeof WS_BASE_URL !== 'undefined'
            ? `${WS_BASE_URL}/api/chat/ws`
            : `ws://${API_HOST}:${API_PORT}/api/chat/ws`;

        websocket = new WebSocket(wsUrl);
        websocket.binaryType = 'arraybuffer';

        websocket.onopen = () => {
            console.log("Chat WS Connected.");
            reconnectAttempts = 0; // Reset เมื่อเชื่อมต่อสำเร็จ
            sendSystemMessage("สวัสดีค่ะ น้องน่าน AI ยินดีให้บริการค่ะ มีอะไรให้น้องน่านช่วยแนะนำการท่องเที่ยว หรือข้อมูลวัฒนธรรมประเพณีของน่านไหมคะ? ว่ามาได้เลยเจ้า!");
        };

        websocket.onmessage = async (event) => {
            if (typeof event.data === 'string') {
                const data = JSON.parse(event.data);

                // 🎭 ซ่อน Thinking Indicator เมื่อได้รับ response
                if (window.ThinkingIndicator) {
                    window.ThinkingIndicator.hide();
                }

                // 5. Display AI Message
                displayMessage(data.answer || "ขออภัยค่ะ ไม่เข้าใจคำถาม", 'ai', data.action, data.action_payload, data.image_url, data.image_gallery, messageCounter, data.processing_time, data.suggested_questions);

                // 6. Handle Auto-Audio
                // This part of the instruction seems to be from a different context or intended for a different function.
                // The current `websocket.onmessage` already handles audio via `event.data instanceof ArrayBuffer`.
                // If `data.audio_url` is meant for a separate audio playback, it would need a new implementation.
                // For now, I'm keeping the existing audio handling and only modifying the displayMessage call.

                // 🔔 Trigger toast notification after AI responds
                if (window.ToastManager) {
                    window.ToastManager.onNewMessage();
                }
            } else if (event.data instanceof ArrayBuffer) {
                await playAudio(event.data);
            }
        };

        websocket.onclose = (event) => {
            console.log("Chat WS Closed:", event);
            if (!event.wasClean) {
                // Exponential backoff: 1s, 2s, 4s, 8s... สูงสุด 30s
                const delay = Math.min(1000 * Math.pow(2, reconnectAttempts), maxReconnectDelay);
                reconnectAttempts++;
                console.log(`Reconnecting in ${delay / 1000}s... (attempt ${reconnectAttempts})`);
                setTimeout(connectChatWebSocket, delay);
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
    function showMapEmbed(embedUrl, title, externalLink = null) {
        const lastMessage = messageArea.lastElementChild;
        if (!lastMessage) return;

        const bubble = lastMessage.querySelector('.bubble');
        if (!bubble) return;

        const mapContainer = document.createElement('div');
        mapContainer.className = 'map-embed-container mt-4 rounded-lg overflow-hidden border border-glass-border';

        // Build navigation link button if available
        const navButtonHtml = externalLink ? `
            <a href="${externalLink}" target="_blank" rel="noopener" 
               class="nav-btn" style="
                display: inline-flex;
                align-items: center;
                gap: 8px;
                padding: 12px 24px;
                background: linear-gradient(135deg, #3b82f6, #2563eb);
                border-radius: 8px;
                color: white;
                text-decoration: none;
                font-weight: bold;
                font-size: 0.95rem;
                box-shadow: 0 4px 15px rgba(59, 130, 246, 0.4);
                margin-top: 15px;
            ">
                <i class="fa-solid fa-route"></i> เริ่มนำทาง
            </a>
        ` : '';

        mapContainer.innerHTML = `
            <div class="bg-black/50 p-2 flex justify-between items-center">
                <span class="text-xs text-accent font-bold"><i class="fa-solid fa-map-location-dot mr-2"></i>${title}</span>
                <a href="${embedUrl}" target="_blank" class="text-xs text-primary hover:text-white transition"><i class="fa-solid fa-expand"></i> ขยาย</a>
            </div>
            <iframe src="${embedUrl}" width="100%" height="250" style="border:0;" allowfullscreen="" loading="lazy" referrerpolicy="no-referrer-when-downgrade"></iframe>
            <div style="padding: 10px; text-align: center;">
                ${navButtonHtml}
            </div>
        `;

        bubble.appendChild(mapContainer);
        messageArea.scrollTop = messageArea.scrollHeight;
    }

    function displayMessage(text, sender, action = null, actionPayload = null, imageUrl = null, imageGallery = null, messageId = null, processingTime = null, suggestedQuestions = []) {
        const messageArea = document.getElementById('message-area');
        if (!messageArea) return;
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

        if (sender === 'ai') {
            // Create wrapper for AI message (Icon + Bubble)
            const wrapper = document.createElement('div');
            wrapper.style.display = 'flex';
            wrapper.style.alignItems = 'flex-start';
            wrapper.style.gap = '10px';
            wrapper.style.maxWidth = '100%';

            // Create Robot Icon
            const iconContainer = document.createElement('div');
            iconContainer.className = 'robot-avatar-icon';
            iconContainer.style.width = '40px';
            iconContainer.style.height = '40px';
            iconContainer.style.flexShrink = '0';
            iconContainer.style.transform = 'scale(0.8)'; // Scale down slightly
            iconContainer.style.transformOrigin = 'top left';

            iconContainer.innerHTML = `
                <div class="head-accent"></div>
                <div class="face-plate" style="width: 28px; height: 18px;">
                    <div class="eye left" style="width: 4px; height: 6px;"></div>
                    <div class="eye right" style="width: 4px; height: 6px;"></div>
                </div>
            `;

            wrapper.appendChild(iconContainer);

            // Container for Bubble + Actions
            const bubbleContainer = document.createElement('div');
            bubbleContainer.style.display = 'flex';
            bubbleContainer.style.flexDirection = 'column';
            bubbleContainer.style.gap = '5px';
            bubbleContainer.style.maxWidth = '80%';

            // Like/Dislike Buttons
            const feedbackContainer = document.createElement('div');
            feedbackContainer.className = 'feedback-actions';
            feedbackContainer.style.display = 'flex';
            feedbackContainer.style.gap = '8px';
            feedbackContainer.style.marginTop = '4px';
            feedbackContainer.style.alignItems = 'center'; // Align items vertically

            // ⏱️ Processing Time Indicator
            if (typeof processingTime !== 'undefined' && processingTime !== null) {
                const timeLabel = document.createElement('span');
                timeLabel.className = 'processing-time';
                timeLabel.style.fontSize = '0.75rem';
                timeLabel.style.color = 'rgba(255, 255, 255, 0.5)';
                timeLabel.style.marginLeft = '10px';
                timeLabel.innerHTML = `<i class="fa-solid fa-stopwatch"></i> AI Time: ${processingTime}s`;
                timeLabel.title = "Server Processing Time (Network latency excluded)";
                feedbackContainer.appendChild(timeLabel);
            }

            const userQuery = document.querySelector('.message-row.user:last-child .bubble')?.innerText || "";

            // Like Button
            const likeBtn = document.createElement('button');
            likeBtn.className = 'feedback-btn like-btn';
            likeBtn.innerHTML = '<i class="fa-regular fa-thumbs-up"></i>';
            likeBtn.onclick = () => submitFeedback('like', messageCounter, userQuery, text, likeBtn, dislikeBtn);

            // Dislike Button
            const dislikeBtn = document.createElement('button');
            dislikeBtn.className = 'feedback-btn dislike-btn';
            dislikeBtn.innerHTML = '<i class="fa-regular fa-thumbs-down"></i>';
            dislikeBtn.onclick = () => submitFeedback('dislike', messageCounter, userQuery, text, likeBtn, dislikeBtn);

            feedbackContainer.appendChild(likeBtn);
            feedbackContainer.appendChild(dislikeBtn);

            // Append Feedback container to Bubble Container
            bubbleContainer.appendChild(bubble);
            bubbleContainer.appendChild(feedbackContainer);

            // Print & Copy Actions
            const interactiveActions = ['SHOW_MAP_EMBED', 'SHOW_SONG_CHOICES', 'PROMPT_FOR_SONG_INPUT'];
            if (!interactiveActions.includes(action)) {

                const actionsBar = document.createElement('div');
                actionsBar.style.display = 'flex';
                actionsBar.style.justifyContent = 'flex-end';
                actionsBar.style.gap = '5px';
                actionsBar.style.marginTop = '5px';

                const printBtn = document.createElement('button');
                printBtn.className = 'btn-icon';
                printBtn.innerHTML = '<i class="fa-solid fa-print"></i>';
                printBtn.onclick = () => printMessage(text, imageUrl, imageGallery);

                const copyBtn = document.createElement('button');
                copyBtn.className = 'btn-icon';
                copyBtn.innerHTML = '<i class="fa-regular fa-copy"></i>';
                copyBtn.onclick = () => {
                    navigator.clipboard.writeText(text).then(() => {
                        copyBtn.innerHTML = '<i class="fa-solid fa-check"></i>';
                        setTimeout(() => copyBtn.innerHTML = '<i class="fa-regular fa-copy"></i>', 2000);
                    });
                };

                actionsBar.appendChild(copyBtn);
                actionsBar.appendChild(printBtn);
                bubbleContainer.appendChild(actionsBar); // Footer Actions
            }

            wrapper.appendChild(bubbleContainer);
            messageRow.appendChild(wrapper);
        } else {
            messageRow.appendChild(bubble);
        }

        messageArea.appendChild(messageRow);
        messageArea.scrollTop = messageArea.scrollHeight;

        messageRow.querySelectorAll('img').forEach(img => {
            img.onload = () => { messageArea.scrollTop = messageArea.scrollHeight; };
        });

        if (action === 'SHOW_MAP_EMBED' && actionPayload && actionPayload.embed_url) {
            showMapEmbed(actionPayload.embed_url, actionPayload.destination_name || "แผนที่นำทาง", actionPayload.external_link);
        } else if (action === 'SHOW_SONG_CHOICES' && actionPayload) {
            showSongChoices(actionPayload);
        }
    }

    function showSongInput(placeholder) {
        const lastMessage = messageArea.lastElementChild;
        if (!lastMessage) return;

        const bubble = lastMessage.querySelector('.bubble');
        if (!bubble) return;

        const inputContainer = document.createElement('div');
        inputContainer.className = 'song-input-container mt-3 flex gap-2';

        const input = document.createElement('input');
        input.type = 'text';
        input.className = 'flex-1 bg-black/30 border border-glass-border rounded-lg px-3 py-2 text-sm text-white focus:outline-none focus:border-accent transition';
        input.placeholder = placeholder;

        const searchBtn = document.createElement('button');
        searchBtn.className = 'bg-accent/20 hover:bg-accent/40 text-accent border border-accent/50 rounded-lg px-3 transition';
        searchBtn.innerHTML = '<i class="fa-solid fa-search"></i>';

        const submitSong = () => {
            const songName = input.value.trim();
            if (songName) {
                sendMessage(songName); // Send as a normal message
                inputContainer.remove(); // Remove input after sending
            }
        };

        searchBtn.onclick = submitSong;
        input.onkeypress = (e) => {
            if (e.key === 'Enter') submitSong();
        };

        inputContainer.appendChild(input);
        inputContainer.appendChild(searchBtn);
        bubble.appendChild(inputContainer);

        // Auto-focus the input
        setTimeout(() => input.focus(), 100);
        messageArea.scrollTop = messageArea.scrollHeight;
    }

    // 🆕 Helper to display music results and player
    function displayMusicResults(container, queryName, songs) {
        // Remove old results
        const oldResults = container.querySelector('.music-results-container');
        if (oldResults) oldResults.remove();

        const resultsHTML = `
            <div class="music-results-container mt-4">
                <p class="text-white/80 text-sm mb-2">🎵 ผลการค้นหา "<strong>${queryName}</strong>":</p>
                <div class="song-choices grid gap-2">
                    ${songs.map(song => `
                        <div class="song-card bg-black/30 border border-glass-border rounded-lg p-3 cursor-pointer hover:bg-black/50 transition flex items-center gap-3" 
                                data-url="${song.url}" data-title="${song.title}">
                            <img src="${song.thumbnail}" alt="" class="w-16 h-12 rounded object-cover">
                            <div class="flex-1 min-w-0">
                                <p class="text-white text-sm font-medium truncate">${song.title}</p>
                                <p class="text-white/50 text-xs">${song.channel || ''}</p>
                            </div>
                            <i class="fa-solid fa-play text-accent"></i>
                        </div>
                    `).join('')}
                </div>
            </div>
        `;
        container.insertAdjacentHTML('beforeend', resultsHTML);

        // Add click handlers
        container.querySelectorAll('.song-card').forEach(card => {
            card.addEventListener('click', () => {
                const url = card.dataset.url;
                const title = card.dataset.title;
                const videoId = url.includes('v=') ? url.split('v=')[1].split('&')[0] : url.split('/').pop();

                // Remove old player
                const oldPlayer = container.querySelector('.youtube-player-container');
                if (oldPlayer) oldPlayer.remove();

                // Highlight card
                container.querySelectorAll('.song-card').forEach(c => c.classList.remove('ring-2', 'ring-accent'));
                card.classList.add('ring-2', 'ring-accent');

                // Create player
                const playerHTML = `
                    <div class="youtube-player-container mt-4 rounded-lg overflow-hidden">
                        <div class="relative w-full" style="padding-bottom: 56.25%;">
                            <iframe 
                                class="absolute inset-0 w-full h-full"
                                src="https://www.youtube.com/embed/${videoId}?autoplay=1&rel=0" 
                                frameborder="0" 
                                allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" 
                                allowfullscreen>
                            </iframe>
                        </div>
                        <div class="bg-black/50 p-2 flex items-center justify-between">
                            <span class="text-white text-sm truncate flex-1">🎵 กำลังเล่น: ${title}</span>
                            <button class="audio-only-btn bg-accent/20 hover:bg-accent/40 text-accent text-xs px-2 py-1 rounded border border-accent/50 transition whitespace-nowrap ml-2" 
                                    data-url="${url}" data-title="${title}">
                                <i class="fa-solid fa-headphones"></i> ฟังแค่เสียง
                            </button>
                        </div>
                    </div>
                `;
                container.insertAdjacentHTML('beforeend', playerHTML);

                // Audio button handler
                const audioBtn = container.querySelector('.audio-only-btn');
                if (audioBtn) {
                    audioBtn.addEventListener('click', () => handleAudioStream(audioBtn, container));
                }

                // Scroll to player
                const player = container.querySelector('.youtube-player-container');
                if (player) player.scrollIntoView({ behavior: 'smooth', block: 'center' });
            });
        });
    }

    async function handleAudioStream(btn, container) {
        const videoUrl = btn.dataset.url;
        const videoTitle = btn.dataset.title;

        btn.disabled = true;
        btn.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i>';

        try {
            const response = await fetch(`${API_BASE_URL}/api/chat/music/stream`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ video_url: videoUrl })
            });
            const data = await response.json();

            if (data.stream_url) {
                const playerContainer = container.querySelector('.youtube-player-container');
                if (playerContainer) {
                    playerContainer.innerHTML = `
                        <div class="bg-black/60 rounded-lg p-4">
                            <div class="flex items-center gap-3 mb-3">
                                <i class="fa-solid fa-headphones text-accent text-2xl"></i>
                                <div class="text-white text-sm truncate flex-1">🎵 ${videoTitle}</div>
                            </div>
                            <audio controls autoplay class="w-full" style="height: 40px;">
                                <source src="${data.stream_url}" type="audio/mpeg">
                            </audio>
                        </div>
                    `;
                }
            } else {
                throw new Error(data.error || 'ไม่สามารถดึงเสียงได้');
            }
        } catch (e) {
            console.error('Audio stream error:', e);
            btn.innerHTML = '<i class="fa-solid fa-exclamation-triangle"></i> ไม่สำเร็จ';
            setTimeout(() => {
                btn.innerHTML = '<i class="fa-solid fa-headphones"></i> ฟังแค่เสียง';
                btn.disabled = false;
            }, 2000);
        }
    }

    // --- Print Functionality ---
    function printMessage(content, imageUrl, imageGallery) {
        const printWindow = window.open('', '_blank');

        let imagesHtml = '';
        if (imageUrl) {
            imagesHtml += `<img src="${imageUrl}" class="main-image" alt="Main Image">`;
        }
        if (imageGallery && imageGallery.length > 0) {
            imagesHtml += '<div class="gallery">';
            imageGallery.forEach(img => {
                imagesHtml += `<img src="${img.url || img}" alt="Gallery Image">`;
            });
            imagesHtml += '</div>';
        }

        const htmlContent = marked.parse(content);

        printWindow.document.write(`
            <!DOCTYPE html>
            <html lang="th">
            <head>
                <meta charset="UTF-8">
                <title>พิมพ์เอกสาร - AI Guide Nan</title>
                <link href="https://fonts.googleapis.com/css2?family=Sarabun:wght@300;400;600&display=swap" rel="stylesheet">
                <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
                <style>
                    body {
                        font-family: 'Sarabun', sans-serif;
                        line-height: 1.6;
                        color: #333;
                        max-width: 210mm; /* A4 width */
                        margin: 0 auto;
                        padding: 20px;
                        background: white;
                    }
                    @page {
                        size: A4;
                        margin: 20mm;
                    }
                    header {
                        border-bottom: 2px solid #3b82f6;
                        padding-bottom: 10px;
                        margin-bottom: 20px;
                        display: flex;
                        justify-content: space-between;
                        align-items: center;
                    }
                    .brand {
                        font-size: 1.5rem;
                        font-weight: bold;
                        color: #1e40af;
                    }
                    .date {
                        font-size: 0.9rem;
                        color: #666;
                    }
                    .content {
                        font-size: 14px;
                    }
                    h1, h2, h3 { color: #1e3a8a; margin-top: 15px; }
                    ul { margin-left: 20px; }
                    .main-image {
                        width: auto;
                        max-width: 100%;
                        max-height: 200px; /* Reduced height for print */
                        object-fit: cover;
                        border-radius: 8px;
                        margin: 10px auto;
                        display: block;
                    }
                    .content img {
                        max-width: 80%; /* Don't let inline images take full width */
                        max-height: 200px; /* Limit height */
                        width: auto;
                        display: block;
                        margin: 10px auto; /* Center */
                        border-radius: 4px;
                    }
                    .gallery {
                        display: grid;
                        grid-template-columns: repeat(3, 1fr);
                        gap: 10px;
                        margin-top: 15px;
                    }
                    .gallery img {
                        width: 100%;
                        height: 100px;
                        object-fit: cover;
                        border-radius: 4px;
                    }
                    .footer {
                        margin-top: 30px;
                        padding-top: 10px;
                        border-top: 1px solid #ddd;
                        text-align: center;
                        font-size: 0.8rem;
                        color: #888;
                    }
                    @media print {
                        body { -webkit-print-color-adjust: exact; }
                        .no-print { display: none; }
                    }
                </style>
            </head>
            <body>
                <header>
                    <div class="brand"><i class="fa-solid fa-robot"></i> AI Guide Nan</div>
                    <div class="date">พิมพ์เมื่อ: ${new Date().toLocaleString('th-TH')}</div>
                </header>
                
                <div class="content">
                    ${htmlContent}
                    ${imagesHtml}
                </div>

                <div class="footer">
                    เอกสารนี้สร้างโดยระบบ AI Robot Guide จังหวัดน่าน | ข้อมูลเพื่อการท่องเที่ยวเท่านั้น
                </div>

                <script>
                    window.onload = () => { setTimeout(() => window.print(), 500); };
                </script>
            </body>
            </html>
        `);
        printWindow.document.close();
    }

    function sendSystemMessage(text) {
        displayMessage(text, 'system');
    }

    // --- User Input & Mic Control ---
    // 🆕 ใช้ NanApp.INTENTS จาก main.js
    function sendMessage(text = null, intent = null, additionalData = {}) {
        if (!text) text = userInput.value.trim();

        const INTENTS = window.NanApp ? window.NanApp.INTENTS : { GENERAL: 'GENERAL' };
        if (!intent) intent = INTENTS.GENERAL;

        if (text && websocket && websocket.readyState === WebSocket.OPEN) {
            displayMessage(text, 'user');
            const aiMode = window.NanApp ? window.NanApp.getAIModeManager().getMode() : 'fast';

            const payload = {
                query: text,
                ai_mode: aiMode,
                intent: intent,
                user_province: localStorage.getItem('nan_user_province'),
                user_origin: localStorage.getItem('nan_user_origin'),
                ...additionalData
            };

            websocket.send(JSON.stringify(payload));
            userInput.value = '';

            // 🎭 แสดง Thinking Indicator ขณะรอ AI ตอบ
            if (window.ThinkingIndicator) {
                window.ThinkingIndicator.show(messageArea);
            }

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

    // 🚀 Initialize FabManager for handling Tool Buttons
    // 🚀 Initialize FabManager for handling Tool Buttons
    const fabManager = new FabManager({
        buttons: {
            music: 'music-button',
            faq: 'faq-button',
            calc: 'calc-button',
            nav: 'nav-button'
        },
        callbacks: {
            sendMessage: (text, intent, additionalData) => sendMessage(text, intent, additionalData),

            onMusicAction: () => {
                const msgElement = document.createElement('div');
                msgElement.className = 'message system fade-in';
                const widget = fabManager.createMusicWidget();
                msgElement.appendChild(widget);
                messageArea.appendChild(msgElement);
                messageArea.scrollTop = messageArea.scrollHeight;
            },

            onFaqAction: () => {
                const msgElement = document.createElement('div');
                msgElement.className = 'message system fade-in';
                const widget = fabManager.createFAQWidget();
                msgElement.appendChild(widget);
                messageArea.appendChild(msgElement);
                messageArea.scrollTop = messageArea.scrollHeight;
            },

            onNavAction: () => {
                const msgElement = document.createElement('div');
                msgElement.className = 'message system fade-in';
                const widget = fabManager.createNavigationWidget();
                msgElement.appendChild(widget);
                messageArea.appendChild(msgElement);
                messageArea.scrollTop = messageArea.scrollHeight;
            },

            onCalcAction: () => {
                const msgElement = document.createElement('div');
                msgElement.className = 'message system fade-in';
                const widget = fabManager.createCalculatorWidget();
                msgElement.appendChild(widget);
                messageArea.appendChild(msgElement);
                messageArea.scrollTop = messageArea.scrollHeight;
            }
        }
    });

    // 🎵 Function แสดง Music Prompt พร้อมช่องกรอก
    // 🎵 Function displayMusicPrompt removed (Logic moved to FabManager)

    // Handle FAQ button click (ถ้ามีปุ่มใน input bar)
    const faqButtonBar = document.getElementById('faq-button-bar');
    if (faqButtonBar) {
        faqButtonBar.addEventListener('click', () => {
            const faqText = "### ❓ คำถามที่พบบ่อย\n\nลองเลือกคำถามด้านล่างได้เลยนะคะ:";
            const questions = [
                "แนะนำที่เที่ยวน่านหน่อย?",
                "วัดสำคัญในน่านมีที่ไหนบ้าง?",
                "ประเพณีของคนน่านมีอะไรบ้าง?",
                "ร้านอาหารพื้นเมืองที่ห้ามพลาด?",
                "โรงแรมที่พักในเมืองน่าน?"
            ];
            displayMessage(faqText, 'system', null, [], 'normal', [], null, null, questions);
        });
    }

    // 🗺️ Function แสดง Navigation Prompt
    // 🗺️ Function displayNavigationPrompt removed (Logic moved to FabManager)

    // --- Navigation Logic (Task 3 Fix) ---
    // Tab switching logic removed as Travel Mode is now a standalone page.
    // Feedback Submission Logic
    async function submitFeedback(type, messageId, query, response, likeBtn, dislikeBtn) {
        const sessionId = sessionStorage.getItem('session_id') || 'anonymous';

        // UI Feedback
        if (type === 'like') {
            likeBtn.innerHTML = '<i class="fa-solid fa-thumbs-up"></i>';
            likeBtn.classList.add('active');
            dislikeBtn.classList.remove('active');
            dislikeBtn.innerHTML = '<i class="fa-regular fa-thumbs-down"></i>';
        } else {
            dislikeBtn.innerHTML = '<i class="fa-solid fa-thumbs-down"></i>';
            dislikeBtn.classList.add('active');
            likeBtn.classList.remove('active');
            likeBtn.innerHTML = '<i class="fa-regular fa-thumbs-up"></i>';
        }

        try {
            const payload = {
                session_id: sessionId,
                query: query || "unknown_query",
                response: response.substring(0, 500), // Limit length
                feedback_type: type
            };

            const res = await fetch('/api/analytics/submit_feedback', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(payload)
            });

            if (res.ok) {
                console.log(`✅ Feedback submitted: ${type}`);
                // Disable buttons after success to prevent spam
                likeBtn.disabled = true;
                dislikeBtn.disabled = true;
                likeBtn.style.opacity = '0.5';
                dislikeBtn.style.opacity = '0.5';
            } else {
                console.error('Failed to submit feedback');
            }
        } catch (e) {
            console.error('Feedback error:', e);
        }
    }
    // --- Initialization ---
    connectChatWebSocket();
});