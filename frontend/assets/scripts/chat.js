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
// AI Mode Manager
// - Toggle between Fast (Llama) and Detailed (Gemini) modes
// ========================================
class AIModeManager {
    constructor() {
        this.mode = localStorage.getItem('nan_ai_mode') || 'fast'; // fast | detailed
        this.button = null;
        this.indicator = null;
    }

    init() {
        this.button = document.getElementById('ai-mode-btn');
        this.indicator = document.getElementById('mode-indicator');

        if (this.button) {
            this.updateUI();
            this.button.addEventListener('click', () => this.toggleMode());
        }
    }

    toggleMode() {
        this.mode = this.mode === 'fast' ? 'detailed' : 'fast';
        localStorage.setItem('nan_ai_mode', this.mode);
        this.updateUI();
        this.showIndicator();
        console.log(`🔄 AI Mode switched to: ${this.mode}`);
    }

    updateUI() {
        if (!this.button || !this.indicator) return;

        // Update button
        this.button.classList.remove('fast-mode', 'detailed-mode');
        this.button.classList.add(`${this.mode}-mode`);

        // Update icon
        const icon = this.button.querySelector('i');
        if (icon) {
            icon.className = this.mode === 'fast'
                ? 'fa-solid fa-bolt'
                : 'fa-solid fa-brain';
        }

        // Update indicator
        this.indicator.classList.remove('fast', 'detailed');
        this.indicator.classList.add(this.mode);
        this.indicator.textContent = this.mode === 'fast'
            ? '⚡ คิดเร็ว (Llama)'
            : '🧠 คิดละเอียด (Gemini)';
    }

    showIndicator() {
        if (!this.indicator) return;
        this.indicator.classList.add('show');
        setTimeout(() => this.indicator.classList.remove('show'), 2000);
    }

    getMode() {
        return this.mode;
    }
}

window.AIModeManager = new AIModeManager();

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

    // Init AI Mode Manager
    if (window.AIModeManager) {
        window.AIModeManager.init();
    }

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
                displayMessage(data.answer || "ขออภัยค่ะ ไม่เข้าใจคำถาม", 'ai', data.image_url, data.image_gallery, data.emotion, data.sources, data.action, data.action_payload);

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

            bubbleContainer.appendChild(bubble);

            // Only show Copy/Print buttons for informational messages (not interactive ones)
            const interactiveActions = ['SHOW_MAP_EMBED', 'SHOW_SONG_CHOICES', 'PROMPT_FOR_SONG_INPUT'];
            if (!interactiveActions.includes(action)) {
                // Print Button
                const actionsBar = document.createElement('div');
                actionsBar.style.display = 'flex';
                actionsBar.style.justifyContent = 'flex-end';
                actionsBar.style.paddingRight = '10px';

                const printBtn = document.createElement('button');
                printBtn.className = 'btn-icon';
                printBtn.style.width = '30px';
                printBtn.style.height = '30px';
                printBtn.style.fontSize = '0.9rem';
                printBtn.title = 'พิมพ์หน้านี้';
                printBtn.innerHTML = '<i class="fa-solid fa-print"></i>';
                printBtn.onclick = () => printMessage(text, imageUrl, imageGallery);

                // Copy Button
                const copyBtn = document.createElement('button');
                copyBtn.className = 'btn-icon';
                copyBtn.style.width = '30px';
                copyBtn.style.height = '30px';
                copyBtn.style.fontSize = '0.9rem';
                copyBtn.title = 'คัดลอกข้อความ';
                copyBtn.innerHTML = '<i class="fa-regular fa-copy"></i>';
                copyBtn.onclick = () => {
                    navigator.clipboard.writeText(text).then(() => {
                        copyBtn.innerHTML = '<i class="fa-solid fa-check" style="color: var(--success-color);"></i>';
                        setTimeout(() => {
                            copyBtn.innerHTML = '<i class="fa-regular fa-copy"></i>';
                        }, 2000);
                    }).catch(err => {
                        console.error('Failed to copy: ', err);
                    });
                };

                actionsBar.appendChild(copyBtn);
                actionsBar.appendChild(printBtn);
                bubbleContainer.appendChild(actionsBar);
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
        } else if (action === 'PROMPT_FOR_SONG_INPUT' && actionPayload) {
            showSongInput(actionPayload.placeholder || "พิมพ์ชื่อเพลง...");
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

    function showSongChoices(songs) {
        const lastMessage = messageArea.lastElementChild;
        if (!lastMessage) return;

        const bubble = lastMessage.querySelector('.bubble');
        if (!bubble) return;

        const songContainer = document.createElement('div');
        songContainer.className = 'song-choices-container mt-4 grid grid-cols-1 gap-2';

        songs.forEach(song => {
            const songCard = document.createElement('div');
            songCard.className = 'flex items-center gap-3 p-3 bg-black/40 rounded-lg hover:bg-black/60 transition cursor-pointer border border-glass-border';
            songCard.innerHTML = `
                <div class="w-10 h-10 rounded-full bg-accent/20 flex items-center justify-center text-accent">
                    <i class="fa-solid fa-music"></i>
                </div>
                <div class="flex-1 min-w-0">
                    <div class="text-sm font-medium text-white truncate">${song.title}</div>
                    <div class="text-xs text-gray-400 truncate">${song.channel}</div>
                </div>
                <button class="text-primary hover:text-white transition">
                    <i class="fa-solid fa-play"></i>
                </button>
            `;
            songCard.onclick = () => {
                // ใช้ MusicPlayer class ใหม่ (ถ้ามี)
                if (typeof musicPlayer !== 'undefined' && musicPlayer.createPlayer) {
                    // Normalize song object
                    const normalizedSong = {
                        video_id: song.video_id || song.id,
                        title: song.title,
                        channel: song.channel,
                        url: song.url || `https://www.youtube.com/watch?v=${song.video_id || song.id}`
                    };
                    musicPlayer.createPlayer(normalizedSong, songContainer);
                } else {
                    // Fallback: เปิด YouTube ใน tab ใหม่
                    const videoId = song.video_id || song.id;
                    window.open(`https://www.youtube.com/watch?v=${videoId}`, '_blank');
                }
            };
            songContainer.appendChild(songCard);
        });

        bubble.appendChild(songContainer);
        messageArea.scrollTop = messageArea.scrollHeight;
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
    function sendMessage(text = null, intent = null) {
        if (!text) text = userInput.value.trim();

        // ใช้ INTENTS จาก main.js
        const INTENTS = window.NanApp ? window.NanApp.INTENTS : { GENERAL: 'GENERAL' };
        if (!intent) intent = INTENTS.GENERAL;

        if (text && websocket && websocket.readyState === WebSocket.OPEN) {
            displayMessage(text, 'user');
            // ใช้ AIModeManager จาก main.js
            const aiMode = window.NanApp ? window.NanApp.getAIModeManager().getMode() : 'fast';
            websocket.send(JSON.stringify({
                query: text,
                ai_mode: aiMode,
                intent: intent
            }));
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

    // Handle Music button click - แสดงตัวเลือกแนวเพลง + ช่องกรอกชื่อเพลง
    if (musicButton) {
        musicButton.addEventListener('click', () => {
            // สร้างข้อความพร้อมปุ่มเลือกและช่องกรอก
            displayMusicPrompt();
        });
    }

    // 🎵 Function แสดง Music Prompt พร้อมช่องกรอก
    function displayMusicPrompt() {
        const messageId = `msg-${Date.now()}`;
        const msgElement = document.createElement('div');
        msgElement.className = 'message system fade-in';
        msgElement.id = messageId;

        msgElement.innerHTML = `
            <div class="bubble system-bubble">
                <div class="prose" style="font-size: 0.95rem;">
                    <h3 style="margin-top: 0;">🎵 ฟังเพลงกันเถอะ!</h3>
                    <p>เลือกแนวเพลงที่ชอบ หรือพิมพ์ชื่อเพลง/ศิลปินได้เลยค่ะ:</p>
                </div>
                <div class="music-genres" style="display: flex; flex-wrap: wrap; gap: 8px; margin: 15px 0;">
                    <button class="genre-btn" data-query="เปิดเพลงเศร้าให้หน่อย">😢 เพลงเศร้า</button>
                    <button class="genre-btn" data-query="เปิดเพลงสนุกๆ ให้หน่อย">🎉 เพลงสนุก</button>
                    <button class="genre-btn" data-query="เปิดเพลงรักหวานๆ">💕 เพลงรัก</button>
                    <button class="genre-btn" data-query="เปิดเพลงลูกทุ่งให้ฟัง">🌾 ลูกทุ่ง</button>
                    <button class="genre-btn" data-query="เปิดเพลงม่วนๆ ภาษาเหนือ">🏔️ เพลงเหนือ</button>
                </div>
                <div class="music-search-container" style="display: flex; gap: 8px; margin-top: 10px;">
                    <input type="text" class="music-search-input" placeholder="หรือพิมพ์ชื่อเพลง/ศิลปิน..." style="
                        flex: 1;
                        padding: 10px 15px;
                        border: 1px solid rgba(255,255,255,0.2);
                        border-radius: 8px;
                        background: rgba(0,0,0,0.3);
                        color: white;
                        font-size: 0.9rem;
                    ">
                    <button class="music-search-btn" style="
                        padding: 10px 20px;
                        background: linear-gradient(135deg, #10b981, #059669);
                        border: none;
                        border-radius: 8px;
                        color: white;
                        cursor: pointer;
                        font-weight: bold;
                    ">🔍 ค้นหา</button>
                </div>
            </div>
        `;

        // Add styles for genre buttons
        const style = document.createElement('style');
        style.textContent = `
            .genre-btn {
                padding: 8px 16px;
                background: rgba(16, 185, 129, 0.2);
                border: 1px solid rgba(16, 185, 129, 0.4);
                border-radius: 20px;
                color: #10b981;
                cursor: pointer;
                font-size: 0.85rem;
                transition: all 0.2s;
            }
            .genre-btn:hover {
                background: rgba(16, 185, 129, 0.4);
                transform: scale(1.05);
            }
        `;
        if (!document.querySelector('#music-prompt-styles')) {
            style.id = 'music-prompt-styles';
            document.head.appendChild(style);
        }

        messageArea.appendChild(msgElement);
        messageArea.scrollTop = messageArea.scrollHeight;

        // Add event listeners - ใช้ NanApp.INTENTS
        const INTENTS = window.NanApp ? window.NanApp.INTENTS : { MUSIC: 'MUSIC' };
        msgElement.querySelectorAll('.genre-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const query = btn.dataset.query;
                displayMessage(query, 'user');
                const aiMode = window.NanApp ? window.NanApp.getAIModeManager().getMode() : 'fast';
                websocket.send(JSON.stringify({ query: query, ai_mode: aiMode, intent: INTENTS.MUSIC }));
            });
        });

        const searchInput = msgElement.querySelector('.music-search-input');
        const searchBtn = msgElement.querySelector('.music-search-btn');

        searchBtn.addEventListener('click', () => {
            if (searchInput.value.trim()) {
                const query = `เปิดเพลง ${searchInput.value.trim()}`;
                displayMessage(query, 'user');
                const aiMode = window.NanApp ? window.NanApp.getAIModeManager().getMode() : 'fast';
                websocket.send(JSON.stringify({ query: query, ai_mode: aiMode, intent: INTENTS.MUSIC }));
            }
        });

        searchInput.addEventListener('keypress', (e) => {
            if (e.key === 'Enter' && searchInput.value.trim()) {
                const query = `เปิดเพลง ${searchInput.value.trim()}`;
                displayMessage(query, 'user');
                websocket.send(JSON.stringify({ query: query }));
            }
        });

        searchInput.focus();
    }

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

    // Handle Calculator button click
    const calcButton = document.getElementById('calc-button');
    if (calcButton) {
        calcButton.addEventListener('click', () => {
            displayMessage("เปิดเครื่องคิดเลข", 'user');
            websocket.send(JSON.stringify({ query: "เปิดเครื่องคิดเลข" }));
        });
    }

    // Handle Navigation button click
    const navButton = document.getElementById('nav-button');
    if (navButton) {
        navButton.addEventListener('click', () => {
            displayNavigationPrompt();
        });
    }

    // 🗺️ Function แสดง Navigation Prompt
    function displayNavigationPrompt() {
        const msgElement = document.createElement('div');
        msgElement.className = 'message system fade-in';
        msgElement.id = `msg-${Date.now()}`;

        msgElement.innerHTML = `
            <div class="bubble system-bubble">
                <div class="prose" style="font-size: 0.95rem;">
                    <h3 style="margin-top: 0;">🗺️ จะไปไหนดีคะ?</h3>
                    <p>เลือกสถานที่ยอดนิยม หรือพิมพ์ชื่อสถานที่ที่ต้องการไป:</p>
                </div>
                <div class="nav-locations" style="display: flex; flex-wrap: wrap; gap: 8px; margin: 15px 0;">
                    <button class="nav-loc-btn" data-query="พาไปวัดภูมินทร์" style="padding: 8px 16px; background: rgba(59, 130, 246, 0.2); border: 1px solid rgba(59, 130, 246, 0.4); border-radius: 20px; color: #3b82f6; cursor: pointer;">🛕 วัดภูมินทร์</button>
                    <button class="nav-loc-btn" data-query="นำทางไปดอยเสมอดาว" style="padding: 8px 16px; background: rgba(59, 130, 246, 0.2); border: 1px solid rgba(59, 130, 246, 0.4); border-radius: 20px; color: #3b82f6; cursor: pointer;">⛰️ ดอยเสมอดาว</button>
                    <button class="nav-loc-btn" data-query="พาไปวัดช้างค้ำ" style="padding: 8px 16px; background: rgba(59, 130, 246, 0.2); border: 1px solid rgba(59, 130, 246, 0.4); border-radius: 20px; color: #3b82f6; cursor: pointer;">🐘 วัดช้างค้ำ</button>
                    <button class="nav-loc-btn" data-query="นำทางไปถนนคนเดิน" style="padding: 8px 16px; background: rgba(59, 130, 246, 0.2); border: 1px solid rgba(59, 130, 246, 0.4); border-radius: 20px; color: #3b82f6; cursor: pointer;">🚶 ถนนคนเดิน</button>
                </div>
                <div style="display: flex; gap: 8px; margin-top: 10px;">
                    <input type="text" id="nav-search-input" placeholder="หรือพิมพ์ชื่อสถานที่..." style="
                        flex: 1;
                        padding: 10px 15px;
                        border: 1px solid rgba(255,255,255,0.2);
                        border-radius: 8px;
                        background: rgba(0,0,0,0.3);
                        color: white;
                        font-size: 0.9rem;
                    ">
                    <button id="nav-search-btn" style="
                        padding: 10px 20px;
                        background: linear-gradient(135deg, #3b82f6, #2563eb);
                        border: none;
                        border-radius: 8px;
                        color: white;
                        cursor: pointer;
                        font-weight: bold;
                    ">🗺️ นำทาง</button>
                </div>
            </div>
        `;

        messageArea.appendChild(msgElement);
        messageArea.scrollTop = messageArea.scrollHeight;

        // Event listeners for location buttons
        msgElement.querySelectorAll('.nav-loc-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const query = btn.dataset.query;
                displayMessage(query, 'user');
                websocket.send(JSON.stringify({ query: query }));
            });
        });

        const navInput = msgElement.querySelector('#nav-search-input');
        const navSearchBtn = msgElement.querySelector('#nav-search-btn');

        navSearchBtn.addEventListener('click', () => {
            if (navInput.value.trim()) {
                const query = `นำทางไป ${navInput.value.trim()}`;
                displayMessage(query, 'user');
                websocket.send(JSON.stringify({ query: query }));
            }
        });

        navInput.addEventListener('keypress', (e) => {
            if (e.key === 'Enter' && navInput.value.trim()) {
                const query = `นำทางไป ${navInput.value.trim()}`;
                displayMessage(query, 'user');
                websocket.send(JSON.stringify({ query: query }));
            }
        });

        navInput.focus();
    }

    // --- Navigation Logic (Task 3 Fix) ---
    // Tab switching logic removed as Travel Mode is now a standalone page.

    // --- Initialization ---
    connectChatWebSocket();
});