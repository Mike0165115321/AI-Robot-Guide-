// /frontend/assets/scripts/chat.js (Ultimate Final - Pure CSS & Map Support)

// --- Class จัดการไมโครโฟน ---
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
            this.callbacks.onError('Browser does not support speech recognition.');
            return null;
        }
        const recognition = new SpeechRecognition();
        recognition.lang = 'th-TH';
        recognition.interimResults = true;
        recognition.continuous = true;

        recognition.onstart = () => {
            this.isListening = true;
            this.finalTranscript = '';
            this.callbacks.onStartRecording();
        };
        recognition.onend = () => {
            this.isListening = false;
            this.callbacks.onStopRecording();
            if (this.finalTranscript.trim()) this.callbacks.onFinalTranscript(this.finalTranscript.trim());
        };
        recognition.onerror = (event) => {
            if (event.error !== 'no-speech') this.callbacks.onError(event.error);
        };
        recognition.onresult = (event) => {
            let interimTranscript = '';
            for (let i = event.resultIndex; i < event.results.length; ++i) {
                if (event.results[i].isFinal) this.finalTranscript += event.results[i][0].transcript;
                else interimTranscript += event.results[i][0].transcript;
            }
            this.callbacks.onInterimTranscript(this.finalTranscript + interimTranscript);
        };
        return recognition;
    }
    start() { if (!this.isListening && this.recognition) this.recognition.start(); }
    stop() { if (this.isListening && this.recognition) this.recognition.stop(); }
}

// --- Main Chat Logic ---
document.addEventListener('DOMContentLoaded', () => {
    const messageArea = document.getElementById('message-area');
    const userInput = document.getElementById('user-input');
    const sendButton = document.getElementById('send-button-icon');
    const micButton = document.getElementById('mic-button');
    const newChatBtn = document.getElementById('new-chat-btn');
    
    // Sidebar Buttons
    document.getElementById('travel-mode-btn')?.addEventListener('click', () => window.location.href = 'travel_mode');
    document.getElementById('convo-btn')?.addEventListener('click', () => window.open('robot_avatar.html', '_blank'));

    let isAnswering = false;
    let browserMicHandler = null;

    // Session Management
    const SESSION_ID_KEY = 'nan_chat_session_id';
    let currentSessionId = localStorage.getItem(SESSION_ID_KEY) || (() => {
        const id = 'session-' + Date.now();
        localStorage.setItem(SESSION_ID_KEY, id);
        return id;
    })();

    const createEmptyResponse = (text) => ({ answer: text, image_gallery: [], sources: [] });

    async function sendMessage(queryOverride = null) {
        const query = queryOverride || userInput.value.trim();
        if (!query || isAnswering) return;

        addMessage(query, 'user');
        userInput.value = '';
        userInput.style.height = 'auto'; // Reset height
        
        isAnswering = true;
        sendButton.disabled = true;
        micButton.disabled = true;
        
        const loadingMsg = addMessage('กำลังประมวลผล...', 'ai-thinking');

        try {
            const response = await fetch(`${API_BASE_URL}/api/chat/`, { 
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ query: query, session_id: currentSessionId }), 
            });
            if (!response.ok) throw new Error('Network response was not ok');
            const data = await response.json();

            loadingMsg.remove();
            addMessage(data, 'ai');
        } catch (error) {
            console.error(error);
            loadingMsg.remove();
            addMessage(createEmptyResponse('ขออภัยค่ะ เกิดข้อผิดพลาดในการเชื่อมต่อ'), 'ai');
        } finally {
            isAnswering = false;
            sendButton.disabled = false;
            micButton.disabled = false;
            userInput.focus();
        }
    }

    function addMessage(data, type) {
        const row = document.createElement('div');
        // ใช้ class ตาม CSS ใหม่ (message-row)
        row.className = `message-row ${type === 'user' ? 'user' : 'ai'}`;

        let innerContent = '';

        if (type === 'user') {
            // User Message
            innerContent = `<div class="bubble user">${data}</div>`;
        } else {
            // AI Message
            if (type === 'ai-thinking') {
                innerContent = `
                <div class="robot-icon-mini"><div class="icon-eyes"></div></div>
                <div class="bubble ai typing-indicator">${data}</div>`;
            } else {
                // Markdown Content
                const answerHtml = window.marked ? marked.parse(data.answer || '') : data.answer;
                
                // 1. Images Grid (รูปภาพแบบ Grid)
                let imagesHtml = '';
                if (data.image_gallery && data.image_gallery.length > 0) {
                    imagesHtml = `<div class="chat-images-grid">` + 
                        data.image_gallery.slice(0, 4).map(url => 
                            `<img src="${url}" class="chat-image" onclick="window.open('${url}')" onerror="this.style.display='none'">`
                        ).join('') + 
                    `</div>`;
                }

                // 2. Map Embed (แผนที่ - ส่วนที่เคยขาดไป)
                let mapHtml = '';
                if (data.action === 'SHOW_MAP_EMBED') {
                    const { embed_url, external_link } = data.action_payload;
                    if (embed_url) {
                        mapHtml = `
                        <div class="map-wrapper">
                            <iframe width="100%" height="250" src="${embed_url}" style="border:0;" allowfullscreen loading="lazy"></iframe>
                        </div>
                        ${external_link ? `<a href="${external_link}" target="_blank" class="btn-map-link"><i class="fa-solid fa-location-arrow"></i> เปิดใน Google Maps</a>` : ''}
                        `;
                    }
                }
                
                // 3. Song Choices (ตัวเลือกเพลง)
                let songChoicesHtml = '';
                if (data.action === 'SHOW_SONG_CHOICES' && Array.isArray(data.action_payload)) {
                    songChoicesHtml = data.action_payload.map((s, index) => 
                        `<button class="song-choice-btn" data-song-index="${index}">🎵 ${s.title}</button>`
                    ).join('');
                }

                // 4. Song Input (ช่องค้นหาเพลง)
                let songInputHtml = '';
                if (data.action === 'PROMPT_FOR_SONG_INPUT') {
                    const placeholder = data.action_payload?.placeholder || 'พิมพ์ชื่อเพลง...';
                    songInputHtml = `
                    <div class="song-input-wrapper" id="song-wrapper-${Date.now()}">
                        <input type="text" class="song-input-field" placeholder="${placeholder}">
                        <button class="song-submit-btn"><i class="fa-solid fa-music"></i> ค้นหา</button>
                    </div>`;
                }

                innerContent = `
                <div class="robot-icon-mini"><div class="icon-eyes"></div></div>
                <div class="bubble ai">
                    <div class="markdown-content">${answerHtml}</div>
                    ${imagesHtml}
                    ${mapHtml}
                    ${songChoicesHtml}
                    ${songInputHtml}
                    <div class="youtube-player-container"></div>
                </div>`;
            }
        }

        row.innerHTML = innerContent;
        messageArea.appendChild(row);
        
        // Auto-hide broken images
        row.querySelectorAll('.markdown-content img').forEach(img => {
            img.classList.add('chat-image'); // ใช้ Style เดียวกัน
            img.onerror = function() { this.style.display = 'none'; };
        });

        // --- Event Bindings ---

        // Song Input Handling
        const songWrapper = row.querySelector('.song-input-wrapper');
        if (songWrapper) {
            const input = songWrapper.querySelector('input');
            const btn = songWrapper.querySelector('button');
            const submitSong = () => {
                const val = input.value.trim();
                if(val) {
                    sendMessage(val);
                    input.disabled = true; btn.disabled = true;
                    songWrapper.style.opacity = 0.7;
                }
            };
            btn.addEventListener('click', submitSong);
            input.addEventListener('keydown', (e) => { if(e.key==='Enter') submitSong(); });
            input.focus();
        }

        // Song Choice Handling
        const choiceBtns = row.querySelectorAll('.song-choice-btn');
        if (choiceBtns.length > 0 && data.action_payload) {
            choiceBtns.forEach(btn => {
                btn.addEventListener('click', () => {
                    const idx = btn.dataset.songIndex;
                    const song = data.action_payload[idx];
                    const playerContainer = row.querySelector('.youtube-player-container');
                    if(playerContainer && song) {
                        playerContainer.innerHTML = `
                        <div class="map-wrapper" style="margin-top:15px;">
                             <iframe width="100%" height="250" src="https://www.youtube.com/embed/${song.video_id}?autoplay=1" frameborder="0" allowfullscreen></iframe>
                        </div>
                        <p style="margin-top:5px; font-size:0.9rem; color:#2dd4bf;">🎵 กำลังเล่น: ${song.title}</p>
                        `;
                    }
                });
            });
        }

        messageArea.scrollTop = messageArea.scrollHeight;
        return row;
    }

    // FAQ Button
    document.getElementById('faq-button')?.addEventListener('click', () => {
        const existingFaq = document.getElementById('faq-container');
        if (existingFaq) existingFaq.remove();

        const faqs = ["แนะนำที่เที่ยวธรรมชาติ", "วัดสวยๆ ในเมือง", "ของกินพื้นเมือง", "เปิดเพลงให้ฟังหน่อย", "นำทางไปดอยเสมอดาว"];
        const faqHtml = faqs.map(q => `<button class="song-choice-btn" style="margin-top:5px;">${q}</button>`).join('');
        
        const row = document.createElement('div');
        row.id = 'faq-container';
        row.className = 'message-row ai';
        row.innerHTML = `
            <div class="robot-icon-mini"><div class="icon-eyes"></div></div>
            <div class="bubble ai">
                <p>ลองถามเรื่องพวกนี้ดูมั้ยคะ?</p>
                ${faqHtml}
            </div>`;
        
        messageArea.appendChild(row);
        messageArea.scrollTop = messageArea.scrollHeight;

        row.querySelectorAll('button').forEach(btn => {
            btn.addEventListener('click', () => {
                sendMessage(btn.innerText);
                row.remove();
            });
        });
    });

    // Reset Chat
    newChatBtn.addEventListener('click', () => {
        messageArea.innerHTML = '';
        localStorage.removeItem(SESSION_ID_KEY);
        currentSessionId = 'session-' + Date.now();
        localStorage.setItem(SESSION_ID_KEY, currentSessionId);
        addMessage(createEmptyResponse("สวัสดีเจ้า... เริ่มทริปใหม่กันเลย! มีอะไรให้น้องน่านช่วยมั้ยคะ?"), 'ai');
    });

    // Input & Mic Handlers
    userInput.addEventListener('keydown', (e) => {
        if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); sendMessage(); }
    });
    userInput.addEventListener('input', function() {
        this.style.height = 'auto';
        this.style.height = (this.scrollHeight) + 'px';
        if(this.value === '') this.style.height = '40px';
    });
    sendButton.addEventListener('click', () => sendMessage());

    micButton.addEventListener('click', () => {
        if (!browserMicHandler) {
             browserMicHandler = new BrowserMicHandler({
                onStartRecording: () => { micButton.classList.add('listening'); userInput.placeholder = 'กำลังฟัง...'; },
                onStopRecording: () => { micButton.classList.remove('listening'); userInput.placeholder = 'ถามน้องน่านได้เลยเจ้า...'; },
                onInterimTranscript: (txt) => userInput.value = txt,
                onFinalTranscript: (txt) => { userInput.value = txt; sendMessage(); },
                onError: (err) => alert(err)
             });
        }
        if (browserMicHandler.isListening) browserMicHandler.stop();
        else browserMicHandler.start();
    });

    // Welcome Message
    addMessage(createEmptyResponse("สวัสดีเจ้า 🙏 ยินดีต้อนรับสู่เมืองน่าน มีอะหยังหื้อน้องน่านช่วยก่อเจ้า?"), 'ai');
});