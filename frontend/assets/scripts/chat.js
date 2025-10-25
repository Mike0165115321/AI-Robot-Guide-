document.addEventListener('DOMContentLoaded', () => {
    // --- Element References ---
    const messageArea = document.getElementById('message-area');
    const userInput = document.getElementById('user-input');
    const sendButton = document.getElementById('send-button-icon');
    const micButton = document.getElementById('mic-button');
    const convoBtn = document.getElementById('convo-btn');
    const faqButton = document.getElementById('faq-button');
    const newChatBtn = document.getElementById('new-chat-btn');
    const inputArea = document.querySelector('.input-area-container');

    let isAnswering = false;

    // A default empty response object for error cases
    const createEmptyResponse = (answerText) => ({
        answer: answerText,
        action: null,
        action_payload: null,
        image_url: null,
        image_gallery: [],
        sources: []
    });

    // --- Initialize Voice Handler ---
    const voiceHandler = new VoiceHandler({
        onStartRecording: () => { micButton.classList.add('mic-listening'); },
        onStopRecording: () => { micButton.classList.remove('mic-listening'); },
        onFinalTranscript: (text) => {
            userInput.value = text;
            if (text.trim()) sendMessage();
        },
        onError: (error) => { addMessage(createEmptyResponse(`ขออภัยค่ะ เกิดข้อผิดพลาดกับระบบเสียง: ${error}`), 'ai'); }
    });

    // --- Send Message Function ---
    async function sendMessage(queryOverride = null) {
        const query = queryOverride || userInput.value.trim();
        if (!query || isAnswering) return;

        addMessage(query, 'user');
        userInput.value = '';
        adjustTextareaHeight(userInput);
        isAnswering = true;
        sendButton.disabled = true;
        micButton.disabled = true;
        const thinkingMessageElement = addMessage('กำลังประมวลผล...', 'ai-thinking');

        try {
            const response = await fetch(`/api/chat/`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ query: query }),
            });
            if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
            const data = await response.json();

            thinkingMessageElement?.remove();
            addMessage(data, 'ai');
        } catch (error)
        {
            console.error('Chat error:', error);
            thinkingMessageElement?.remove();
            addMessage(createEmptyResponse('ขออภัยค่ะ เกิดข้อผิดพลาดในการเชื่อมต่อ'), 'ai');
        } finally {
            isAnswering = false;
            sendButton.disabled = false;
            micButton.disabled = false;
            if (!document.querySelector('.song-input-field')) {
                 userInput.focus();
            }
        }
    }

    function addMessage(data, type) {
        const messageId = `msg-${Date.now()}`;
        const messageElement = document.createElement('div');
        messageElement.id = messageId;
        let html = '';

        if (type === 'user') {
            messageElement.className = 'flex justify-end user-message';
            html = `<div class="bg-blue-600 p-3 rounded-2xl rounded-br-lg max-w-md text-white shadow-md"><p>${data.replace(/</g, "&lt;").replace(/>/g, "&gt;")}</p></div>`;
        } else { // ai, ai-thinking
            messageElement.className = 'flex items-start space-x-3 ai-message';
            let contentHtml = '';

            if (type === 'ai-thinking') {
                contentHtml = `<p class="italic text-text-secondary animate-pulse">${data}</p>`;
            } else { // type === 'ai'
                const answerHtml = window.marked ? marked.parse(data.answer || '') : (data.answer || '');
                contentHtml = `<div class="prose prose-invert max-w-none text-text-primary">${answerHtml}</div>`;

                if (data.image_gallery && data.image_gallery.length > 0) {
                    const galleryHtml = data.image_gallery.slice(0, 5).map(imgUrl => `
                        <div class="flex-shrink-0 w-full sm:w-1/2 lg:w-1/3 p-1">
                            <div class="relative group rounded-lg overflow-hidden border border-border">
                                <a href="${imgUrl}" target="_blank" rel="noopener noreferrer">
                                    <img src="${imgUrl}" alt="Gallery Image" class="w-full h-32 object-cover transition-transform duration-300 group-hover:scale-105">
                                </a>
                            </div>
                        </div>
                    `).join('');
                    contentHtml += `<div class="mt-4"><h4 class="text-sm font-semibold text-text-secondary mb-2">รูปภาพประกอบ:</h4><div class="flex flex-wrap -m-1">${galleryHtml}</div></div>`;
                }

                if (data.action === 'PROMPT_FOR_SONG_INPUT') {
                    const placeholder = data.action_payload?.placeholder || 'พิมพ์ชื่อเพลง หรือ ศิลปิน...';
                    contentHtml += `
                        <div id="song-input-wrapper-${messageId}" class="mt-4 flex items-center space-x-2">
                            <input type="text" placeholder="${placeholder}" class="song-input-field flex-grow bg-slate-700/60 text-text-primary border border-border rounded-lg py-2 px-4 focus:outline-none focus:ring-2 focus:ring-blue-500 transition">
                            <button class="song-submit-btn bg-blue-600 text-white rounded-lg p-2 hover:bg-blue-500 transition">
                                <svg xmlns="http://www.w3.org/2000/svg" class="h-5 w-5" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z" /></svg>
                            </button>
                        </div>
                    `;
                }
                else if (data.action === 'SHOW_SONG_CHOICES' && Array.isArray(data.action_payload)) {
                    const songsHtml = data.action_payload.map((song, index) => `
                        <button class="song-choice-btn block w-full text-left p-2.5 mt-2 bg-slate-700/60 hover:bg-slate-700 rounded-lg transition" data-song-index="${index}">
                            🎵 ${song.title.replace(/</g, "&lt;").replace(/>/g, "&gt;")}
                        </button>
                    `).join('');
                    contentHtml += `<div class="mt-4">${songsHtml}</div>`;
                }
                else if (!data.action && data.sources && data.sources.length > 0) {
                    const sourcesWithImages = data.sources.filter(s => s.image_urls && s.image_urls.length > 0);
                    if (sourcesWithImages.length > 0) {
                        const sourcesHtml = sourcesWithImages.slice(0, 3).map(source => {
                            const imageUrlToShow = source.image_urls[0];
                            const title = source.title || 'แหล่งข้อมูล';
                            return `
                                <div class="flex-shrink-0 w-full sm:w-1/2 lg:w-1/3 p-1">
                                    <div class="relative group rounded-lg overflow-hidden border border-border cursor-pointer" onclick="window.open('${imageUrlToShow}', '_blank')">
                                        <img src="${imageUrlToShow}" alt="${title}" class="w-full h-32 object-cover transition-transform duration-300 group-hover:scale-105">
                                        <div class="absolute bottom-0 left-0 w-full p-2 bg-black/60 backdrop-blur-sm">
                                            <p class="text-xs text-white truncate font-semibold">${title.replace(/</g, "&lt;").replace(/>/g, "&gt;")}</p>
                                        </div>
                                    </div>
                                </div>
                            `;
                        }).join('');
                        contentHtml += `<div class="mt-4"><h4 class="text-sm font-semibold text-text-secondary mb-2">แหล่งข้อมูลเพิ่มเติม:</h4><div class="flex flex-wrap -m-1">${sourcesHtml}</div></div>`;
                    }
                }
            }
            html = `<div class="robot-icon-container"><div class="icon-head-top-accent"></div><div class="icon-robot-face"><div class="icon-robot-eye"></div><div class="icon-robot-eye"></div></div></div><div class="glass-ai-bubble p-4 rounded-2xl rounded-bl-lg max-w-md w-full shadow-lg">${contentHtml}</div>`;
        }
        messageElement.innerHTML = html;
        messageArea.appendChild(messageElement);

        if (type === 'ai' && data.action === 'PROMPT_FOR_SONG_INPUT') {
            const wrapper = document.getElementById(`song-input-wrapper-${messageId}`);
            if (wrapper) {
                const inputField = wrapper.querySelector('.song-input-field');
                const submitButton = wrapper.querySelector('.song-submit-btn');
                if (inputField && submitButton) {
                    const submitSong = () => {
                        const query = inputField.value.trim();
                        if (query) {
                            sendMessage(query);
                            inputField.disabled = true;
                            submitButton.disabled = true;
                            wrapper.style.opacity = '0.7';
                        }
                    };
                    submitButton.addEventListener('click', submitSong);
                    inputField.addEventListener('keydown', (e) => {
                        if (e.key === 'Enter') { e.preventDefault(); submitSong(); }
                    });
                    inputField.focus();
                }
            }
        }

        if (type === 'ai' && data.action === 'SHOW_SONG_CHOICES') {
            messageElement.querySelectorAll('.song-choice-btn').forEach(button => {
                button.addEventListener('click', () => {
                    const songIndex = parseInt(button.dataset.songIndex, 10);
                    const selectedSong = data.action_payload[songIndex];
                    playVideoInBubble(selectedSong, messageElement, data.answer);
                });
            });
        }
        messageArea.scrollTop = messageArea.scrollHeight;
        return messageElement;
    }

    function playVideoInBubble(song, messageElement, originalAnswer) {
        if (!song || !song.video_id || !messageElement) return;
        const bubbleContent = messageElement.querySelector('.glass-ai-bubble');
        if (!bubbleContent) return;
        const answerHtml = `<div class="prose prose-invert max-w-none text-text-primary"><p>${originalAnswer.replace(/</g, "&lt;")}</p><p>กำลังเล่นเพลง: <strong>${song.title.replace(/</g, "&lt;")}</strong></p></div>`;
        const iframeHtml = `<div class="youtube-player-wrapper mt-4 rounded-lg overflow-hidden border border-border"><iframe width="100%" height="250" src="https://www.youtube.com/embed/${song.video_id}?autoplay=1&rel=0" title="${song.title}" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe></div>`;
        const fallbackLinkHtml = `<div class="mt-2 text-center text-sm"><p class="text-text-secondary">หากวิดีโอไม่แสดงผล</p><a href="https://www.youtube.com/watch?v=${song.video_id}" target="_blank" rel="noopener noreferrer" class="inline-block mt-1 px-4 py-2 bg-red-600 text-white font-semibold rounded-lg hover:bg-red-700 transition">คลิกเพื่อดูบน YouTube</a></div>`;
        bubbleContent.innerHTML = answerHtml + iframeHtml + fallbackLinkHtml;
        messageArea.scrollTop = messageArea.scrollHeight;
    }

    function adjustTextareaHeight(el) {
        el.style.height = 'auto';
        const newHeight = Math.min(el.scrollHeight, 160);
        el.style.height = `${newHeight}px`;
        el.style.overflowY = (el.scrollHeight > 160) ? 'auto' : 'hidden';
    }

    // --- Event Listeners (Full Code) ---
    sendButton.addEventListener('click', () => sendMessage());
    userInput.addEventListener('keydown', (e) => {
        if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); sendMessage(); }
    });
    userInput.addEventListener('input', (e) => adjustTextareaHeight(e.target));
    convoBtn.addEventListener('click', () => { window.open('robot_avatar.html', '_blank'); });

    newChatBtn.addEventListener('click', () => {
        messageArea.innerHTML = '';
        addMessage(createEmptyResponse("เริ่มต้นการสนทนาใหม่ค่ะ มีอะไรให้ช่วยเกี่ยวกับจังหวัดน่านไหมคะ?"), 'ai');
        const songInput = document.querySelector('.song-input-field');
        if(songInput) songInput.closest('div').remove();
        inputArea.style.display = 'flex';
        userInput.focus();
    });

    let audioContext = null;
    micButton.addEventListener('click', async () => {
        if (!audioContext) {
            try {
                audioContext = new (window.AudioContext || window.webkitAudioContext)();
                await audioContext.resume();
            } catch (e) {
                console.error("Could not create/resume AudioContext", e);
                addMessage(createEmptyResponse('ขออภัยค่ะ ไม่สามารถเปิดใช้งานระบบเสียงได้'), 'ai');
                return;
            }
        }
        voiceHandler.isListening ? voiceHandler.stop() : voiceHandler.start(audioContext);
    });
    
    faqButton.addEventListener('click', () => {
        const existingFaq = document.getElementById('faq-container');
        if (existingFaq) existingFaq.remove();
        const faqContainer = document.createElement('div');
        faqContainer.id = 'faq-container';
        faqContainer.className = 'flex items-start space-x-3 ai-message';
        const faqs = ["แนะนำที่เที่ยวธรรมชาติหน่อย", "วัดสวยๆ ในตัวเมืองมีที่ไหนบ้าง", "ของกินพื้นเมืองที่ต้องลองคืออะไร", "อยากได้แผนเที่ยว 1 วัน", "เปิดเพลง", "เปิดเครื่องคิดเลข","ร้านอาหารกลางคืนอร่อยๆ"];
        let buttonsHtml = faqs.map(q => `<button class="block w-full text-left p-2.5 mt-2 bg-slate-700/60 hover:bg-slate-700 rounded-lg transition">${q.replace(/</g, "&lt;")}</button>`).join('');
        faqContainer.innerHTML = `<div class="robot-icon-container"><div class="icon-head-top-accent"></div><div class="icon-robot-face"><div class="icon-robot-eye"></div><div class="icon-robot-eye"></div></div></div><div class="glass-ai-bubble p-4 rounded-2xl rounded-bl-lg max-w-md w-full"><p class="font-medium text-text-primary mb-2">ลองถามคำถามเหล่านี้ดูสิคะ:</p>${buttonsHtml}</div>`;
        faqContainer.querySelectorAll('button').forEach(btn => {
            btn.addEventListener('click', () => { sendMessage(btn.textContent); faqContainer.remove(); });
        });
        messageArea.appendChild(faqContainer);
        messageArea.scrollTop = messageArea.scrollHeight;
    });

    addMessage(createEmptyResponse("สวัสดีเจ้า... ยินดีต้อนรับสู่เมืองน่าน มีอะหยังหื้อน้องน่านช่วยก่อเจ้า?"), 'ai');
    adjustTextareaHeight(userInput);

});