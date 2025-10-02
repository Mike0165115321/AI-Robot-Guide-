// /frontend/assets/scripts/chat.js (Revised for Frontend STT & Image Display)

document.addEventListener('DOMContentLoaded', () => {
    
    // --- Element References ---
    const chatBox = document.getElementById('chat-box');
    const userInput = document.getElementById('user-input');
    const sendBtn = document.getElementById('send-btn');
    const pttBtn = document.getElementById('ptt-btn');
    const convoBtn = document.getElementById('convo-btn');

    // --- State Variables ---
    let isAnswering = false;

    const voiceHandler = new VoiceHandler({
    onStartRecording: () => {
        pttBtn.style.color = '#dc3545';
        pttBtn.textContent = '🛑';
        userInput.placeholder = "กำลังฟังอยู่ พูดได้เลย...";
        userInput.value = ''; 
    },
    onStopRecording: () => {
        pttBtn.style.color = '#555';
        pttBtn.textContent = '🎤';
        userInput.placeholder = "พิมพ์คำถามของคุณที่นี่...";
    },
    onInterimTranscript: (text) => {
        userInput.value = text;
    },
    onFinalTranscript: (text) => {
        userInput.value = text;
    },
    onError: (error) => {
        console.error(error);
        addMessage(`ขออภัยค่ะ เกิดข้อผิดพลาดกับระบบเสียง: ${error}`, 'bot');
    }
});

    async function sendMessage() {
        const query = userInput.value.trim();
        if (!query || isAnswering) return;

        addMessage(query, 'user');
        userInput.value = '';
        isAnswering = true;
        const thinkingMessage = addMessage('AI กำลังคิด...', 'bot thinking');

        try {
            const response = await fetch(`${API_BASE_URL}/api/chat/`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ query: query }),
            });

            if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
            
            const data = await response.json();
            
            thinkingMessage.remove();

            addMessage(data, 'bot');


        } catch (error) {
            console.error('Chat error:', error);
            thinkingMessage.remove();
            addMessage({ answer: 'ขออภัยค่ะ เกิดข้อผิดพลาดในการเชื่อมต่อ' }, 'bot');
        } finally {
            isAnswering = false;
        }
    }

    function addMessage(data, sender) {
    const messageElement = document.createElement('div');
    
    const classes = sender.split(' '); 
    messageElement.classList.add('message', ...classes);

    if (sender.startsWith('user')) {
        messageElement.textContent = data;
    } else {
        const answerText = typeof data === 'string' ? data : data.answer;
        const textElement = document.createElement('p');
        textElement.textContent = answerText;
        messageElement.appendChild(textElement);

        if (typeof data === 'object' && data.image_url) {
            const imageElement = document.createElement('img');
            imageElement.src = data.image_url;
            imageElement.alt = "ภาพประกอบ";
            messageElement.appendChild(imageElement);
        }
    }

    chatBox.appendChild(messageElement);
    chatBox.scrollTop = chatBox.scrollHeight;
    return messageElement;
}

    // --- Event Listeners Setup ---
    function setupEventListeners() {
        sendBtn.addEventListener('click', sendMessage);
        userInput.addEventListener('keypress', e => {
            if (e.key === 'Enter') sendMessage();
        });

        // ใช้ VoiceHandler ที่เราสร้างขึ้น
        pttBtn.addEventListener('click', () => voiceHandler.toggleRecording());

        convoBtn.addEventListener('click', () => {
            window.open('robot_avatar.html', '_blank');
        });
    }

    // --- Initialization ---
    setupEventListeners();
    addMessage({ answer: "สวัสดีครับ! มีอะไรให้ช่วยเกี่ยวกับจังหวัดน่านไหมครับ?" }, 'bot');

});