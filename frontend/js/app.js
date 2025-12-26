/**
 * # App.js - Entry Point
 * จุดเริ่มต้นของ Application
 */

import { CONFIG } from './config.js';
import { $, $$, on, delegate } from './utils/dom.js';

// ==========================================
// APP STATE
// ==========================================
const state = {
    messages: [],
    isLoading: false,
    currentMood: 'normal'
};

// ==========================================
// INIT
// ==========================================
document.addEventListener('DOMContentLoaded', () => {
    console.log('🚀 App initializing...');

    init();
});

function init() {
    // Bind events
    bindEvents();

    // Add welcome message
    addMessage('สวัสดีค่ะ! ดิฉันชื่อน้องน่าน 🤖 ยินดีให้บริการข้อมูลท่องเที่ยวจังหวัดน่านค่ะ', 'bot');

    console.log('✅ App ready!');
}

// ==========================================
// EVENT BINDINGS
// ==========================================
function bindEvents() {
    // Send button
    const btnSend = $('#btn-send');
    if (btnSend) {
        on(btnSend, 'click', handleSend);
    }

    // Enter key in input
    const chatInput = $('#chat-input');
    if (chatInput) {
        on(chatInput, 'keypress', (e) => {
            if (e.key === 'Enter') {
                handleSend();
            }
        });
    }

    // Voice button
    const btnVoice = $('#btn-voice');
    if (btnVoice) {
        on(btnVoice, 'click', handleVoice);
    }

    // Dev mood buttons
    delegate($('#dev-panel'), 'click', '[data-mood]', (e, target) => {
        const mood = target.dataset.mood;
        setMood(mood);
    });
}

// ==========================================
// HANDLERS
// ==========================================
async function handleSend() {
    const input = $('#chat-input');
    const text = input.value.trim();

    if (!text) return;

    // Add user message
    addMessage(text, 'user');
    input.value = '';

    // Set thinking mood
    setMood('thinking');

    // TODO: ส่งไปยัง Backend
    // Simulate response for now
    setTimeout(() => {
        setMood('speaking');
        addMessage('ขอบคุณสำหรับคำถามค่ะ! กำลังหาข้อมูลให้นะคะ...', 'bot');

        setTimeout(() => setMood('normal'), 2000);
    }, 1500);
}

function handleVoice() {
    console.log('🎤 Voice button clicked');
    setMood('listening');

    // TODO: Implement STT
    setTimeout(() => setMood('normal'), 3000);
}

// ==========================================
// CHAT FUNCTIONS
// ==========================================
function addMessage(text, sender = 'bot') {
    const messagesContainer = $('#chat-messages');
    if (!messagesContainer) return;

    const bubble = document.createElement('div');
    bubble.className = `chat-bubble chat-bubble-${sender}`;
    bubble.textContent = text;

    messagesContainer.appendChild(bubble);

    // Auto scroll
    messagesContainer.scrollTop = messagesContainer.scrollHeight;

    // Save to state
    state.messages.push({ text, sender, time: new Date() });
}

// ==========================================
// AVATAR FUNCTIONS
// ==========================================
function setMood(mood) {
    state.currentMood = mood;
    console.log(`🎭 Mood: ${mood}`);

    // TODO: เชื่อมกับ Avatar
    // if (window.NanAvatar) {
    //     window.NanAvatar.setMood(mood);
    // }
}

// ==========================================
// EXPORTS (for debugging)
// ==========================================
window.App = {
    state,
    addMessage,
    setMood
};
