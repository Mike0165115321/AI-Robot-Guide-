
/**
 * # App.js - Main Application Logic
 */

import { $, $$, on, delegate } from './utils/dom.js';
import chatService from './services/chatService.js';
import avatarService from './services/avatarService.js';
import speechService from './services/speechService.js';
import responseRenderer from './components/responseRenderer.js';
import alertService from './services/alertService.js';
import { renderNavbar } from './components/Navbar.js'; // Import component

// ==========================================
// STATE
// ==========================================
const state = {
    sessionId: localStorage.getItem('session_id') || `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
    isRecording: false,
    isLoading: false
};

// Save session ID
localStorage.setItem('session_id', state.sessionId);

// ==========================================
// INIT
// ==========================================
document.addEventListener('DOMContentLoaded', () => {
    console.log('🚀 App Initializing...', state.sessionId);

    renderNavbar('navbar-container'); // Render Navbar (Auto-detect Active)
    bindEvents();
    initServices();
    loadAvatar();
});

function initServices() {
    // 1. Connect Avatar WebSocket
    avatarService.connect(state.sessionId);

    // 2. Connect Alert WebSocket
    alertService.connect();

    // 3. Listengers
    avatarService.onMessage((data) => handleAvatarMessage(data));
    alertService.onAlert((data) => handleAlertMessage(data));
}

function loadAvatar() {
    const wrapper = $('#avatar-wrapper');
    if (wrapper) {
        wrapper.innerHTML = `<iframe src="/avatar/avatar_export.html?embed=true" allow="autoplay"></iframe>`;
    }
}

// ==========================================
// UI HANDLERS
// ==========================================
function bindEvents() {
    // Chat Input & Send
    const btnSend = $('#btn-send');
    const input = $('#query-input');

    if (btnSend) on(btnSend, 'click', handleSend);
    if (input) on(input, 'keypress', (e) => {
        if (e.key === 'Enter') handleSend();
    });

    const btnVoice = $('#btn-voice');
    if (btnVoice) on(btnVoice, 'click', handleVoice);

    // Clean input on focus
    if (input) on(input, 'focus', () => {
        // Optional: clear placeholder or style
    });

    // Panel Close
    const panelClose = $('#panel-close');
    if (panelClose) on(panelClose, 'click', hidePanel);

    // Skin Selector
    const skinToggle = $('#skin-toggle');
    const skinCarousel = $('#skin-carousel');
    if (skinToggle) on(skinToggle, 'click', () => {
        skinCarousel.classList.toggle('open');
        skinToggle.classList.toggle('active');
    });

    delegate(document.body, 'click', '.skin-btn', (e, target) => {
        changeSkin(target);
    });
}

async function handleSend() {
    const input = $('#query-input');
    const text = input.value.trim();
    if (!text) return;

    // UI Updates
    input.value = '';
    updateSpeech('กำลังคิดคำตอบ... 🤔');
    showLoading();

    try {
        // Send to Backend
        // 1. Send text to Chat API (triggers RAG)
        const response = await chatService.sendText(text, state.sessionId);

        // Note: The response comes back via HTTP (for text/cards) OR WebSocket (for avatar speech)
        // Usually, the Chat API returns the text answer and data cards immediately.
        // The Avatar speech might be triggered via WebSocket if configured, or we use the text response to TTS locally.
        // Assuming Backend Architecture:
        // - POST /chat/text returns { answer: "...", payload: { ... } }
        // - AND it might push to Avatar WS.

        if (response.success) {
            handleBackendResponse(response.data);
        } else {
            updateSpeech('ขอโทษค่ะ เกิดข้อผิดพลาด 😓');
            console.error(response.error);
        }

    } catch (err) {
        console.error(err);
        updateSpeech('เชื่อมต่อไม่ได้ค่ะ 🔌');
    } finally {
        hideLoading();
    }
}

function handleBackendResponse(data) {
    // data.answer = Text response
    // data.payload = Rich data (cards, location, etc.)

    if (data.answer) {
        updateSpeech(data.answer); // Basic text update
        // The avatar will speak it if connected via WS or if we synthesize it here.
        // Let's assume Avatar WS handles the speech/animation syncing.
        // If not, we might need to send text to avatar iframe.
    }

    // Show Rich Content (Cards)
    if (data.payload) {
        renderPayload(data.payload);
    }
}

async function handleVoice() {
    const btnVoice = $('#btn-voice');

    if (!state.isRecording) {
        // Start Recording
        const success = await speechService.startRecording();
        if (success) {
            state.isRecording = true;
            btnVoice.classList.add('recording'); // CSS class for red pulse or similar
            updateSpeech('กำลังฟังค่ะ... 👂');
        } else {
            updateSpeech('เปิดไมค์ไม่ได้ค่ะ 🎤❌');
        }
    } else {
        // Stop Recording
        state.isRecording = false;
        btnVoice.classList.remove('recording');
        updateSpeech('กำลังประมวลผลเสียง... 🔄');
        showLoading();

        try {
            const audioBlob = await speechService.stopRecording();
            if (audioBlob) {
                // Send to Audio API
                const response = await chatService.sendAudio(audioBlob, state.sessionId);

                if (response.success) {
                    handleBackendResponse(response.data);
                } else {
                    updateSpeech('ขอโทษค่ะ ฟังไม่ทัน 😓');
                }
            }
        } catch (err) {
            console.error(err);
            updateSpeech('มีปัญหาในการส่งเสียงค่ะ 🔌');
        } finally {
            hideLoading();
        }
    }
}

function handleAvatarMessage(data) {
    if (data.type === 'speech_start') {
        // updateSpeech(data.text);
    }
    // Handle other avatar events
}

// ==========================================
// UI HELPERS
// ==========================================
function updateSpeech(text) {
    const speechText = $('#speech-text');
    if (speechText) {
        speechText.textContent = text;
        // Simple fade effect
        speechText.style.opacity = 0;
        setTimeout(() => speechText.style.opacity = 1, 50);
    }
}

function showLoading() {
    // Add loading indicator class
}

function hideLoading() {
    // Remove loading indicator
}

function showPanel(htmlContent) {
    const panel = $('#presentation-panel');
    const panelContent = $('#presentation-content');
    const avatarSection = $('#avatar-section');

    if (panel && panelContent) {
        panelContent.innerHTML = htmlContent;
        panel.classList.add('visible');
        if (avatarSection) {
            avatarSection.classList.remove('centered');
            avatarSection.classList.add('shifted');
        }
    }
}

function hidePanel() {
    const panel = $('#presentation-panel');
    const avatarSection = $('#avatar-section');

    if (panel) panel.classList.remove('visible');
    if (avatarSection) {
        avatarSection.classList.remove('shifted');
        avatarSection.classList.add('centered');
    }
    updateSpeech('มีอะไรให้ช่วยอีกไหมคะ? 😊');
}

function changeSkin(btn) {
    const skinName = btn.dataset.skin;
    if (!skinName) return;

    $$('.skin-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');

    const avatarFrame = document.querySelector('#avatar-wrapper iframe');
    if (avatarFrame && avatarFrame.contentWindow) {
        avatarFrame.contentWindow.postMessage({ type: 'changeSkin', skin: skinName }, '*');
    }
    updateSpeech(`เปลี่ยนเป็น ${btn.title} แล้วค่ะ! ✨`);
}

function renderPayload(payload) {
    const html = responseRenderer.render(payload);
    if (html) showPanel(html);
}

function handleAlertMessage(data) {
    console.log('🚨 Alert Received:', data);

    if (data.type === 'connection_established') {
        // Show recent alerts if any
        if (data.recent_alerts && data.recent_alerts.length > 0) {
            data.recent_alerts.forEach(alert => showToastAlert(alert));
        }
    } else if (data.type === 'alert') {
        showToastAlert(data);
    }
}

function showToastAlert(alert) {
    // Create a toast notification
    const toast = document.createElement('div');
    toast.className = 'response-card alert-toast';
    toast.style.cssText = `
        position: fixed;
        top: 20px;
        right: 20px;
        z-index: 1000;
        width: 300px;
        background: rgba(220, 38, 38, 0.95);
        border: 1px solid #fecaca;
        color: white;
        box-shadow: 0 4px 12px rgba(0,0,0,0.5);
        animation: slide-up-fade 0.5s ease-out;
        cursor: pointer; /* Change cursor to pointer */
        transition: transform 0.2s;
    `;

    // Add hover effect
    toast.onmouseover = () => toast.style.transform = 'scale(1.02)';
    toast.onmouseout = () => toast.style.transform = 'scale(1)';

    toast.innerHTML = `
        <div style="display:flex;justify-content:space-between;">
            <strong>⚠️ แจ้งเตือนด่วน</strong>
            <span class="close-btn" style="cursor:pointer;padding:0 5px;">✕</span>
        </div>
        <div style="margin-top:5px;font-size:0.9rem;">${alert.summary}</div>
        <div style="margin-top:5px;font-size:0.8rem;opacity:0.8;">${alert.location_name || ''}</div>
        <div style="margin-top:8px;font-size:0.75rem;text-align:right;text-decoration:underline;opacity:0.9;">
            คลิกเพื่อดูรายละเอียด >
        </div>
    `;

    document.body.appendChild(toast);

    // Click Handler for Details
    toast.onclick = (e) => {
        // Don't trigger if close button was clicked
        if (e.target.classList.contains('close-btn') || e.target.innerText === '✕') return;

        // Show Full Details in Panel
        const html = responseRenderer.renderAlert(alert);
        showPanel(html);

        // Optional: remove toast after clicking? Or keep it?
        // Let's keep it but maybe hide it if user wants to focus on panel.
        // toast.remove(); 
    };

    // Close Button Handler
    const closeBtn = toast.querySelector('.close-btn');
    if (closeBtn) {
        closeBtn.onclick = (e) => {
            e.stopPropagation(); // Prevent opening details
            toast.remove();
        };
    }

    // Auto remove after 15 seconds (increased time)
    setTimeout(() => {
        if (toast.parentElement) toast.remove();
    }, 15000);

    // Also, if severe, speak it?
    if (alert.severity_score >= 4) {
        updateSpeech(`⚠️ แจ้งเตือน: ${alert.summary}`);
    }
}
