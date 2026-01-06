
/**
 * # App.js - Main Application Logic
 */

import { $, $$, on, delegate } from './utils/dom.js';
import chatService from './services/chatService.js';
import avatarService from './services/avatarService.js';
import speechService from './services/speechService.js';
import responseRenderer from './components/responseRenderer.js';
import alertService from './services/alertService.js';
import { renderInline } from './services/markdownService.js';
import { renderNavbar } from './components/Navbar.js';
import { fabManager } from './components/FabManager.js';
import { getFullImageUrl } from './config.js';

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

    // 3. Initialize FAB Manager
    fabManager.init({
        onSendMessage: (text) => {
            // ส่งข้อความจาก FAB widgets ไปที่ chat
            const input = $('#query-input');
            if (input) input.value = text;
            handleSend();
        }
    });

    // 4. Listeners
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

    // Music play button
    delegate(document.body, 'click', '.play-song-btn', (e, target) => {
        const videoId = target.dataset.videoId;
        const title = target.dataset.title;
        if (videoId) {
            playYouTubeVideo(videoId, title);
        }
    });

    // Close panel button
    delegate(document.body, 'click', '.close-panel-btn', () => {
        hidePanel();
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
    // data.action = Special action (SHOW_SONG_CHOICES, SHOW_MAP_EMBED, etc.)
    // data.action_payload = Rich data (songs, map, etc.)

    if (data.answer) {
        updateSpeech(data.answer); // Basic text update
    }

    // Handle special actions
    if (data.action) {
        switch (data.action) {
            case 'SHOW_SONG_CHOICES':
                // แสดงรายการเพลงให้เลือก
                if (data.action_payload && data.action_payload.length > 0) {
                    const musicHtml = renderMusicList(data.action_payload);
                    showPanel(musicHtml);
                }
                break;

            case 'SHOW_MAP_EMBED':
                // แสดงแผนที่
                if (data.action_payload) {
                    const mapHtml = responseRenderer.renderMapEmbed(data.action_payload);
                    showPanel(mapHtml);
                }
                break;

            case 'PROMPT_FOR_SONG_INPUT':
                // ถามว่าอยากฟังเพลงอะไร - ไม่ต้องทำอะไรเพิ่ม, ข้อความแสดงแล้ว
                break;

            default:
                console.log('Unknown action:', data.action);
        }
    }

    // Show image gallery if available
    if (data.image_gallery && data.image_gallery.length > 0) {
        // แปลง image URLs ให้เป็น full URL ก่อนแสดงผล
        const fullUrls = data.image_gallery.map(url => getFullImageUrl(url));
        const galleryHtml = responseRenderer.renderGallery(fullUrls);
        showPanel(galleryHtml);
    }

    // Original payload handling
    if (data.payload) {
        renderPayload(data.payload);
    }
}

/**
 * Render music list from search results
 */
function renderMusicList(songs) {
    return `
        <div class="music-results">
            <h3 style="margin-bottom: 15px;">🎵 เลือกเพลงที่ต้องการฟัง</h3>
            <div style="display: flex; flex-direction: column; gap: 10px;">
                ${songs.slice(0, 5).map((song, idx) => `
                    <div class="song-item" style="display: flex; gap: 12px; background: rgba(255,255,255,0.05); padding: 10px; border-radius: 10px; align-items: center;">
                        <img src="${song.thumbnail}" alt="${song.title}" style="width: 80px; height: 60px; object-fit: cover; border-radius: 6px;">
                        <div style="flex: 1; min-width: 0;">
                            <div style="font-weight: bold; white-space: nowrap; overflow: hidden; text-overflow: ellipsis;">${song.title}</div>
                            <div style="font-size: 0.8rem; opacity: 0.7;">${song.duration || ''}</div>
                        </div>
                        <button class="play-song-btn" data-video-id="${song.video_id}" data-title="${song.title}" 
                            style="background: #1db954; border: none; border-radius: 50%; width: 40px; height: 40px; cursor: pointer; color: white; font-size: 1rem;">
                            ▶
                        </button>
                    </div>
                `).join('')}
            </div>
            <button class="close-panel-btn" style="margin-top: 15px; background: none; border: 1px solid rgba(255,255,255,0.2); padding: 8px 20px; border-radius: 20px; color: white; cursor: pointer;">
                ปิด
            </button>
        </div>
    `;
}

async function handleVoice() {
    const btnVoice = $('#btn-voice');

    if (!state.isRecording) {
        // เริ่ม VAD Mode (ตรวจจับเสียงอัตโนมัติ)
        const success = await speechService.startVAD({
            onStatusUpdate: (status) => {
                updateSpeech(status);
            },
            onSpeechEnd: async (audioBlob) => {
                // เมื่อพูดจบ (ตรวจจับ silence)
                state.isRecording = false;
                btnVoice.classList.remove('recording');
                updateSpeech('กำลังประมวลผลเสียง... 🔄');
                showLoading();

                try {
                    const response = await chatService.sendAudio(audioBlob, state.sessionId);
                    if (response.success) {
                        handleBackendResponse(response.data);
                    } else {
                        updateSpeech('ขอโทษค่ะ ฟังไม่ทัน 😓');
                    }
                } catch (err) {
                    console.error(err);
                    updateSpeech('มีปัญหาในการส่งเสียงค่ะ 🔌');
                } finally {
                    hideLoading();
                }
            },
            onVolumeChange: (volume) => {
                // Optional: แสดง volume indicator
                // updateVolumeBar(volume);
            }
        });

        if (success) {
            state.isRecording = true;
            btnVoice.classList.add('recording');
            updateSpeech('กำลังฟังค่ะ... 👂');
        } else {
            updateSpeech('เปิดไมค์ไม่ได้ค่ะ 🎤❌');
        }
    } else {
        // หยุด VAD
        speechService.stopVAD(true); // interrupted = true
        state.isRecording = false;
        btnVoice.classList.remove('recording');
        updateSpeech('หยุดฟังแล้วค่ะ');
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
        // ใช้ Markdown renderer สำหรับแสดงข้อความจาก AI
        speechText.innerHTML = renderInline(text);
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

/**
 * Play YouTube video in panel
 */
function playYouTubeVideo(videoId, title) {
    const playerHtml = `
        <div class="music-player">
            <h3 style="margin-bottom: 15px;">🎵 กำลังเล่น</h3>
            <p style="margin-bottom: 10px; opacity: 0.8; font-size: 0.9rem;">${title || 'เพลง'}</p>
            <div class="video-wrapper" style="position: relative; padding-bottom: 56.25%; height: 0; border-radius: 12px; overflow: hidden;">
                <iframe 
                    style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"
                    src="https://www.youtube.com/embed/${videoId}?autoplay=1&rel=0" 
                    frameborder="0" 
                    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" 
                    allowfullscreen>
                </iframe>
            </div>
            <button class="close-panel-btn" style="margin-top: 15px; background: none; border: 1px solid rgba(255,255,255,0.2); padding: 8px 20px; border-radius: 20px; color: white; cursor: pointer;">
                ปิด
            </button>
        </div>
    `;
    showPanel(playerHtml);
    updateSpeech(`กำลังเล่นเพลงให้ฟังค่ะ 🎧`);
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
