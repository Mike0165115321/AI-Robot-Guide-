
/**
 * # App.js - Main Application Logic
 */

import { $, $$, on, delegate } from './utils/dom.js';
import chatService from './services/chatService.js';
import avatarService from './services/avatarService.js';
import speechService from './services/speechService.js';
import responseRenderer from './components/responseRenderer.js';
import alertService from './services/alertService.js';
import { renderMarkdown, renderInline } from './services/markdownService.js';
import { renderNavbar } from './components/Navbar.js';
import { fabManager } from './components/FabManager.js';
import { voiceModeManager } from './components/VoiceModeManager.js';
import { getFullImageUrl } from './config.js';
import aiModeManager from './services/aiModeManager.js';

// ==========================================
// STATE
// ==========================================
const state = {
    sessionId: localStorage.getItem('session_id') || `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
    isRecording: false,
    isLoading: false,
    isSpeaking: false,
    isVoiceMode: false  // 🆕 สำหรับสลับโหมดพิมพ์/พูด
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

    // 4. Initialize Voice Mode Manager
    voiceModeManager.init({
        onAudioSend: async (audioBlob) => {
            console.log('🎤 Voice Mode: Sending audio for STT...');
            // 1. Get Text from Audio
            const text = await chatService.transcribeAudio(audioBlob);
            console.log('📝 Transcribed:', text);

            if (text) {
                // 2. Put text in input (Visual feedback)
                const input = $('#query-input');
                if (input) input.value = text;

                // 3. Trigger standard Chat Flow (Send -> AI -> TTS)
                // Determine if we need to wait or just send immediately
                // For a smooth experience, we send immediately
                handleSend(text);

                return { success: true, data: { answer: 'กำลังประมวลผลค่ะ...' } }; // Dummy success for VoiceManager
            } else {
                return { success: false, error: 'No speech detected' };
            }
        },
        onModeChange: (mode) => {
            state.isVoiceMode = (mode === 'voice');
        },
        showLoading: showLoading,
        hideLoading: hideLoading
    });
    // 5. Listeners
    avatarService.onMessage((data) => handleAvatarMessage(data));
    alertService.onAlert((data) => handleAlertMessage(data));

    // 6. Init UI State
    updateAIModeButton();
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

    // Voice Mode is now handled by VoiceModeManager (initialized in initApp)

    const btnMode = $('#btn-ai-mode');
    if (btnMode) on(btnMode, 'click', () => {
        aiModeManager.toggle();
        updateAIModeButton();
    });

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

async function handleSend(manualText = null) {
    const input = $('#query-input');
    // Use manualText if provided, otherwise read input. 
    // IMPORTANT: If manualText is meant to be sent effectively, we should use it.
    const text = (typeof manualText === 'string') ? manualText : input.value.trim();

    if (!text) return;

    // UI Updates
    input.value = ''; // Clear input (or keep it? User might want to see it? Usually clear after send)
    // If it's voice, we already put it in input in step 2. 
    // Clearing it immediately might hide it before user sees it.
    // Let's keep it cleared for standard chat behavior.
    // Or better: update speech bubble instead of input?
    // User said: "เอามาใส่ในช่องแชทอัตโนมัติ (ให้พี่เห็นเลยว่าพูดอะไรไป)"
    // So we set it, BUT normal handleSend ALWAYS clears input.
    // Maybe we set it, wait a bit, then clear? Or just let standard chat logic take over.
    // Standard chat logic clears input.
    // If we want user to see it *as history*, we need a history view.
    // Currently there is no chat history view, only the "Speech Bubble" and "Panel".
    // If we clear it, it's gone.
    // But effectively handleSend clears it.

    // Let's set the text in input just so handleSend can read it if we didn't pass it?
    // No, we passed manualText.

    updateSpeech(`🗣️ ${text}`); // Show what user said in speech bubble briefly?
    showLoading();

    // 🛑 Pause Idle Behaviors during entire thinking + speaking flow
    sendAvatarCommand({ type: 'pauseIdle' });
    state.isSpeaking = true;  // Use this flag for the whole flow

    // Trigger Avatar Thinking Mood
    setAvatarMood('thinking');

    try {
        // Send to Backend
        // 1. Send text to Chat API (triggers RAG)
        const response = await chatService.sendText(text, state.sessionId);

        // ... rest of function ...

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
            // Resume idle on error
            state.isSpeaking = false;
            sendAvatarCommand({ type: 'resumeIdle' });
            setAvatarMood('normal');
        }

    } catch (err) {
        console.error(err);
        updateSpeech('เชื่อมต่อไม่ได้ค่ะ 🔌');
        // Resume idle on error
        state.isSpeaking = false;
        sendAvatarCommand({ type: 'resumeIdle' });
        setAvatarMood('normal');
    } finally {
        hideLoading();
    }
}

function handleBackendResponse(data) {
    // data.answer = Text response
    // data.action = Special action (SHOW_SONG_CHOICES, SHOW_MAP_EMBED, etc.)
    // data.action_payload = Rich data (songs, map, etc.)
    // data.avatar_mood = Mood string (e.g. 'happy')
    // data.avatar_action = Action string (e.g. 'wave')

    // 🔊 TTS & Avatar Mood Management
    // If there is an answer, we speak it and manage mood via TTS events
    if (data.answer) {
        speakText(data.answer, data.avatar_mood || 'normal');
    } else {
        // If no text to speak, just set the mood directly
        if (data.avatar_mood) setAvatarMood(data.avatar_mood);
    }

    // Trigger Action if present (or infer from mood)
    if (data.avatar_action) {
        sendAvatarCommand({ type: 'action', action: data.avatar_action });
    } else {
        // ✨ Add some randomness/life based on mood
        if (data.avatar_mood === 'happy' && Math.random() > 0.7) {
            sendAvatarCommand({ type: 'action', action: 'waveHand' });
        } else if (data.avatar_mood === 'curious') {
            sendAvatarCommand({ type: 'action', action: 'lookAround' });
        }
    }

    // 📝 Prepare Content for Panel (The Board)
    let panelHtml = '';

    // 1. Add Text Answer to Panel
    if (data.answer) {
        panelHtml += `<div class="response-text" style="font-size: 1.1rem; line-height: 1.6; color: var(--color-text); margin-bottom: 20px;">
            ${renderMarkdown(data.answer)}
        </div>`;

        // Update Speech Bubble with Summary (Truncated)
        // User requested removal of speech bubble
        // const summary = data.answer.length > 100 ? data.answer.substring(0, 80) + '...' : data.answer;
        // updateSpeech(summary);
    }

    // 2. Add Images / Gallery
    if (data.image_gallery && data.image_gallery.length > 0) {
        const fullUrls = data.image_gallery.map(url => getFullImageUrl(url));
        panelHtml += responseRenderer.renderGallery(fullUrls);
    } else if (data.image_url) {
        const fullUrl = getFullImageUrl(data.image_url);
        panelHtml += `<div class="image-gallery"><img src="${fullUrl}" alt="รูปภาพประกอบ" loading="lazy" onclick="window.open('${fullUrl}', '_blank')"></div>`;
    }

    // 3. Add Special Actions / Payloads
    if (data.action) {
        switch (data.action) {
            case 'SHOW_SONG_CHOICES':
                if (data.action_payload) panelHtml += renderMusicList(data.action_payload);
                break;
            case 'SHOW_MAP_EMBED':
                if (data.action_payload) panelHtml += responseRenderer.renderMapEmbed(data.action_payload);
                break;
        }
    }

    if (data.payload) {
        panelHtml += responseRenderer.render(data.payload);
    }

    // 🚀 SHOW PANEL if there is content
    if (panelHtml) {
        showPanel(panelHtml);
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


// Voice Mode is now handled by VoiceModeManager.js


function handleAvatarMessage(data) {
    if (data.type === 'speech_start') {
        // updateSpeech(data.text);
    }
    // Handle other avatar events
}

function updateAIModeButton() {
    const btn = $('#btn-ai-mode');
    if (!btn) return;

    const info = aiModeManager.getModeInfo();
    btn.innerText = info.icon;
    btn.title = info.description;

    // Update Styles
    if (info.mode === 'fast') {
        btn.style.background = 'rgba(251, 191, 36, 0.2)'; // Yellow tint
        btn.style.color = '#fbbf24';
        btn.style.borderColor = '#fbbf24';
    } else {
        btn.style.background = 'rgba(139, 92, 246, 0.2)'; // Purple tint
        btn.style.color = '#a78bfa';
        btn.style.borderColor = '#a78bfa';
    }
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
    } else {
        // Fallback or do nothing since user requested removal
        console.log('Bot says:', text);
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
        speakText(`แจ้งเตือน: ${alert.summary}`, 'normal');
        updateSpeech(`⚠️ แจ้งเตือน: ${alert.summary}`);
    }
}

// ==========================================
// 🔊 AUDIO QUEUE SYSTEM (Streaming TTS)
// ==========================================
const audioPlayer = new Audio();
let audioQueue = [];
let isPlayingQueue = false;

function stopSpeaking() {
    audioQueue = [];
    isPlayingQueue = false;
    audioPlayer.pause();
    audioPlayer.currentTime = 0;
    state.isSpeaking = false;
}

async function speakText(text, finalMood = 'normal') {
    // 1. Reset/Stop previous
    stopSpeaking();

    if (!text) {
        setAvatarMood(finalMood);
        return;
    }

    // 2. Pre-processing & Split
    // Split by delimiters (.|?|!|newline) followed by space or end
    // Clean markdown symbols to avoid saying "asterisk" etc
    const cleanText = text.replace(/[*#`]/g, '');
    const chunks = cleanText.match(/[^.!?\n]+[.!?\n]+|[^.!?\n]+$/g) || [cleanText];

    console.log(`🗣️ TTS Queue: Processing ${chunks.length} chunks`);

    // 3. Setup Queue (Convert to Objects)
    audioQueue = chunks.map(t => ({ text: t.trim(), blob: null, promise: null }));
    // Filter empty chunks
    audioQueue = audioQueue.filter(item => item.text.length > 0);

    if (audioQueue.length === 0) return;

    // 4. Start Queue
    isPlayingQueue = true;
    state.isSpeaking = true;
    sendAvatarCommand({ type: 'pauseIdle' });
    showAvatarMessage('กำลังพูด...');
    voiceModeManager.pauseRecording(); // Stop VAD

    playNextChunk(finalMood);
}

async function fetchTTSBlob(text) {
    console.log('🗣️ Fetching TTS:', text.substring(0, 20) + '...');
    const response = await fetch('/api/chat/tts', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text: text })
    });
    if (!response.ok) throw new Error(`TTS Error: ${response.status}`);
    return await response.blob();
}

async function playNextChunk(finalMood) {
    // Check if stopped or empty
    if (!isPlayingQueue || audioQueue.length === 0) {
        finishSpeaking(finalMood);
        return;
    }

    const currentItem = audioQueue[0]; // Peek

    try {
        // 1. Get Blob (Fetch if not ready)
        let blob = currentItem.blob;
        if (!blob) {
            // If promise exists, wait for it, else create new
            if (!currentItem.promise) {
                currentItem.promise = fetchTTSBlob(currentItem.text);
            }
            blob = await currentItem.promise;
            currentItem.blob = blob;
        }

        // 2. Play
        const url = URL.createObjectURL(blob);
        audioPlayer.src = url;

        audioPlayer.onplay = () => {
            setAvatarMood('speaking');
            hideAvatarMessage();

            // 🚀 PRE-FETCH NEXT CHUNK
            if (audioQueue.length > 1) {
                const nextItem = audioQueue[1];
                if (!nextItem.promise && !nextItem.blob) {
                    console.log('🚀 Pre-fetching next chunk...');
                    nextItem.promise = fetchTTSBlob(nextItem.text); // Start fetch
                }
            }
        };

        audioPlayer.onended = () => {
            URL.revokeObjectURL(url);
            audioQueue.shift(); // Remove finished item
            playNextChunk(finalMood); // Next!
        };

        audioPlayer.onerror = (e) => {
            console.error("❌ Audio Playback Error", e);
            audioQueue.shift(); // Skip bad chunk
            playNextChunk(finalMood);
        };

        await audioPlayer.play();

    } catch (e) {
        console.error("❌ TTS Processing Error:", e);
        audioQueue.shift(); // Skip bad chunk
        playNextChunk(finalMood);
    }
}

function finishSpeaking(finalMood) {
    console.log('✅ TTS Queue Finished');
    isPlayingQueue = false;
    state.isSpeaking = false;
    hideAvatarMessage();
    sendAvatarCommand({ type: 'resumeIdle' });
    setAvatarMood(finalMood);
    voiceModeManager.resumeRecording(); // Resume VAD
}

function setAvatarMood(mood) {
    // Wrapper to send mood change
    sendAvatarCommand({ type: 'changeMood', mood: mood });
}

function sendAvatarCommand(command) {
    const avatarFrame = document.querySelector('#avatar-wrapper iframe');
    if (avatarFrame && avatarFrame.contentWindow) {
        avatarFrame.contentWindow.postMessage(command, '*');
        console.log(`📤 [Avatar] Sent command:`, command);
    }
}

// ==========================================
// AVATAR MESSAGE BUBBLE (above avatar head)
// ==========================================
function showAvatarMessage(text) {
    let bubble = document.getElementById('avatar-loading-message');
    if (!bubble) {
        bubble = document.createElement('div');
        bubble.id = 'avatar-loading-message';
        bubble.style.cssText = `
            position: absolute;
            top: 10px;
            left: 50%;
            transform: translateX(-50%);
            background: rgba(45, 212, 191, 0.9);
            color: #0f172a;
            padding: 8px 16px;
            border-radius: 20px;
            font-size: 0.9rem;
            font-weight: 500;
            z-index: 1000;
            animation: pulse 1.5s ease-in-out infinite;
            box-shadow: 0 4px 15px rgba(45, 212, 191, 0.3);
        `;
        const wrapper = document.querySelector('#avatar-wrapper');
        if (wrapper) {
            wrapper.style.position = 'relative';
            wrapper.appendChild(bubble);
        }
    }
    bubble.textContent = text;
    bubble.style.display = 'block';
}

function hideAvatarMessage() {
    const bubble = document.getElementById('avatar-loading-message');
    if (bubble) {
        bubble.style.display = 'none';
    }
}
