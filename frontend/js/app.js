
/**
 * # App.js - Main Application Logic (Refactored)
 * Acts as the centralized "Bootstrapper" and "Controller".
 * Delegates responsibilities to specialized modules.
 */

import { $, $$, on, delegate } from './utils/dom.js';
import chatService from './services/chatService.js';
import avatarService from './services/avatarService.js';
import alertService from './services/alertService.js';
import { renderNavbar } from './components/Navbar.js';
import { fabManager } from './components/FabManager.js';
import { voiceModeManager } from './components/VoiceModeManager.js';
import aiModeManager from './services/aiModeManager.js';
import FrontendDirector from './services/frontendDirector.js';
import * as LanguageUtils from './utils/languageUtils.js';

// 🆕 Modules
import stateManager from './modules/StateManager.js';
import uiManager from './modules/UIManager.js';
import avatarManager from './modules/AvatarManager.js';
import { renderMarkdown } from './services/markdownService.js'; // Import directly
import responseRenderer from './components/responseRenderer.js';

// ==========================================
// INIT
// ==========================================
document.addEventListener('DOMContentLoaded', () => {
    console.log('🚀 App Initializing...', stateManager.get('sessionId'));

    renderNavbar('navbar-container');
    bindEvents();
    initServices();
    loadAvatar();

    // Unlock Audio Context
    const unlockAudio = () => {
        const silentAudio = new Audio();
        silentAudio.src = 'data:audio/wav;base64,UklGRigAAABXQVZFZm10IBIAAAABAAEARKwAAIhYAQACABAgZGF0YQQAAAAAAA==';
        silentAudio.play().then(() => {
            console.log('🔓 Audio Context Unlocked');
            cleanup();
        }).catch(e => console.log('🔒 Autoplay still locked', e));
    };

    const cleanup = () => {
        document.removeEventListener('click', unlockAudio);
        document.removeEventListener('keydown', unlockAudio);
        document.removeEventListener('touchstart', unlockAudio);
    };

    document.addEventListener('click', unlockAudio);
    document.addEventListener('keydown', unlockAudio);
    document.addEventListener('touchstart', unlockAudio);
});

function initServices() {
    // 1. Connect Services
    avatarService.connect(stateManager.get('sessionId'));
    alertService.connect();

    // 2. FAB Manager
    fabManager.init({
        onSendMessage: (text) => {
            console.log('🔘 FAB onSendMessage:', text);
            const input = $('#query-input');
            if (input) input.value = text;
            handleSend(text);
        }
    });

    // 3. Voice Mode Manager
    voiceModeManager.init({
        onAudioSend: async (audioBlob) => {
            console.log('🎤 Voice Mode: Sending audio for STT...');
            const text = await chatService.transcribeAudio(audioBlob);

            if (text) {
                const input = $('#query-input');
                if (input) input.value = text;
                handleSend(text);
                return { success: true, data: { answer: 'กำลังประมวลผลค่ะ...' } };
            } else {
                return { success: false, error: 'No speech detected' };
            }
        },
        onModeChange: (mode) => {
            stateManager.set('isVoiceMode', mode === 'voice');
            stateManager.set('currentMode', mode);
        },
        showLoading: () => uiManager.showLoading(),
        hideLoading: () => uiManager.hideLoading()
    });

    // 4. Global Listeners
    avatarService.onMessage(handleAvatarMessage);
    alertService.onAlert(handleAlertMessage);

    // 5. Init UI
    uiManager.updateAIModeButton();
}

function loadAvatar() {
    const wrapper = $('#avatar-wrapper');
    if (wrapper) {
        wrapper.innerHTML = `<iframe src="/avatar/avatar_export.html?embed=true" allow="autoplay"></iframe>`;
    }
}

// ==========================================
// CONTROL LOGIC (CONTROLLER)
// ==========================================

async function handleSend(manualText = null) {
    if (stateManager.get('isProcessing')) {
        console.warn("⚠️ Already processing. Ignoring.");
        return;
    }

    const input = $('#query-input');
    const text = (typeof manualText === 'string') ? manualText : (input ? input.value.trim() : '');

    // ⛔ STRICT BLOCK: Empty Text
    if (!text || text.length === 0) {
        console.warn("⚠️ Empty text detected. Aborting.");
        return;
    }

    // 1. UI Setup
    stateManager.set('lastUserQuery', text);
    stateManager.set('isProcessing', true); // Lock
    if (input) {
        input.value = '';
        input.disabled = true;
    }

    uiManager.updateSpeech(`🗣️ ${text}`);
    uiManager.showLoading();

    // 2. Avatar Immediate State
    avatarManager.sendCommand({ type: 'pauseIdle' });
    avatarManager.setMood('thinking');

    try {
        const detectedLang = LanguageUtils.detect(text);
        console.log(`🎬 [App] Director Decision for: "${text}"`);

        const decision = await FrontendDirector.decide(text, detectedLang);

        if (decision.type === 'LOCAL') {
            uiManager.hideLoading();
            executeLocalAction(decision.action);
            return;
        }

        if (decision.type === 'GOOGLE') {
            handleBackendResponse(decision.data);
            return;
        }

        // RAG Fallback
        console.log("📚 Fallback to RAG");

        // 🗣️ Wait Message (Immediate) - Avatar will speak, then go back to thinking automatically
        avatarManager.speak("รอสักครู่นะคะ กำลังค้นข้อมูลให้ค่ะ", "thinking", detectedLang);
        uiManager.updateSpeech("กำลังค้นข้อมูล...");

        // ⏳ Progressive Feedback
        const feedbackTimer = setTimeout(() => {
            if (stateManager.get('isProcessing') && !stateManager.get('isSpeaking')) {
                // Use interrupt: false to safely append or play if idle
                avatarManager.speak("ข้อมูลเยอะนิดนึงนะคะ ขอเวลาเรียบเรียงแป๊บนึงค่ะ", "thinking", "th", null, false);
                uiManager.updateSpeech("กำลังเรียบเรียงข้อมูล... 📝");
            }
        }, 7000);

        const longWaitTimer = setTimeout(() => {
            if (stateManager.get('isProcessing') && !stateManager.get('isSpeaking')) {
                // Use interrupt: false
                avatarManager.speak("ยังหาอยู่นะคะ หัวข้อนี้ยากจัง รออีกนิดนะคะ", "worried", "th", null, false);
                uiManager.updateSpeech("ข้อมูลลึกมาก... รอสักครู่ค่ะ 😅");
            }
        }, 15000);

        const response = await chatService.sendText(text, stateManager.get('sessionId'), detectedLang, 'FAQ');

        clearTimeout(feedbackTimer);
        clearTimeout(longWaitTimer);

        if (response.success) {
            handleBackendResponse(response.data);
        } else {
            throw new Error("RAG Failed");
        }

    } catch (err) {
        console.error("❌ Error:", err);
        avatarManager.setMood('worried');
        uiManager.updateSpeech('เชื่อมต่อไม่ได้ค่ะ 🔌');
        avatarManager.sendCommand({ type: 'resumeIdle' });
    } finally {
        uiManager.hideLoading();
        stateManager.set('isProcessing', false); // Unlock
        if (input) {
            input.disabled = false;
            input.focus();
        }
        // Ensure idle resumes if not speaking
        if (!stateManager.get('isSpeaking')) {
            avatarManager.setMood('normal');
            avatarManager.sendCommand({ type: 'resumeIdle' });
        }
    }
}

function handleBackendResponse(data) {
    // 1. Speak / Mood
    if (data.answer) {
        avatarManager.speak(data.answer, data.avatar_mood || 'normal');
    } else {
        if (data.avatar_mood) avatarManager.setMood(data.avatar_mood);
    }

    // 2. Avatar Action
    if (data.avatar_action) {
        avatarManager.sendCommand({ type: 'action', action: data.avatar_action });
    } else {
        if (data.avatar_mood === 'happy' && Math.random() > 0.7) {
            avatarManager.sendCommand({ type: 'action', action: 'waveHand' });
        }
    }

    // ... (UI Panel Logic - unchanged) ...
    renderResponsePanel(data);
}

// Separate function for Panel Rendering to keep handleBackendResponse clean
function renderResponsePanel(data) {
    const isMusicChoice = (data.action === 'SHOW_SONG_CHOICES');
    const shouldShow = (data.show_slide !== false) || isMusicChoice;

    console.log('[DEBUG] renderResponsePanel:', {
        show_slide: data.show_slide,
        isMusicChoice,
        shouldShow,
        hasAnswer: !!data.answer,
        hasGallery: !!data.image_gallery
    });

    if (!shouldShow) {
        console.log('[DEBUG] Not showing panel (shouldShow=false)');
        uiManager.hidePanel();
        return;
    }

    let panelHtml = '';

    if (data.answer) {
        panelHtml += `<div class="response-text" style="font-size: 1.1rem; line-height: 1.6; color: var(--color-text); margin-bottom: 20px;">
            ${renderMarkdown(data.answer)} 
        </div>`;
    }

    if (data.image_gallery && data.image_gallery.length > 0) {
        panelHtml += responseRenderer.renderGallery(data.image_gallery);
    } else if (data.image_url) {
        const fullUrl = data.image_url.startsWith('http') ? data.image_url : data.image_url;
        panelHtml += `<div class="image-gallery"><img src="${fullUrl}" alt="รูปภาพประกอบ" loading="lazy" onclick="window.open('${fullUrl}', '_blank')"></div>`;
    }

    if (isMusicChoice && data.action_payload) {
        panelHtml += uiManager.renderMusicList(data.action_payload);
    }

    if (data.payload) {
        panelHtml += responseRenderer.render(data.payload);
    }

    // Footer
    panelHtml += `
        <div class="response-footer" style="display: flex; justify-content: flex-end; align-items: center; margin-top: 20px; padding-top: 15px; border-top: 1px solid rgba(255,255,255,0.1); gap: 10px;">
            ${data.processing_time ? `<div class="processing-time" style="font-size: 0.8rem; opacity: 0.6; margin-right: auto;">⏱️ ${data.processing_time}s</div>` : ''}
            <div class="feedback-group" style="display: flex; gap: 5px;">
                <button class="btn-img-action btn-like" onclick="window.submitFeedback('like', this)" title="ถูกใจ" style="background: rgba(255, 255, 255, 0.1); border: 1px solid rgba(255,255,255,0.2); width: 36px; height: 36px; border-radius: 6px; cursor: pointer; color: var(--color-text); display: flex; align-items: center; justify-content: center; font-size: 1.2rem;">👍</button>
                <button class="btn-img-action btn-dislike" onclick="window.submitFeedback('dislike', this)" title="ไม่ถูกใจ" style="background: rgba(255, 255, 255, 0.1); border: 1px solid rgba(255,255,255,0.2); width: 36px; height: 36px; border-radius: 6px; cursor: pointer; color: var(--color-text); display: flex; align-items: center; justify-content: center; font-size: 1.2rem;">👎</button>
            </div>
             <button class="btn-print" onclick="window.printCurrentResponse()" title="พิมพ์หน้านี้" style="background: rgba(255, 255, 255, 0.1); border: 1px solid rgba(255,255,255,0.2); padding: 5px 12px; border-radius: 6px; cursor: pointer; color: var(--color-text); font-size: 0.9rem; display: flex; align-items: center; gap: 6px;"><i class="fa-solid fa-print"></i> พิมพ์</button>
        </div>
    `;

    if (panelHtml) {
        console.log('[DEBUG] Calling uiManager.showPanel with HTML length:', panelHtml.length);
        uiManager.showPanel(panelHtml);
    } else {
        console.warn('[DEBUG] panelHtml is empty!');
    }
}

function executeLocalAction(actionName) {
    if (actionName === 'dance') {
        avatarManager.speak("ได้เลยค่ะ เดี๋ยวเต้นให้ดู!", 'happy');
        setTimeout(() => avatarManager.sendCommand({ type: 'action', action: 'dance' }), 1000);
    } else if (actionName === 'laugh') {
        avatarManager.speak("ฮ่าๆๆๆ ตลกจังเลยค่ะ", 'happy');
    } else if (actionName === 'stop') {
        stopSpeaking();
    }
}

function handleAvatarMessage(data) {
    if (data.type === 'speech_start') {
        // Optional
    }
}

function handleAlertMessage(data) {
    console.log('🚨 Alert Received:', data);
    if (data.type === 'connection_established') {
        if (data.recent_alerts && data.recent_alerts.length > 0) {
            // Show toasts
            data.recent_alerts.forEach(alert => uiManager.showToastAlert(alert));

            // 🗣️ FIX: Speak for recent alerts (Connection History)
            // Just speak the LATEST one to avoid spamming 10 sentences.
            const latestAlert = data.recent_alerts[data.recent_alerts.length - 1]; // Or [0]? depending on sort. usually [0] is oldest?
            // Assuming array is standard push, last is latest? Or backend sorts?
            // Let's assume user wants to hear about the alert they see.
            // Safe bet: Speak generic + latest summary
            avatarManager.speak(`มีแจ้งเตือนค้างอยู่ค่ะ ${latestAlert.summary}`, 'worried');
        }
    } else if (data.type === 'alert') {
        uiManager.showToastAlert(data);
        console.log('🚨 Speaking Alert:', data.summary);

        // 🗣️ FIX: Use AvatarManager to speak the alert clearly
        // Force 'worried' mood for visual impact
        avatarManager.speak(`มีแจ้งเตือนด่วนค่ะ! ${data.summary}`, 'worried', 'th');
        uiManager.updateSpeech(`⚠️ แจ้งเตือน: ${data.summary}`);
    }
}

// ==========================================
// BINDINGS & HELPERS
// ==========================================

function bindEvents() {
    const btnSend = $('#btn-send');
    const input = $('#query-input');

    if (btnSend) on(btnSend, 'click', () => handleSend());
    if (input) on(input, 'keypress', (e) => {
        if (e.key === 'Enter') handleSend();
    });

    const btnMode = $('#btn-ai-mode');
    if (btnMode) on(btnMode, 'click', () => {
        aiModeManager.toggle();
        uiManager.updateAIModeButton();
    });

    const panelClose = $('#panel-close');
    if (panelClose) on(panelClose, 'click', () => uiManager.hidePanel());

    // Skin Selector
    const skinToggle = $('#skin-toggle');
    const skinCarousel = $('#skin-carousel');
    if (skinToggle) on(skinToggle, 'click', () => {
        skinCarousel.classList.toggle('open');
        skinToggle.classList.toggle('active');
    });

    delegate(document.body, 'click', '.skin-btn', (e, target) => {
        // UI Active Class
        $$('.skin-btn').forEach(b => b.classList.remove('active'));
        target.classList.add('active');

        // Delegate to Manager
        avatarManager.changeSkin(target.dataset.skin);
        uiManager.updateSpeech(`เปลี่ยนเป็น ${target.title} แล้วค่ะ! ✨`);
    });

    delegate(document.body, 'click', '.play-song-btn', (e, target) => {
        uiManager.playYouTubeVideo(target.dataset.videoId, target.dataset.title);
    });
    delegate(document.body, 'click', '.close-panel-btn', () => uiManager.hidePanel());
}

//     if (wrapper && wrapper.contentWindow) {
//         wrapper.contentWindow.postMessage({ type: 'changeSkin', skin: skinName }, '*');
//     }
//     uiManager.updateSpeech(`เปลี่ยนเป็น ${btn.title} แล้วค่ะ! ✨`);
// }

// ==========================================
// AUDIO / TTS LOGIC (Remaining Logic)
// ==========================================
// const audioPlayer = new Audio();
// let audioQueue = [];
// let isPlayingQueue = false;
// let currentSpeechId = 0;

// function speakText(text, mood = 'normal', lang = null) {
//     if (!text) return;

//     // If text is short (< 80 chars), show it in bubble too
//     if (text.length < 80) uiManager.updateSpeech(text);

//     // Reset Queue
//     audioQueue = [];
//     isPlayingQueue = false;
//     currentSpeechId++;
//     audioPlayer.pause();

//     setAvatarMood(mood);

//     const sentences = text.match(/[^.!?\s]+[.!?]*/g) || [text];

//     sentences.forEach(s => {
//         audioQueue.push({ text: s, lang: lang || 'th', mood: mood });
//     });

//     processAudioQueue();
// }

// async function processAudioQueue() {
//     if (isPlayingQueue || audioQueue.length === 0) return;
//     isPlayingQueue = true;

//     const item = audioQueue.shift();
//     try {
//         const blob = await fetchTTSBlob(item.text, item.lang);
//         const url = URL.createObjectURL(blob);
//         audioPlayer.src = url;

//         audioPlayer.onended = () => {
//             isPlayingQueue = false;
//             processAudioQueue();
//         };

//         audioPlayer.play();
//         setAvatarMood('speaking');

//     } catch (e) {
//         console.error("TTS Error:", e);
//         isPlayingQueue = false;
//     }
// }

// async function fetchTTSBlob(text, lang) {
//     const response = await fetch('/api/chat/tts', {
//         method: 'POST',
//         headers: { 'Content-Type': 'application/json' },
//         body: JSON.stringify({ text, language: lang })
//     });
//     return await response.blob();
// }

function stopSpeaking() {
    avatarManager.stop();
    uiManager.hideLoading();
    if (stateManager.get('isVoiceMode')) {
        voiceModeManager.resumeRecording();
    }
}

// Expose Global Feedback
window.submitFeedback = async (type, btn) => {
    const query = stateManager.get('lastUserQuery') || "unknown";
    const sessionId = stateManager.get('sessionId');

    try {
        await fetch('/api/analytics/submit_feedback', {
            method: 'POST',
            body: JSON.stringify({ session_id: sessionId, query, feedback_type: type })
        });
        console.log(`Feedback: ${type}`);
        if (btn) btn.disabled = true;
    } catch (e) { console.error(e); }
};


