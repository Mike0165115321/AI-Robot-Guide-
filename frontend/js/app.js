
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
import { languageManager } from './modules/LanguageManager.js';
import * as LanguageUtils from './utils/languageUtils.js';

// 🆕 Modules
import stateManager from './modules/StateManager.js';
import uiManager from './modules/UIManager.js';
import avatarManager from './modules/AvatarManager.js';
import idlePrompter from './modules/IdlePrompter.js';
import { renderMarkdown } from './services/markdownService.js';
import responseRenderer from './components/responseRenderer.js';
import { quickScripts } from './data/scripts.js';
// import { wakeWordService } from './services/WakeWordService.js';

// ==========================================
// INIT
// ==========================================
document.addEventListener('DOMContentLoaded', () => {
    console.log('🚀 App Initializing...', stateManager.get('sessionId'));

    renderNavbar('navbar-container');
    bindEvents();
    initServices();
    loadAvatar();

    // 🌍 Init I18N
    updateStaticText(languageManager.getCurrentLanguage()); // Initial
    renderQuickScripts(); // 🆕 Render Quick Scripts

    languageManager.subscribe((lang) => {
        updateStaticText(lang);
        renderQuickScripts(); // 🆕 Re-render on language change
    });

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

    // 🆕 Start IdlePrompter (พูดชวนกดปุ่มทุก 15-30 วิ)
    idlePrompter.start();

    // 🎤 Start Wake Word Detection (Disabled for stability as requested)
    // initWakeWordService();
});

function updateStaticText(lang) {
    document.querySelectorAll('[data-i18n]').forEach(el => {
        const key = el.getAttribute('data-i18n');
        const text = languageManager.getText(key);

        if (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA') {
            el.placeholder = text;
        } else {
            el.innerText = text;
        }
    });
}

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

    // Sync Stop Button
    avatarManager.onAudioStateChange = (isSpeaking) => {
        const btnStop = $('#btn-stop-tts');
        if (btnStop) {
            btnStop.style.display = isSpeaking ? 'inline-flex' : 'none';
        }
    };

    // 5. Init UI
    uiManager.updateAIModeButton();
}

// ==========================================
// WAKE WORD SERVICE (DISABLED)
// ==========================================

/**
 * เริ่ม Wake Word Detection
 * ฟังอยู่เบื้องหลัง เมื่อได้ยิน "สวัสดีน้องน่าน" จะเปิดใช้งาน
 */
/*
function initWakeWordService() {
    // ตรวจสอบว่า browser รองรับหรือไม่
    if (wakeWordService.getStatus() === 'unsupported') {
        console.warn('⚠️ Wake Word not supported in this browser');
        return;
    }

    // Callback เมื่อตรวจพบ wake word
    wakeWordService.on('detected', async (transcript) => {
        console.log('✨ Wake Word Detected:', transcript);

        // หยุด IdlePrompter ชั่วคราว
        idlePrompter.pause();

        // น้องน่านทักทายกลับ
        const greeting = 'สวัสดีค่ะ มีอะไรให้น้องน่านช่วยไหมคะ?';

        // TTS ทักทาย และรอให้พูดจบ
        await avatarManager.speak(greeting);

        // เปิด STT mode รอฟังคำถาม
        console.log('🎤 Entering STT mode...');
        voiceModeManager.activateVoiceMode();

        // Resume wake word หลัง STT จบ (จัดการใน voiceModeManager)
    });

    // Callback เมื่อสถานะเปลี่ยน
    wakeWordService.on('status', (status) => {
        console.log(`🎤 Wake Word Status: ${status}`);
        updateWakeWordIndicator(status);
    });

    // Callback เมื่อเกิด error
    wakeWordService.on('error', (error) => {
        console.error('❌ Wake Word Error:', error);
        if (error === 'microphone_denied') {
            uiManager.showToast('กรุณาอนุญาตใช้ไมโครโฟนเพื่อเปิดใช้ Wake Word');
        }
    });

    // เริ่มฟัง (หลังจาก user interact กับหน้าเว็บแล้ว)
    document.addEventListener('click', startWakeWordOnce, { once: true });
    document.addEventListener('touchstart', startWakeWordOnce, { once: true });
}

function startWakeWordOnce() {
    if (wakeWordService.getStatus() === 'stopped') {
        console.log('🎤 Starting Wake Word Service...');
        wakeWordService.start();
    }
}

function updateWakeWordIndicator(status) {
    let indicator = $('#wake-word-indicator');

    // สร้าง indicator ถ้ายังไม่มี
    if (!indicator) {
        indicator = document.createElement('div');
        indicator.id = 'wake-word-indicator';
        indicator.className = 'wake-word-indicator';
        document.body.appendChild(indicator);
    }

    // อัปเดตสถานะ
    indicator.className = `wake-word-indicator wake-word-${status}`;

    switch (status) {
        case 'listening':
            indicator.innerHTML = '🎤 กำลังฟัง "น้องน่าน"...';
            indicator.style.display = 'block';
            break;
        case 'paused':
            indicator.innerHTML = '⏸️ หยุดชั่วคราว';
            break;
        case 'stopped':
            indicator.style.display = 'none';
            break;
        default:
            indicator.style.display = 'none';
    }
}
*/

// 🆕 Export function to pause/resume wake word from other modules
window.pauseWakeWord = () => { /* wakeWordService.pause(); */ };
window.resumeWakeWord = () => {
    // wakeWordService.resume();
    idlePrompter.resume();
};

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
        const interfaceLang = languageManager.getCurrentLanguage(); // 🌍 Use Interface Language for system feedback

        console.log(`🎬 [App] Director Decision for: "${text}" (Detected: ${detectedLang}, Interface: ${interfaceLang})`);

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

        // 🗣️ Wait Message (Immediate) - Speak in INTERFACE Language
        avatarManager.speak(languageManager.getText('chat_wait_msg') || "รอสักครู่นะคะ กำลังค้นข้อมูลให้ค่ะ", "thinking", interfaceLang);
        uiManager.updateSpeech(languageManager.getText('chat_searching') || "กำลังค้นข้อมูล...");

        // ⏳ Progressive Feedback
        const feedbackTimer = setTimeout(() => {
            if (stateManager.get('isProcessing') && !stateManager.get('isSpeaking')) {
                avatarManager.speak(languageManager.getText('chat_long_wait') || "ข้อมูลเยอะนิดนึงนะคะ ขอเวลาเรียบเรียงแป๊บนึงค่ะ", "thinking", interfaceLang, null, false);
                uiManager.updateSpeech("กำลังเรียบเรียงข้อมูล... 📝");
            }
        }, 7000);

        const longWaitTimer = setTimeout(() => {
            if (stateManager.get('isProcessing') && !stateManager.get('isSpeaking')) {
                avatarManager.speak(languageManager.getText('chat_very_long_wait') || "ยังหาอยู่นะคะ หัวข้อนี้ยากจัง รออีกนิดนะคะ", "worried", interfaceLang, null, false);
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

        // Reset IdlePrompter timer (user interacted)
        idlePrompter.reset();
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

    renderResponsePanel(data);
}

function renderResponsePanel(data) {
    const isMusicChoice = (data.action === 'SHOW_SONG_CHOICES');
    const shouldShow = (data.show_slide !== false) || isMusicChoice;

    if (!shouldShow) {
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

    // 🆕 Fix: Handle Map Embed (Previously missing!)
    if (data.action === 'SHOW_MAP_EMBED' && data.action_payload) {
        console.log("🗺️ Rendering Map Embed:", data.action_payload);
        panelHtml += responseRenderer.renderMapEmbed(data.action_payload);
    }

    if (data.payload) {
        panelHtml += responseRenderer.render(data.payload);
    }

    // Footer
    const txtLike = languageManager.getText('btn_like') || 'Like';
    const txtDislike = languageManager.getText('btn_dislike') || 'Dislike';
    const txtPrint = languageManager.getText('btn_print') || 'Print';

    panelHtml += `
        <div class="response-footer" style="display: flex; justify-content: flex-end; align-items: center; margin-top: 20px; padding-top: 15px; border-top: 1px solid rgba(255,255,255,0.1); gap: 10px;">
            ${data.processing_time ? `<div class="processing-time" style="font-size: 0.8rem; opacity: 0.6; margin-right: auto;">⏱️ ${data.processing_time}s</div>` : ''}
            <div class="feedback-group" style="display: flex; gap: 5px;">
                <button class="btn-img-action btn-like" onclick="window.submitFeedback('like', this)" title="${txtLike}" style="background: rgba(255, 255, 255, 0.1); border: 1px solid rgba(255,255,255,0.2); width: 36px; height: 36px; border-radius: 6px; cursor: pointer; color: var(--color-text); display: flex; align-items: center; justify-content: center; font-size: 1.2rem;">👍</button>
                <button class="btn-img-action btn-dislike" onclick="window.submitFeedback('dislike', this)" title="${txtDislike}" style="background: rgba(255, 255, 255, 0.1); border: 1px solid rgba(255,255,255,0.2); width: 36px; height: 36px; border-radius: 6px; cursor: pointer; color: var(--color-text); display: flex; align-items: center; justify-content: center; font-size: 1.2rem;">👎</button>
            </div>
             <button class="btn-print" onclick="window.printCurrentResponse()" title="${txtPrint}" style="background: rgba(255, 255, 255, 0.1); border: 1px solid rgba(255,255,255,0.2); padding: 5px 12px; border-radius: 6px; cursor: pointer; color: var(--color-text); font-size: 0.9rem; display: flex; align-items: center; gap: 6px;"><i class="fa-solid fa-print"></i> ${txtPrint}</button>
        </div>
    `;

    if (panelHtml) {
        uiManager.showPanel(panelHtml);
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
            data.recent_alerts.forEach(alert => uiManager.showToastAlert(alert));
            const latestAlert = data.recent_alerts[data.recent_alerts.length - 1];
            avatarManager.speak(`มีแจ้งเตือนค้างอยู่ค่ะ ${latestAlert.summary}`, 'worried');
        }
    } else if (data.type === 'alert') {
        uiManager.showToastAlert(data);
        avatarManager.speak(`มีแจ้งเตือนด่วนค่ะ! ${data.summary}`, 'worried', 'th');
        uiManager.updateSpeech(`⚠️ แจ้งเตือน: ${data.summary}`);
    }
}

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

    const btnStopTTS = $('#btn-stop-tts');
    if (btnStopTTS) on(btnStopTTS, 'click', (e) => {
        e.preventDefault();
        stopSpeaking();
    });

    const btnStopVoice = $('#btn-stop-tts-voice');
    if (btnStopVoice) on(btnStopVoice, 'click', (e) => {
        e.preventDefault();
        stopSpeaking();
    });

    const btnStopCommand = $('#btn-stop-voice');
    if (btnStopCommand) on(btnStopCommand, 'click', (e) => {
        e.preventDefault();
        stopSpeaking();
    });

    const skinToggle = $('#skin-toggle');
    const skinCarousel = $('#skin-carousel');
    if (skinToggle) on(skinToggle, 'click', () => {
        skinCarousel.classList.toggle('open');
        skinToggle.classList.toggle('active');
    });

    delegate(document.body, 'click', '.skin-btn', (e, target) => {
        $$('.skin-btn').forEach(b => b.classList.remove('active'));
        target.classList.add('active');
        avatarManager.changeSkin(target.dataset.skin);
        uiManager.updateSpeech(`เปลี่ยนเป็น ${target.title} แล้วค่ะ! ✨`);
    });

    delegate(document.body, 'click', '.play-song-btn', (e, target) => {
        uiManager.playYouTubeVideo(target.dataset.videoId, target.dataset.title);
    });
    delegate(document.body, 'click', '.close-panel-btn', () => uiManager.hidePanel());
}

function renderQuickScripts() {
    const container = document.getElementById('quick-chips-container');
    if (!container) return;

    container.innerHTML = '';
    const currentLang = languageManager.getCurrentLanguage();

    quickScripts.forEach(script => {
        const chip = document.createElement('div');
        chip.className = 'quick-chip';
        chip.innerHTML = `
            <span class="quick-chip-icon">${script.icon}</span> 
            <span class="quick-chip-text">${script.label[currentLang] || script.label['en']}</span>
        `;
        chip.onclick = () => {
            handleLocalScript(script);
        };
        container.appendChild(chip);
    });
}

async function handleLocalScript(script) {
    const currentLang = languageManager.getCurrentLanguage();
    const userPrompt = script.prompt[currentLang] || script.prompt['en'];
    const aiResponse = script.response[currentLang] || script.response['en'];
    if (!userPrompt || !aiResponse) return;

    uiManager.updateSpeech(`🗣️ ${userPrompt}`);
    uiManager.showLoading();
    await new Promise(r => setTimeout(r, 600));
    uiManager.hideLoading();

    stateManager.set('lastAiResponse', aiResponse);
    const fakeResult = {
        answer: aiResponse,
        avatar_mood: script.mood || 'talking',
        conversation_id: stateManager.get('sessionId')
    };

    renderResponsePanel(fakeResult);

    if (fakeResult.avatar_mood) {
        avatarManager.setMood(fakeResult.avatar_mood);
    }
    avatarManager.speak(fakeResult.answer, fakeResult.avatar_mood, currentLang);
}

function stopSpeaking() {
    avatarManager.stop();
    uiManager.hideLoading();
    if (stateManager.get('isVoiceMode')) {
        voiceModeManager.resumeRecording();
    }
}

window.submitFeedback = async (type, btn) => {
    const query = stateManager.get('lastUserQuery') || "unknown";
    const sessionId = stateManager.get('sessionId');
    const response = stateManager.get('lastAiResponse') || "No response recorded";

    try {
        await fetch('/api/analytics/submit_feedback', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ session_id: sessionId, query, response, feedback_type: type })
        });
        if (btn) {
            btn.disabled = true;
            btn.style.opacity = '0.5';
        }
    } catch (e) { console.error(e); }
};
