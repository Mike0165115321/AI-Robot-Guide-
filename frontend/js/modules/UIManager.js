/**
 * # UIManager.js
 * Handles all DOM manipulations and visual feedback.
 */

import { $, $$ } from '../utils/dom.js';
import responseRenderer from '../components/responseRenderer.js';
import { renderInline } from '../services/markdownService.js';
import { getFullImageUrl } from '../config.js';
import aiModeManager from '../services/aiModeManager.js';

class UIManager {
    constructor() {
        if (UIManager.instance) return UIManager.instance;
        UIManager.instance = this;
    }

    // ==========================================
    // SPEECH BUBBLE & FEEDBACK
    // ==========================================
    updateSpeech(text) {
        const speechText = $('#speech-text');
        if (speechText) {
            speechText.innerHTML = renderInline(text);
            speechText.style.opacity = 0;
            setTimeout(() => speechText.style.opacity = 1, 50);
        } else {
            console.log('Bot says:', text);
        }
    }

    // ==========================================
    // LOADING INDICATORS
    // ==========================================
    showLoading() {
        const loading = $('#loading-indicator');
        if (loading) loading.style.display = 'flex';
        // Add additional loading states if needed
    }

    hideLoading() {
        const loading = $('#loading-indicator');
        if (loading) loading.style.display = 'none';
    }

    // ==========================================
    // RIGHT PANEL (SLIDE-OUT)
    // ==========================================
    showPanel(htmlContent) {
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

    hidePanel() {
        const panel = $('#presentation-panel');
        const avatarSection = $('#avatar-section');

        if (panel) panel.classList.remove('visible');
        if (avatarSection) {
            avatarSection.classList.remove('shifted');
            avatarSection.classList.add('centered');
        }
        this.updateSpeech('มีอะไรให้ช่วยอีกไหมคะ? 😊');
    }

    // ==========================================
    // SPECIAL RENDERERS
    // ==========================================

    /**
     * Play YouTube video in panel
     */
    playYouTubeVideo(videoId, title) {
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
        this.showPanel(playerHtml);
        this.updateSpeech(`กำลังเล่นเพลงให้ฟังค่ะ 🎧`);
    }

    /**
     * Render music selection list
     */
    renderMusicList(songs) {
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

    renderPayload(payload) {
        const html = responseRenderer.render(payload);
        if (html) this.showPanel(html);
    }

    // ==========================================
    // ALERT TOASTS
    // ==========================================
    showToastAlert(alert) {
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
            cursor: pointer;
            transition: transform 0.2s;
        `;

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

        // Events
        toast.onclick = (e) => {
            if (e.target.classList.contains('close-btn') || e.target.innerText === '✕') return;
            const html = responseRenderer.renderAlert(alert);
            this.showPanel(html);
        };

        const closeBtn = toast.querySelector('.close-btn');
        if (closeBtn) {
            closeBtn.onclick = (e) => {
                e.stopPropagation();
                toast.remove();
            };
        }

        setTimeout(() => {
            if (toast.parentElement) toast.remove();
        }, 15000);

        // Return toast so caller can update speech if needed, but usually logic handles speech
    }

    // ==========================================
    // BUTTON UPDATES
    // ==========================================
    updateAIModeButton() {
        const btn = $('#btn-ai-mode');
        if (!btn) return;

        const info = aiModeManager.getModeInfo();
        btn.innerText = info.icon;
        btn.title = info.description;

        if (info.mode === 'fast') {
            btn.style.background = 'rgba(251, 191, 36, 0.2)';
            btn.style.color = '#fbbf24';
            btn.style.borderColor = '#fbbf24';
        } else {
            btn.style.background = 'rgba(139, 92, 246, 0.2)';
            btn.style.color = '#a78bfa';
            btn.style.borderColor = '#a78bfa';
        }
    }
}

const uiManager = new UIManager();
export default uiManager;
