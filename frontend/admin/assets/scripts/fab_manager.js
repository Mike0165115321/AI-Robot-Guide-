/**
 * fab_manager.js
 * Centralized Manager for Floating Action Buttons (FAB) and their associated Widgets (UI).
 * Serves as a "Widget Factory" for both chat.js and avatar_logic.js.
 */

class FabManager {
    constructor(config) {
        this.config = {
            buttons: {
                music: 'music-button',
                faq: 'faq-button',
                calc: 'calc-button',
                nav: 'nav-button',
                ...config.buttons
            },
            callbacks: {
                // (text, intent, additionalData) => {}
                sendMessage: null,
                // Optional overrides (if you really need to hijack the action before widget creation)
                onMusicAction: null,
                onNavAction: null,
                onFaqAction: null,
                ...config.callbacks
            },
            isAvatarMode: config.isAvatarMode || false,
            apiBaseUrl: config.apiBaseUrl || (window.API_BASE_URL || 'http://localhost:9090')
        };

        this.init();
    }

    init() {
        console.log("🛠️ FabManager Initializing...");
        const { buttons } = this.config;

        this._bindButton(buttons.music, () => this.handleMusicAction());
        this._bindButton(buttons.faq, () => this.handleFaqAction());
        this._bindButton(buttons.calc, () => this.handleCalcAction());
        this._bindButton(buttons.nav, () => this.handleNavAction());
    }

    _bindButton(id, handler) {
        const btn = document.getElementById(id);
        if (btn) {
            // Remove old listeners by cloning (safest way to clear anonymous listeners)
            const newBtn = btn.cloneNode(true);
            if (btn.parentNode) {
                btn.parentNode.replaceChild(newBtn, btn);
                newBtn.addEventListener('click', handler);
            }
        }
    }

    // --- Action Handlers (Controller Logic) ---

    handleMusicAction() {
        if (this.config.callbacks.onMusicAction) {
            this.config.callbacks.onMusicAction(); // Allow override
        } else {
            // Default: Request Host to display Music Widget
            console.log("🎵 FabManager: Music Action Triggered");
            // NOTE: The host (chat.js/avatar_logic.js) should have its own way to "show" the widget.
            // But here we are centralized. 
            // Ideally, the host subscribes to 'onMusicAction' and calls `createMusicWidget()` to append it.
            // Since we are refactoring, we will assume the HOST passed a callback that *uses* the widget.
            console.warn("Please implement onMusicAction in the host to use createMusicWidget()");
        }
    }

    handleNavAction() {
        if (this.config.callbacks.onNavAction) {
            this.config.callbacks.onNavAction();
        } else {
            console.warn("Please implement onNavAction in the host to use createNavigationWidget()");
        }
    }

    handleFaqAction() {
        if (this.config.callbacks.onFaqAction) {
            this.config.callbacks.onFaqAction();
        } else {
            console.warn("Please implement onFaqAction in the host to use createFAQWidget()");
        }
    }

    handleCalcAction() {
        if (this.config.callbacks.onCalcAction) {
            this.config.callbacks.onCalcAction();
        } else {
            console.warn("Please implement onCalcAction in the host to use createCalculatorWidget()");
        }
    }

    // --- Widget Factories (View Logic) ---

    /**
     * Create the Navigation Widget (In-Place Map Search)
     */
    createNavigationWidget() {
        const container = document.createElement('div');
        container.className = 'fab-widget-container';

        container.innerHTML = `
            <div class="bubble system-bubble" style="max-width: 100%;">
                <div class="prose" style="font-size: 0.95rem;">
                    <h3 style="margin-top: 0;">🗺️ จะไปไหนดีคะ?</h3>
                    <p>เลือกสถานที่ยอดนิยม หรือพิมพ์ชื่อสถานที่ที่ต้องการไป:</p>
                </div>
                <div class="nav-locations" style="display: flex; flex-wrap: wrap; gap: 8px; margin: 15px 0;">
                    <button class="nav-loc-btn" data-slug="wat-phumin" style="padding: 8px 16px; background: rgba(59, 130, 246, 0.2); border: 1px solid rgba(59, 130, 246, 0.4); border-radius: 20px; color: #3b82f6; cursor: pointer;">🛕 วัดภูมินทร์</button>
                    <button class="nav-loc-btn" data-slug="doi-samer-dao" style="padding: 8px 16px; background: rgba(59, 130, 246, 0.2); border: 1px solid rgba(59, 130, 246, 0.4); border-radius: 20px; color: #3b82f6; cursor: pointer;">⛰️ ดอยเสมอดาว</button>
                    <button class="nav-loc-btn" data-slug="wat-chang-kham" style="padding: 8px 16px; background: rgba(59, 130, 246, 0.2); border: 1px solid rgba(59, 130, 246, 0.4); border-radius: 20px; color: #3b82f6; cursor: pointer;">🐘 วัดช้างค้ำ</button>
                </div>
                <div style="display: flex; gap: 8px; margin-top: 10px;">
                    <input type="text" class="nav-search-input" placeholder="หรือพิมพ์ชื่อสถานที่..." style="flex: 1; padding: 10px 15px; border: 1px solid rgba(255,255,255,0.2); border-radius: 8px; background: rgba(0,0,0,0.3); color: white; fontsize: 0.9rem;">
                    <button class="nav-search-btn" style="padding: 10px 20px; background: linear-gradient(135deg, #3b82f6, #2563eb); border: none; border-radius: 8px; color: white; cursor: pointer; font-weight: bold;">🗺️ นำทาง</button>
                </div>
            </div>
        `;

        // Logic binding
        const bindEvents = () => {
            const locBtns = container.querySelectorAll('.nav-loc-btn');
            const searchBtn = container.querySelector('.nav-search-btn');
            const input = container.querySelector('.nav-search-input');
            const bubble = container.querySelector('.bubble');

            const handleNav = async (slug, query, btn) => {
                const originalHtml = btn.innerHTML;
                btn.disabled = true;
                btn.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i>';

                try {
                    const payload = {};
                    if (slug) payload.slug = slug;
                    if (query) payload.query = query;

                    const res = await fetch(`${this.config.apiBaseUrl}/api/chat/navigation`, {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify(payload)
                    });
                    const data = await res.json();

                    if (data.success && data.result) {
                        const answer = data.result.answer;
                        const mapEmbed = data.result.action_payload?.embed_url
                            ? `<iframe src="${data.result.action_payload.embed_url}" width="100%" height="250" style="border:0; border-radius:12px; margin-top:10px;" allowfullscreen="" loading="lazy"></iframe>` : '';
                        const linkUrl = data.result.action_payload?.link
                            ? `<a href="${data.result.action_payload.link}" target="_blank" style="display:block; margin-top:10px; text-align:center; background:#ef4444; color:white; padding:8px; border-radius:8px; text-decoration:none;">เปิดใน Google Maps 🚗</a>` : '';

                        bubble.innerHTML = `
                            <div class="prose" style="font-size: 0.95rem;">
                                <p>${answer}</p>
                                ${mapEmbed}
                                ${linkUrl}
                            </div>
                            <button class="close-widget-btn" style="margin-top:10px; background:none; border:none; color:#aaa; font-size:0.8rem; cursor:pointer;">ปิดหน้าต่างนี้</button>
                        `;
                        bubble.querySelector('.close-widget-btn').addEventListener('click', () => container.remove());

                    } else {
                        alert(`ไม่พบสถานที่: ${data.error || 'ERROR'}`);
                        btn.innerHTML = originalHtml;
                        btn.disabled = false;
                    }
                } catch (e) {
                    console.error(e);
                    alert("เกิดข้อผิดพลาดในการเชื่อมต่อ");
                    btn.innerHTML = originalHtml;
                    btn.disabled = false;
                }
            };

            locBtns.forEach(btn => btn.addEventListener('click', () => handleNav(btn.dataset.slug, null, btn)));

            searchBtn.addEventListener('click', () => {
                if (input.value.trim()) handleNav(null, input.value.trim(), searchBtn);
            });
            input.addEventListener('keypress', (e) => {
                if (e.key === 'Enter' && input.value.trim()) handleNav(null, input.value.trim(), searchBtn);
            });
        };

        // Defer binding to ensure appended? No, direct binding works on created elements.
        bindEvents();
        return container;
    }

    /**
     * Create the Music Widget (Genre Buttons + Search)
     */
    createMusicWidget() {
        const container = document.createElement('div');
        container.className = 'fab-widget-container';

        container.innerHTML = `
            <div class="bubble system-bubble" style="max-width: 100%;">
                <div class="prose" style="font-size: 0.95rem;">
                    <h3 style="margin-top: 0;">🎵 ฟังเพลงอะไรดีคะ?</h3>
                    <p>เลือกแนวเพลง หรือพิมพ์ชื่อเพลง/ศิลปิน:</p>
                </div>
                <div class="music-genres" style="display: flex; flex-wrap: wrap; gap: 8px; margin: 15px 0;">
                    <button class="genre-btn" data-genre="เพลงคำเมือง" style="padding: 8px 16px; background: rgba(16, 185, 129, 0.2); border: 1px solid rgba(16, 185, 129, 0.4); border-radius: 20px; color: #10b981; cursor: pointer;">🎻 คำเมือง</button>
                    <button class="genre-btn" data-genre="เพลงลูกทุ่ง" style="padding: 8px 16px; background: rgba(236, 72, 153, 0.2); border: 1px solid rgba(236, 72, 153, 0.4); border-radius: 20px; color: #ec4899; cursor: pointer;">🌾 ลูกทุ่ง</button>
                    <button class="genre-btn" data-genre="เพลงป๊อปสบายๆ" style="padding: 8px 16px; background: rgba(245, 158, 11, 0.2); border: 1px solid rgba(245, 158, 11, 0.4); border-radius: 20px; color: #f59e0b; cursor: pointer;">🎸 ป๊อปสบายๆ</button>
                </div>
                <div style="display: flex; gap: 8px; margin-top: 10px;">
                    <input type="text" class="music-search-input" placeholder="พิมพ์ชื่อเพลง..." style="flex: 1; padding: 10px 15px; border: 1px solid rgba(255,255,255,0.2); border-radius: 8px; background: rgba(0,0,0,0.3); color: white; fontsize: 0.9rem;">
                    <button class="music-search-btn" style="padding: 10px 20px; background: linear-gradient(135deg, #10b981, #059669); border: none; border-radius: 8px; color: white; cursor: pointer; font-weight: bold;">🔍 ค้นหา</button>
                </div>
            </div>
        `;

        const bindEvents = () => {
            const genreBtns = container.querySelectorAll('.genre-btn');
            const searchBtn = container.querySelector('.music-search-btn');
            const input = container.querySelector('.music-search-input');
            const bubble = container.querySelector('.bubble');

            const searchMusic = async (term, btn) => {
                const originalHtml = btn.innerHTML;
                btn.disabled = true;
                btn.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i>';

                try {
                    const res = await fetch(`${this.config.apiBaseUrl}/api/chat/music-search`, {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify({ song_name: term })
                    });
                    const data = await res.json();

                    if (data.success && data.results && data.results.length > 0) {
                        this._renderMusicResults(bubble, term, data.results);
                    } else {
                        // Keep search box, just show error below
                        const existingErr = bubble.querySelector('.music-error');
                        if (existingErr) existingErr.remove();
                        bubble.insertAdjacentHTML('beforeend', `<div class="music-error" style="color:#ef4444; margin-top:10px;">❌ ไม่พบเพลง "${term}"</div>`);
                    }
                } catch (e) {
                    console.error(e);
                    alert("เกิดข้อผิดพลาด");
                } finally {
                    btn.innerHTML = originalHtml;
                    btn.disabled = false;
                    input.value = '';
                }
            };

            genreBtns.forEach(btn => btn.addEventListener('click', () => searchMusic(btn.dataset.genre, btn)));
            searchBtn.addEventListener('click', () => {
                if (input.value.trim()) searchMusic(input.value.trim(), searchBtn);
            });
            input.addEventListener('keypress', (e) => {
                if (e.key === 'Enter' && input.value.trim()) searchMusic(input.value.trim(), searchBtn);
            });
        };
        bindEvents();
        return container;
    }

    _renderMusicResults(bubble, title, results) {
        // Clear previous content but keep header for context? Or replace all?
        // User liked "In-Place" loading. So let's replace the searching part with results.

        let listHtml = `<div class="music-results" style="margin-top:10px;">`;
        if (title) listHtml += `<p style="margin-bottom:8px;">ผลลัพธ์สำหรับ: <strong>${title}</strong></p>`;

        results.slice(0, 5).forEach((song, index) => {
            listHtml += `
                <div class="song-item" data-index="${index}" style="display: flex; gap: 10px; margin-bottom: 8px; background: rgba(255,255,255,0.05); padding: 8px; border-radius: 8px; align-items: center;">
                    <img src="${song.thumbnail}" style="width: 50px; height: 50px; border-radius: 4px; object-fit: cover;">
                    <div style="flex: 1; overflow: hidden;">
                        <div style="font-weight: bold; white-space: nowrap; overflow: hidden; text-overflow: ellipsis;">${song.title}</div>
                        <div style="font-size: 0.8rem; color: #aaa;">${song.duration}</div>
                    </div>
                    <button class="play-btn" data-index="${index}" style="background: #1db954; color: white; border: none; border-radius: 50%; width: 32px; height: 32px; cursor: pointer; display: flex; align-items: center; justify-content: center;"><i class="fa-solid fa-play"></i></button>
                </div>
            `;
        });
        listHtml += `</div>
            <div class="music-player-wrapper" style="margin-top: 15px;"></div>
            <button class="close-widget-btn" style="margin-top:10px; background:none; border:none; color:#aaa; font-size:0.8rem; cursor:pointer;">ปิดหน้าต่างนี้</button>
        `;

        bubble.innerHTML = listHtml; // Replace content

        // Store results for later use
        const songsData = results.slice(0, 5);

        // Bind Play Buttons - Use global musicPlayer instance
        bubble.querySelectorAll('.play-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const index = parseInt(btn.dataset.index);
                const song = songsData[index];

                if (song && typeof musicPlayer !== 'undefined') {
                    // Get the player wrapper inside this bubble
                    const playerWrapper = bubble.querySelector('.music-player-wrapper');
                    if (playerWrapper) {
                        // Create player using the global musicPlayer instance
                        musicPlayer.createPlayer(song, playerWrapper);
                        // Scroll to player
                        playerWrapper.scrollIntoView({ behavior: 'smooth', block: 'center' });
                    }
                } else {
                    console.warn("No music player found! Make sure music_player.js is loaded.");
                }
            });
        });

        bubble.querySelector('.close-widget-btn').addEventListener('click', () => bubble.closest('.fab-widget-container').remove());
    }

    /**
     * Create FAQ Widget
     */
    createFAQWidget() {
        const container = document.createElement('div');
        container.className = 'fab-widget-container';

        const questions = [
            "แนะนำที่เที่ยวน่านหน่อย",
            "วัดสำคัญในน่านมีที่ไหนบ้าง",
            "ร้านอาหารพื้นเมืองที่ห้ามพลาด",
            "โรงแรมที่พักในเมืองน่าน",
            "ของฝากน่านมีอะไรบ้าง"
        ];

        let html = `
            <div class="bubble system-bubble" style="max-width: 100%;">
                <div class="prose" style="font-size: 0.95rem;">
                    <h3 style="margin-top: 0;">❓ คำถามที่พบบ่อย</h3>
                    <p>ลองเลือกหัวข้อที่สนใจได้เลยค่ะ:</p>
                </div>
                <div class="faq-list" style="display: flex; flex-direction: column; gap: 8px; margin-top: 15px;">
        `;

        questions.forEach(q => {
            html += `<button class="faq-btn" data-q="${q}" style="text-align: left; padding: 10px 15px; background: rgba(255,255,255,0.1); border: 1px solid rgba(255,255,255,0.1); border-radius: 12px; color: white; cursor: pointer; transition: all 0.2s;">
                <i class="fa-regular fa-comment-dots" style="margin-right: 8px; color: #60a5fa;"></i> ${q}
            </button>`;
        });

        html += `</div>
            <button class="close-widget-btn" style="margin-top:10px; background:none; border:none; color:#aaa; font-size:0.8rem; cursor:pointer;">ปิดหน้าต่างนี้</button>
        </div>`;

        container.innerHTML = html;

        container.querySelectorAll('.faq-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const text = btn.dataset.q;
                if (this.config.callbacks.sendMessage) {
                    const intent = window.NanApp?.INTENTS?.FAQ || 'FAQ';
                    this.config.callbacks.sendMessage(text, intent);
                    container.remove(); // Close after select
                }
            });
        });

        container.querySelector('.close-widget-btn').addEventListener('click', () => container.remove());

        return container;
    }

    /**
     * Create the Calculator Widget (Scientific Calculator)
     * Delegates to CalculatorWidget class from calculator_widget.js
     */
    createCalculatorWidget() {
        if (window.CalculatorWidget) {
            return CalculatorWidget.create();
        } else {
            console.error("CalculatorWidget not loaded! Make sure calculator_widget.js is included.");
            const container = document.createElement('div');
            container.innerHTML = '<div class="bubble system-bubble"><p style="color: #ef4444;">❌ ไม่สามารถโหลดเครื่องคิดเลขได้</p></div>';
            return container;
        }
    }
}

window.FabManager = FabManager;
