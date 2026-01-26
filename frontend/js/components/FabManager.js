/**
 * # FAB Manager (Floating Action Buttons)
 * 
 * จัดการปุ่มลอยและ Widgets
 * - 🎵 Music - ค้นหาและเล่นเพลง
 * - ❓ FAQ - คำถามที่พบบ่อย
 * - 🧮 Calculator - เครื่องคิดเลข
 * - 🗺️ Navigation - นำทางไปสถานที่
 * 
 * @example
 * import { fabManager } from './components/FabManager.js';
 * fabManager.init({
 *     onSendMessage: (text) => chatService.sendText(text)
 * });
 */

import { CONFIG } from '../config.js';

// =============================================
// FAB MANAGER CLASS
// =============================================
class FabManager {
    constructor() {
        this.isOpen = false;
        this.container = null;
        this.callbacks = {
            onSendMessage: null,
            onWidgetOpen: null,
            onWidgetClose: null
        };

        // Button IDs
        this.buttonIds = {
            toggle: 'fab-toggle',
            music: 'fab-music',
            faq: 'fab-faq',
            calc: 'fab-calc',
            nav: 'fab-nav'
        };
    }

    /**
     * Initialize FAB Manager
     */
    init(options = {}) {
        this.callbacks = { ...this.callbacks, ...options };
        this._createFabContainer();
        this._bindEvents();
        console.log('🔘 FabManager: Initialized');
    }

    /**
     * Multi-language labels for FAB buttons
     */
    static labels = {
        music: { th: 'ฟังเพลง', en: 'Music', ja: '音楽', zh: '听音乐', ru: 'Музыка', hi: 'संगीत', ms: 'Dengar Lagu' },
        faq: { th: 'ถามบ่อย', en: 'FAQ', ja: 'よくある質問', zh: '常见问题', ru: 'Вопросы', hi: 'सवाल', ms: 'Soalan' },
        calc: { th: 'คิดเลข', en: 'Calculator', ja: '電卓', zh: '计算器', ru: 'Калькулятор', hi: 'कैलकुलेटर', ms: 'Kalkulator' }
    };

    /**
     * Create Sidebar Cards HTML structure
     * @private
     */
    _createFabContainer() {
        const container = document.getElementById('right-actions-sidebar');
        if (!container) return;

        this._renderButtons(container);
        this.container = container;
        this._injectStyles();

        // Listen for language changes
        window.addEventListener('languageChanged', () => {
            this._updateButtonLabels();
        });
    }

    /**
     * Render FAB buttons with current language
     * @private
     */
    _renderButtons(container) {
        const lang = localStorage.getItem('app_language') || 'th';
        const labels = FabManager.labels;

        container.innerHTML = `
            <button id="fab-music" class="quick-chip" title="${labels.music[lang] || labels.music.th}">
                <span class="quick-chip-icon">🎵</span>
                <span class="quick-chip-text">${labels.music[lang] || labels.music.th}</span>
            </button>
            <button id="fab-faq" class="quick-chip" title="${labels.faq[lang] || labels.faq.th}">
                <span class="quick-chip-icon">❓</span>
                <span class="quick-chip-text">${labels.faq[lang] || labels.faq.th}</span>
            </button>
            <button id="fab-calc" class="quick-chip" title="${labels.calc[lang] || labels.calc.th}">
                <span class="quick-chip-icon">🧮</span>
                <span class="quick-chip-text">${labels.calc[lang] || labels.calc.th}</span>
            </button>
        `;
    }

    /**
     * Update button labels when language changes
     * @private
     */
    _updateButtonLabels() {
        const lang = localStorage.getItem('app_language') || 'th';
        const labels = FabManager.labels;

        const musicBtn = document.getElementById('fab-music');
        const faqBtn = document.getElementById('fab-faq');
        const calcBtn = document.getElementById('fab-calc');

        if (musicBtn) {
            musicBtn.querySelector('.quick-chip-text').textContent = labels.music[lang] || labels.music.th;
            musicBtn.title = labels.music[lang] || labels.music.th;
        }
        if (faqBtn) {
            faqBtn.querySelector('.quick-chip-text').textContent = labels.faq[lang] || labels.faq.th;
            faqBtn.title = labels.faq[lang] || labels.faq.th;
        }
        if (calcBtn) {
            calcBtn.querySelector('.quick-chip-text').textContent = labels.calc[lang] || labels.calc.th;
            calcBtn.title = labels.calc[lang] || labels.calc.th;
        }

        console.log(`🌍 [FabManager] Labels updated to: ${lang}`);
    }

    /**
     * Inject CSS styles
     * @private
     */
    _injectStyles() {
        if (document.getElementById('fab-styles')) return;

        const css = `
            .fab-container {
                position: fixed;
                bottom: 100px;
                right: 20px;
                z-index: 1000;
                display: flex;
                flex-direction: column;
                align-items: flex-end;
                gap: 10px;
            }

            .fab-toggle {
                width: 56px;
                height: 56px;
                border-radius: 50%;
                border: none;
                background: linear-gradient(135deg, #6366f1, #8b5cf6);
                color: white;
                font-size: 1.5rem;
                cursor: pointer;
                box-shadow: 0 4px 15px rgba(99, 102, 241, 0.4);
                transition: all 0.3s ease;
            }

            .fab-toggle:hover {
                transform: scale(1.1);
                box-shadow: 0 6px 20px rgba(99, 102, 241, 0.6);
            }

            .fab-toggle.active {
                background: linear-gradient(135deg, #ef4444, #dc2626);
            }

            .fab-toggle .fab-icon-close {
                display: none;
            }

            .fab-toggle.active .fab-icon-open {
                display: none;
            }

            .fab-toggle.active .fab-icon-close {
                display: inline;
            }

            .fab-actions {
                display: flex;
                flex-direction: column;
                gap: 10px;
                opacity: 0;
                visibility: hidden;
                transform: translateY(20px);
                transition: all 0.3s ease;
            }

            .fab-actions.open {
                opacity: 1;
                visibility: visible;
                transform: translateY(0);
            }

            .fab-btn {
                width: 48px;
                height: 48px;
                border-radius: 50%;
                border: none;
                background: rgba(255, 255, 255, 0.1);
                backdrop-filter: blur(10px);
                color: white;
                font-size: 1.3rem;
                cursor: pointer;
                box-shadow: 0 2px 10px rgba(0, 0, 0, 0.3);
                transition: all 0.2s ease;
            }

            .fab-btn:hover {
                transform: scale(1.15);
                background: rgba(255, 255, 255, 0.2);
            }

            /* Widget Container */
            .fab-widget {
                position: fixed;
                bottom: 180px;
                right: 20px;
                width: 340px;
                max-height: 70vh;
                background: rgba(30, 30, 40, 0.95);
                backdrop-filter: blur(20px);
                border: 1px solid rgba(255, 255, 255, 0.1);
                border-radius: 16px;
                padding: 20px;
                z-index: 999;
                box-shadow: 0 10px 40px rgba(0, 0, 0, 0.5);
                overflow-y: auto;
                animation: widget-slide-up 0.3s ease-out;
            }

            @keyframes widget-slide-up {
                from {
                    opacity: 0;
                    transform: translateY(20px);
                }
                to {
                    opacity: 1;
                    transform: translateY(0);
                }
            }

            .fab-widget-header {
                display: flex;
                justify-content: space-between;
                align-items: center;
                margin-bottom: 15px;
                padding-bottom: 10px;
                border-bottom: 1px solid rgba(255, 255, 255, 0.1);
            }

            .fab-widget-header h3 {
                margin: 0;
                font-size: 1.1rem;
                color: white;
            }

            .fab-widget-close {
                background: none;
                border: none;
                color: rgba(255, 255, 255, 0.5);
                font-size: 1.2rem;
                cursor: pointer;
            }

            .fab-widget-close:hover {
                color: white;
            }

            /* FAQ Buttons */
            .faq-btn {
                width: 100%;
                text-align: left;
                padding: 12px 15px;
                background: rgba(255, 255, 255, 0.05);
                border: 1px solid rgba(255, 255, 255, 0.1);
                border-radius: 10px;
                color: white;
                cursor: pointer;
                transition: all 0.2s;
                font-size: 0.9rem;
            }

            .faq-btn:hover {
                background: rgba(99, 102, 241, 0.2);
                border-color: rgba(99, 102, 241, 0.4);
            }

            /* Genre/Location Buttons */
            .genre-btn, .nav-loc-btn {
                padding: 8px 16px;
                border-radius: 20px;
                border: 1px solid;
                cursor: pointer;
                transition: all 0.2s;
                font-size: 0.85rem;
            }

            .genre-btn:hover, .nav-loc-btn:hover {
                transform: scale(1.05);
            }

            /* Mobile Responsive */
            @media (max-width: 480px) {
                .fab-widget {
                    width: calc(100vw - 40px);
                    right: 20px;
                    left: 20px;
                }
            }
        `;

        const style = document.createElement('style');
        style.id = 'fab-styles';
        style.textContent = css;
        document.head.appendChild(style);
    }

    /**
     * Bind button events
     * @private
     */
    _bindEvents() {
        // Toggle button
        const toggleBtn = document.getElementById(this.buttonIds.toggle);
        if (toggleBtn) {
            toggleBtn.addEventListener('click', () => this.toggle());
        }

        // Action buttons
        document.getElementById(this.buttonIds.music)?.addEventListener('click', () => this.showMusicWidget());
        document.getElementById(this.buttonIds.faq)?.addEventListener('click', () => this.showFaqWidget());
        document.getElementById(this.buttonIds.calc)?.addEventListener('click', () => this.showCalcWidget());
        document.getElementById(this.buttonIds.nav)?.addEventListener('click', () => this.showNavWidget());
    }

    /**
     * Toggle FAB menu
     */
    toggle() {
        this.isOpen = !this.isOpen;

        const toggleBtn = document.getElementById(this.buttonIds.toggle);
        const actions = document.getElementById('fab-actions');

        if (this.isOpen) {
            toggleBtn?.classList.add('active');
            actions?.classList.add('open');
        } else {
            toggleBtn?.classList.remove('active');
            actions?.classList.remove('open');
        }
    }

    /**
     * Close FAB menu
     */
    close() {
        this.isOpen = false;
        document.getElementById(this.buttonIds.toggle)?.classList.remove('active');
        document.getElementById('fab-actions')?.classList.remove('open');
    }

    /**
     * Show a widget
     * @private
     */
    _showWidget(title, content) {
        this._closeAllWidgets();
        this.close();

        const widget = document.createElement('div');
        widget.className = 'fab-widget';
        widget.innerHTML = `
            <div class="fab-widget-header">
                <h3>${title}</h3>
                <button class="fab-widget-close">✕</button>
            </div>
            <div class="fab-widget-content">
                ${content}
            </div>
        `;

        document.body.appendChild(widget);

        widget.querySelector('.fab-widget-close').addEventListener('click', () => {
            this._closeWidget(widget);
        });

        this.callbacks.onWidgetOpen?.();
        return widget;
    }

    /**
     * Close a widget
     * @private
     */
    _closeWidget(widget) {
        widget.style.opacity = '0';
        widget.style.transform = 'translateY(20px)';
        setTimeout(() => widget.remove(), 300);
        this.callbacks.onWidgetClose?.();
    }

    /**
     * Close all widgets
     * @private
     */
    _closeAllWidgets() {
        document.querySelectorAll('.fab-widget').forEach(w => w.remove());
    }

    // ==========================================
    // WIDGET CREATORS
    // ==========================================

    /**
     * Show FAQ Widget
     */
    showFaqWidget() {
        const lang = localStorage.getItem('app_language') || 'th';

        // i18n for FAQ
        const faqData = {
            title: { th: '❓ คำถามที่พบบ่อย', en: '❓ Frequently Asked Questions', ja: '❓ よくある質問', zh: '❓ 常见问题', ru: '❓ Часто задаваемые вопросы', hi: '❓ अक्सर पूछे जाने वाले प्रश्न', ms: '❓ Soalan Lazim' },
            questions: [
                { th: 'แนะนำที่เที่ยวน่านหน่อย', en: 'Recommend places to visit in Nan', ja: '南の観光地を教えて', zh: '推荐南部的旅游景点', ru: 'Порекомендуйте места для посещения в Нане', hi: 'नान में घूमने की जगहें सुझाएं', ms: 'Cadangkan tempat menarik di Nan' },
                { th: 'วัดสำคัญในน่านมีที่ไหนบ้าง', en: 'What are important temples in Nan?', ja: '南の重要なお寺はどこ？', zh: '南部有哪些重要的寺庙？', ru: 'Какие важные храмы в Нане?', hi: 'नान में महत्वपूर्ण मंदिर कौन से हैं?', ms: 'Apakah kuil penting di Nan?' },
                { th: 'ร้านอาหารพื้นเมืองที่ห้ามพลาด', en: 'Local restaurants not to miss', ja: '見逃せない地元のレストラン', zh: '不可错过的当地餐厅', ru: 'Местные рестораны, которые нельзя пропустить', hi: 'स्थानीय रेस्तरां जो मिस नहीं करने चाहिए', ms: 'Restoran tempatan yang tidak boleh dilepaskan' },
                { th: 'โรงแรมที่พักในเมืองน่าน', en: 'Hotels in Nan city', ja: '南市内のホテル', zh: '南市的酒店', ru: 'Отели в городе Нан', hi: 'नान शहर में होटल', ms: 'Hotel di bandar Nan' },
                { th: 'ของฝากน่านมีอะไรบ้าง', en: 'What souvenirs are from Nan?', ja: '南のお土産は何がある？', zh: '南部有什么纪念品？', ru: 'Какие сувениры из Нана?', hi: 'नान से क्या स्मृति चिन्ह मिलते हैं?', ms: 'Apakah cenderamata dari Nan?' }
            ]
        };

        const content = `
            <div style="display: flex; flex-direction: column; gap: 10px;">
                ${faqData.questions.map(q => `
                    <button class="faq-btn" data-q="${q[lang] || q.th}">
                        💬 ${q[lang] || q.th}
                    </button>
                `).join('')}
            </div>
        `;

        const widget = this._showWidget(faqData.title[lang] || faqData.title.th, content);

        widget.querySelectorAll('.faq-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const text = btn.dataset.q;
                this.callbacks.onSendMessage?.(text);
                this._closeWidget(widget);
            });
        });
    }

    /**
     * Show Music Widget
     */
    showMusicWidget() {
        const lang = localStorage.getItem('app_language') || 'th';

        // i18n for Music Widget
        const musicData = {
            title: { th: '🎵 ฟังเพลง', en: '🎵 Listen to Music', ja: '🎵 音楽を聴く', zh: '🎵 听音乐', ru: '🎵 Слушать музыку', hi: '🎵 संगीत सुनें', ms: '🎵 Dengar Lagu' },
            subtitle: { th: 'เลือกแนวเพลง หรือพิมพ์ชื่อเพลง:', en: 'Choose a genre or type a song name:', ja: 'ジャンルを選ぶか曲名を入力:', zh: '选择流派或输入歌曲名称:', ru: 'Выберите жанр или введите название:', hi: 'शैली चुनें या गाने का नाम टाइप करें:', ms: 'Pilih genre atau taip nama lagu:' },
            placeholder: { th: 'พิมพ์ชื่อเพลง...', en: 'Type song name...', ja: '曲名を入力...', zh: '输入歌曲名称...', ru: 'Введите название песни...', hi: 'गाने का नाम टाइप करें...', ms: 'Taip nama lagu...' },
            genres: [
                { th: 'คำเมือง', en: 'Northern Thai', ja: '北部タイ', zh: '北部泰式', ru: 'Северный Тайский', hi: 'उत्तरी थाई', ms: 'Thai Utara', icon: '🎻', color: '#10b981' },
                { th: 'ลูกทุ่ง', en: 'Luk Thung', ja: 'ルクトゥン', zh: '乡村', ru: 'Лук Тунг', hi: 'लुक थुंग', ms: 'Luk Thung', icon: '🌾', color: '#ec4899' },
                { th: 'ป๊อปสบายๆ', en: 'Easy Pop', ja: 'イージーポップ', zh: '轻松流行', ru: 'Поп', hi: 'आसान पॉप', ms: 'Pop Santai', icon: '🎸', color: '#f59e0b' },
                { th: 'บรรเลง', en: 'Instrumental', ja: '器楽曲', zh: '器乐', ru: 'Инструментал', hi: 'वाद्य', ms: 'Instrumental', icon: '🎹', color: '#6366f1' }
            ]
        };

        const content = `
            <p style="margin-bottom: 15px; opacity: 0.8;">${musicData.subtitle[lang] || musicData.subtitle.th}</p>
            <div style="display: flex; flex-wrap: wrap; gap: 8px; margin-bottom: 15px;">
                ${musicData.genres.map(g => `
                    <button class="genre-btn" data-genre="เพลง${g.th}" 
                        style="background: ${g.color}20; border-color: ${g.color}60; color: ${g.color}">
                        ${g.icon} ${g[lang] || g.th}
                    </button>
                `).join('')}
            </div>
            <div style="display: flex; gap: 8px;">
                <input type="text" class="music-input" placeholder="${musicData.placeholder[lang] || musicData.placeholder.th}" 
                    style="flex: 1; padding: 10px; border: 1px solid rgba(255,255,255,0.2); border-radius: 8px; background: rgba(0,0,0,0.3); color: white;">
                <button class="music-search-btn" style="padding: 10px 15px; background: #10b981; border: none; border-radius: 8px; color: white; cursor: pointer;">
                    <span style="pointer-events: none;">🔍</span>
                </button>
            </div>
            <div class="music-results" style="margin-top: 15px;"></div>
        `;

        const widget = this._showWidget(musicData.title[lang] || musicData.title.th, content);

        // Common Search Function
        const searchMusic = (term) => {
            if (!term) return;
            const text = `เปิดเพลง ${term}`;
            console.log('🎵 Searching:', text);
            this.callbacks.onSendMessage?.(text);
            this._closeWidget(widget);
        };

        // 1. Bind Genre Buttons
        widget.querySelectorAll('.genre-btn').forEach(btn => {
            btn.addEventListener('click', () => searchMusic(btn.dataset.genre));
        });

        // 2. Bind Input & Search Button
        const input = widget.querySelector('.music-input');
        const searchBtn = widget.querySelector('.music-search-btn');

        if (searchBtn && input) {
            searchBtn.onclick = () => {
                const term = input.value.trim();
                if (term) {
                    searchMusic(term);
                } else {
                    input.focus();
                    input.style.borderColor = '#ef4444';
                    setTimeout(() => input.style.borderColor = 'rgba(255,255,255,0.2)', 500);
                }
            };

            input.onkeypress = (e) => {
                if (e.key === 'Enter') searchBtn.onclick();
            };
        }
    }

    /**
     * Show Navigation Widget
     */
    showNavWidget() {
        const lang = localStorage.getItem('app_language') || 'th';

        // i18n for Navigation Widget
        const navData = {
            title: { th: '🗺️ นำทาง', en: '🗺️ Navigation', ja: '🗺️ ナビゲーション', zh: '🗺️ 导航', ru: '🗺️ Навигация', hi: '🗺️ नेविगेशन', ms: '🗺️ Navigasi' },
            subtitle: { th: 'เลือกสถานที่ยอดนิยม:', en: 'Choose a popular location:', ja: '人気のスポットを選択:', zh: '选择热门地点:', ru: 'Выберите популярное место:', hi: 'लोकप्रिय स्थान चुनें:', ms: 'Pilih lokasi popular:' },
            placeholder: { th: 'หรือพิมพ์ชื่อสถานที่...', en: 'Or type a place name...', ja: 'または場所名を入力...', zh: '或输入地点名称...', ru: 'Или введите название места...', hi: 'या जगह का नाम टाइप करें...', ms: 'Atau taip nama tempat...' }
        };

        const locations = [
            { name: 'วัดภูมินทร์', icon: '🛕' },
            { name: 'ดอยเสมอดาว', icon: '⛰️' },
            { name: 'วัดช้างค้ำ', icon: '🐘' },
            { name: 'วัดศรีพันต้น', icon: '🏛️' }
        ];

        const content = `
            <p style="margin-bottom: 15px; opacity: 0.8;">${navData.subtitle[lang] || navData.subtitle.th}</p>
            <div style="display: flex; flex-wrap: wrap; gap: 8px; margin-bottom: 15px;">
                ${locations.map(loc => `
                    <button class="nav-loc-btn" data-loc="${loc.name}" 
                        style="background: rgba(59, 130, 246, 0.2); border-color: rgba(59, 130, 246, 0.4); color: #3b82f6">
                        ${loc.icon} ${loc.name}
                    </button>
                `).join('')}
            </div>
            <div style="display: flex; gap: 8px;">
                <input type="text" class="nav-input" placeholder="${navData.placeholder[lang] || navData.placeholder.th}" 
                    style="flex: 1; padding: 10px; border: 1px solid rgba(255,255,255,0.2); border-radius: 8px; background: rgba(0,0,0,0.3); color: white;">
                <button class="nav-search-btn" style="padding: 10px 15px; background: #3b82f6; border: none; border-radius: 8px; color: white; cursor: pointer;">
                    🗺️
                </button>
            </div>
        `;

        const widget = this._showWidget(navData.title[lang] || navData.title.th, content);

        const navigate = (place) => {
            const text = `นำทางไป ${place}`;
            this.callbacks.onSendMessage?.(text);
            this._closeWidget(widget);
        };

        widget.querySelectorAll('.nav-loc-btn').forEach(btn => {
            btn.addEventListener('click', () => navigate(btn.dataset.loc));
        });

        const input = widget.querySelector('.nav-input');
        const searchBtn = widget.querySelector('.nav-search-btn');

        searchBtn.addEventListener('click', () => {
            if (input.value.trim()) navigate(input.value.trim());
        });

        input.addEventListener('keypress', (e) => {
            if (e.key === 'Enter' && input.value.trim()) navigate(input.value.trim());
        });
    }

    /**
     * Show Calculator Widget (Scientific)
     */
    showCalcWidget() {
        const lang = localStorage.getItem('app_language') || 'th';

        // i18n for Calculator Widget
        const calcTitle = {
            th: '🔢 เครื่องคิดเลขวิทยาศาสตร์',
            en: '🔢 Scientific Calculator',
            ja: '🔢 関数電卓',
            zh: '🔢 科学计算器',
            ru: '🔢 Научный калькулятор',
            hi: '🔢 वैज्ञानिक कैलकुलेटर',
            ms: '🔢 Kalkulator Saintifik'
        };

        const widget = document.createElement('div');
        widget.className = 'fab-widget';
        widget.style.width = '360px';
        widget.style.padding = '0';

        widget.innerHTML = `
            <div style="background: linear-gradient(135deg, #1e293b, #0f172a); padding: 15px; border-radius: 16px;">
                <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px;">
                    <h3 style="margin: 0; font-size: 1rem; color: #10b981;">${calcTitle[lang] || calcTitle.th}</h3>
                    <button class="fab-widget-close" style="background: none; border: none; color: #aaa; font-size: 1.2rem; cursor: pointer;">✕</button>
                </div>
                
                <!-- Display -->
                <div class="calc-display" style="
                    background: #0f172a;
                    border: 1px solid rgba(16, 185, 129, 0.3);
                    border-radius: 8px;
                    padding: 12px;
                    margin-bottom: 12px;
                    text-align: right;
                    font-family: 'Consolas', monospace;
                ">
                    <div class="calc-expression" style="font-size: 0.8rem; color: #64748b; min-height: 18px;"></div>
                    <div class="calc-result" style="font-size: 1.8rem; color: #f1f5f9; font-weight: bold;">0</div>
                </div>
                
                <!-- Scientific Row 1 -->
                <div style="display: grid; grid-template-columns: repeat(5, 1fr); gap: 6px; margin-bottom: 6px;">
                    <button class="calc-btn sci" data-func="sin" style="background: rgba(96, 165, 250, 0.2); color: #60a5fa;">sin</button>
                    <button class="calc-btn sci" data-func="cos" style="background: rgba(96, 165, 250, 0.2); color: #60a5fa;">cos</button>
                    <button class="calc-btn sci" data-func="tan" style="background: rgba(96, 165, 250, 0.2); color: #60a5fa;">tan</button>
                    <button class="calc-btn sci" data-func="log" style="background: rgba(96, 165, 250, 0.2); color: #60a5fa;">log</button>
                    <button class="calc-btn sci" data-func="ln" style="background: rgba(96, 165, 250, 0.2); color: #60a5fa;">ln</button>
                </div>
                
                <!-- Scientific Row 2 -->
                <div style="display: grid; grid-template-columns: repeat(5, 1fr); gap: 6px; margin-bottom: 6px;">
                    <button class="calc-btn sci" data-func="sqrt" style="background: rgba(168, 85, 247, 0.2); color: #a78bfa;">√</button>
                    <button class="calc-btn sci" data-func="sq" style="background: rgba(168, 85, 247, 0.2); color: #a78bfa;">x²</button>
                    <button class="calc-btn sci" data-func="pow" style="background: rgba(168, 85, 247, 0.2); color: #a78bfa;">xʸ</button>
                    <button class="calc-btn sci" data-val="pi" style="background: rgba(168, 85, 247, 0.2); color: #a78bfa;">π</button>
                    <button class="calc-btn sci" data-val="e" style="background: rgba(168, 85, 247, 0.2); color: #a78bfa;">e</button>
                </div>
                
                <!-- Scientific Row 3 -->
                <div style="display: grid; grid-template-columns: repeat(5, 1fr); gap: 6px; margin-bottom: 6px;">
                    <button class="calc-btn" data-val="(" style="background: rgba(148, 163, 184, 0.15); color: #94a3b8;">(</button>
                    <button class="calc-btn" data-val=")" style="background: rgba(148, 163, 184, 0.15); color: #94a3b8;">)</button>
                    <button class="calc-btn sci" data-func="inv" style="background: rgba(96, 165, 250, 0.2); color: #60a5fa;">1/x</button>
                    <button class="calc-btn sci" data-func="abs" style="background: rgba(96, 165, 250, 0.2); color: #60a5fa;">|x|</button>
                    <button class="calc-btn" data-val="backspace" style="background: rgba(239, 68, 68, 0.2); color: #f87171;">⌫</button>
                </div>
                
                <!-- Main Buttons -->
                <div style="display: grid; grid-template-columns: repeat(4, 1fr); gap: 6px;">
                    <button class="calc-btn" data-val="C" style="background: rgba(239, 68, 68, 0.3); color: #f87171;">C</button>
                    <button class="calc-btn" data-val="±" style="background: rgba(148, 163, 184, 0.2); color: #94a3b8;">±</button>
                    <button class="calc-btn" data-val="%" style="background: rgba(148, 163, 184, 0.2); color: #94a3b8;">%</button>
                    <button class="calc-btn op" data-val="/" style="background: rgba(251, 191, 36, 0.3); color: #fbbf24;">÷</button>
                    
                    <button class="calc-btn num" data-val="7">7</button>
                    <button class="calc-btn num" data-val="8">8</button>
                    <button class="calc-btn num" data-val="9">9</button>
                    <button class="calc-btn op" data-val="*" style="background: rgba(251, 191, 36, 0.3); color: #fbbf24;">×</button>
                    
                    <button class="calc-btn num" data-val="4">4</button>
                    <button class="calc-btn num" data-val="5">5</button>
                    <button class="calc-btn num" data-val="6">6</button>
                    <button class="calc-btn op" data-val="-" style="background: rgba(251, 191, 36, 0.3); color: #fbbf24;">−</button>
                    
                    <button class="calc-btn num" data-val="1">1</button>
                    <button class="calc-btn num" data-val="2">2</button>
                    <button class="calc-btn num" data-val="3">3</button>
                    <button class="calc-btn op" data-val="+" style="background: rgba(251, 191, 36, 0.3); color: #fbbf24;">+</button>
                    
                    <button class="calc-btn num" data-val="0" style="grid-column: span 2;">0</button>
                    <button class="calc-btn num" data-val=".">.</button>
                    <button class="calc-btn eq" data-val="=" style="background: linear-gradient(135deg, #10b981, #059669); color: white;">=</button>
                </div>
            </div>
        `;

        // Add styles
        const btnStyle = document.createElement('style');
        btnStyle.textContent = `
            .calc-btn {
                padding: 12px 8px;
                border: none;
                border-radius: 8px;
                font-size: 1rem;
                font-weight: 600;
                cursor: pointer;
                transition: all 0.15s;
            }
            .calc-btn.num {
                background: rgba(255, 255, 255, 0.1);
                color: #f1f5f9;
            }
            .calc-btn.sci {
                font-size: 0.85rem;
                padding: 10px 6px;
            }
            .calc-btn:hover {
                transform: scale(1.05);
                filter: brightness(1.2);
            }
            .calc-btn:active {
                transform: scale(0.95);
            }
        `;
        widget.appendChild(btnStyle);

        this._closeAllWidgets();
        this.close();
        document.body.appendChild(widget);

        // Calculator logic
        let currentValue = '0';
        let previousValue = '';
        let operator = '';
        let shouldReset = false;

        const expressionEl = widget.querySelector('.calc-expression');
        const resultEl = widget.querySelector('.calc-result');

        const format = (num) => {
            if (typeof num !== 'number' || isNaN(num)) return 'Error';
            if (!isFinite(num)) return num > 0 ? '∞' : '-∞';
            return parseFloat(num.toFixed(10)).toString();
        };

        const updateDisplay = () => {
            resultEl.textContent = currentValue.length > 15 ? parseFloat(currentValue).toExponential(5) : currentValue;
            if (previousValue && operator) {
                const opSymbol = { '+': '+', '-': '−', '*': '×', '/': '÷', 'pow': '^' }[operator] || operator;
                expressionEl.textContent = `${previousValue} ${opSymbol}`;
            } else {
                expressionEl.textContent = '';
            }
        };

        const calculate = () => {
            if (!previousValue || !operator) return;
            const prev = parseFloat(previousValue);
            const curr = parseFloat(currentValue);
            let result;
            switch (operator) {
                case '+': result = prev + curr; break;
                case '-': result = prev - curr; break;
                case '*': result = prev * curr; break;
                case '/': result = curr !== 0 ? prev / curr : NaN; break;
                case 'pow': result = Math.pow(prev, curr); break;
                default: return;
            }
            currentValue = format(result);
            previousValue = '';
            operator = '';
            shouldReset = true;
        };

        const applyFunc = (func) => {
            const num = parseFloat(currentValue);
            let result;
            switch (func) {
                case 'sin': result = Math.sin(num * Math.PI / 180); break;
                case 'cos': result = Math.cos(num * Math.PI / 180); break;
                case 'tan': result = Math.abs(num % 180) === 90 ? NaN : Math.tan(num * Math.PI / 180); break;
                case 'log': result = num > 0 ? Math.log10(num) : NaN; break;
                case 'ln': result = num > 0 ? Math.log(num) : NaN; break;
                case 'sqrt': result = num >= 0 ? Math.sqrt(num) : NaN; break;
                case 'sq': result = num * num; break;
                case 'inv': result = num !== 0 ? 1 / num : NaN; break;
                case 'abs': result = Math.abs(num); break;
                default: return;
            }
            currentValue = format(result);
            shouldReset = true;
            updateDisplay();
        };

        widget.querySelectorAll('.calc-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const val = btn.dataset.val;
                const func = btn.dataset.func;

                // Scientific function
                if (func) {
                    if (func === 'pow') {
                        if (previousValue && operator) calculate();
                        previousValue = currentValue;
                        operator = 'pow';
                        shouldReset = true;
                        updateDisplay();
                    } else {
                        applyFunc(func);
                    }
                    return;
                }

                // Clear
                if (val === 'C') { currentValue = '0'; previousValue = ''; operator = ''; updateDisplay(); return; }
                // Backspace
                if (val === 'backspace') { currentValue = currentValue.length > 1 ? currentValue.slice(0, -1) : '0'; updateDisplay(); return; }
                // Toggle sign
                if (val === '±') { currentValue = format(parseFloat(currentValue) * -1); updateDisplay(); return; }
                // Percent
                if (val === '%') { currentValue = format(parseFloat(currentValue) / 100); updateDisplay(); return; }
                // Constants
                if (val === 'pi') { currentValue = Math.PI.toString(); shouldReset = true; updateDisplay(); return; }
                if (val === 'e') { currentValue = Math.E.toString(); shouldReset = true; updateDisplay(); return; }
                // Parentheses
                if (val === '(' || val === ')') { currentValue = shouldReset && val === '(' ? '(' : currentValue + val; shouldReset = false; updateDisplay(); return; }
                // Operators
                if (['+', '-', '*', '/'].includes(val)) {
                    if (previousValue && operator) calculate();
                    previousValue = currentValue;
                    operator = val;
                    shouldReset = true;
                    updateDisplay();
                    return;
                }
                // Equals
                if (val === '=') { calculate(); updateDisplay(); return; }
                // Numbers
                if (shouldReset) { currentValue = val === '.' ? '0.' : val; shouldReset = false; }
                else {
                    if (val === '.' && currentValue.includes('.')) return;
                    currentValue = currentValue === '0' && val !== '.' ? val : currentValue + val;
                }
                updateDisplay();
            });
        });

        widget.querySelector('.fab-widget-close').addEventListener('click', () => this._closeWidget(widget));
    }
}

// Singleton
export const fabManager = new FabManager();
export default fabManager;
