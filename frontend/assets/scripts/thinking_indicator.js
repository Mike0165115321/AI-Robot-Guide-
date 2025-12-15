/**
 * thinking_indicator.js
 * แสดงข้อความชั่วคราวขณะ AI กำลังคิด
 * ให้รู้สึกมีชีวิตชีวา ไม่ดูไร้ชีวิต
 */

class ThinkingIndicator {
    constructor() {
        // ข้อความที่จะสลับแสดง (น่ารัก กระตือรือร้น)
        this.messages = [
            "🤔 แปบนึงนะคะ กำลังคิดอยู่ค่า~",
            "💭 รอแปบน้าา... น้องน่านกำลังหาข้อมูลให้",
            "✨ เดี๋ยวนะคะ กำลังรวบรวมข้อมูลดีๆ",
            "🔍 ค้นหาอยู่ค่ะ... อีกนิดเดียว!",
            "📚 อ่านข้อมูลอยู่นะคะ ใจเย็นๆ",
            "🧠 คิดอยู่ค่ะ~ รอหน่อยนะ",
            "⏳ แปบเดียวค่า... เกือบเสร็จแล้ว!",
            "🌟 กำลังเตรียมคำตอบให้เจ้า~",
            "💡 อืม... น่าสนใจ! รอแปบนะ",
            "🎯 หาคำตอบที่ดีที่สุดให้อยู่ค่ะ"
        ];

        // Fun emoji animations
        this.thinkingEmojis = ["🤔", "💭", "✨", "🔍", "📚", "🧠", "⏳", "🌟", "💡", "🎯"];

        this.element = null;
        this.messageInterval = null;
        this.emojiInterval = null;
        this.currentIndex = 0;
        this.isActive = false;
        this.timerInterval = null; // ⏱️ Timer variable
        this.startTime = 0;        // ⏱️ Start timestamp
    }

    /**
     * สร้าง HTML element สำหรับ indicator
     */
    _createIndicatorElement() {
        const container = document.createElement('div');
        container.className = 'thinking-indicator-container';
        container.id = 'thinking-indicator';

        container.innerHTML = `
            <div class="thinking-bubble">
                <div class="thinking-avatar">
                    <span class="thinking-emoji">🤔</span>
                </div>
                <div class="thinking-content">
                    <div class="thinking-row" style="display: flex; align-items: center; gap: 8px;">
                        <div class="thinking-dots">
                            <span></span><span></span><span></span>
                        </div>
                        <span class="thinking-timer">0.0s</span> <!-- ⏱️ Timer Element -->
                    </div>
                    <p class="thinking-message">${this.messages[0]}</p>
                </div>
            </div>
        `;

        // Add styles (Force update to ensure new CSS classes are applied)
        let style = document.getElementById('thinking-indicator-styles');
        if (!style) {
            style = document.createElement('style');
            style.id = 'thinking-indicator-styles';
            document.head.appendChild(style);
        }
        style.textContent = `
                .thinking-indicator-container {
                    display: flex;
                    justify-content: flex-start;
                    padding: 10px 0;
                    animation: fadeInUp 0.3s ease-out;
                }

                @keyframes fadeInUp {
                    from {
                        opacity: 0;
                        transform: translateY(10px);
                    }
                    to {
                        opacity: 1;
                        transform: translateY(0);
                    }
                }

                .thinking-bubble {
                    display: flex;
                    align-items: flex-start;
                    gap: 12px;
                    background: linear-gradient(135deg, rgba(16, 185, 129, 0.15), rgba(59, 130, 246, 0.1));
                    border: 1px solid rgba(16, 185, 129, 0.3);
                    border-radius: 20px;
                    padding: 16px 20px;
                    max-width: 350px;
                    box-shadow: 0 4px 15px rgba(0, 0, 0, 0.2);
                    backdrop-filter: blur(10px);
                }

                .thinking-avatar {
                    width: 40px;
                    height: 40px;
                    background: linear-gradient(135deg, #10b981, #059669);
                    border-radius: 50%;
                    display: flex;
                    align-items: center;
                    justify-content: center;
                    flex-shrink: 0;
                    animation: pulse-glow 2s infinite;
                }

                @keyframes pulse-glow {
                    0%, 100% {
                        box-shadow: 0 0 10px rgba(16, 185, 129, 0.5);
                    }
                    50% {
                        box-shadow: 0 0 20px rgba(16, 185, 129, 0.8), 0 0 30px rgba(16, 185, 129, 0.4);
                    }
                }

                .thinking-emoji {
                    font-size: 1.3rem;
                    animation: bounce 1s infinite;
                }

                @keyframes bounce {
                    0%, 100% { transform: translateY(0); }
                    50% { transform: translateY(-3px); }
                }

                .thinking-content {
                    display: flex;
                    flex-direction: column;
                    gap: 6px;
                }

                .thinking-dots {
                    display: flex;
                    gap: 4px;
                }

                .thinking-dots span {
                    width: 8px;
                    height: 8px;
                    background: #10b981;
                    border-radius: 50%;
                    animation: dot-pulse 1.4s infinite ease-in-out;
                }

                .thinking-dots span:nth-child(1) { animation-delay: 0s; }
                .thinking-dots span:nth-child(2) { animation-delay: 0.2s; }
                .thinking-dots span:nth-child(3) { animation-delay: 0.4s; }

                @keyframes dot-pulse {
                    0%, 80%, 100% {
                        transform: scale(0.6);
                        opacity: 0.5;
                    }
                    40% {
                        transform: scale(1);
                        opacity: 1;
                    }
                }

                .thinking-message {
                    margin: 0;
                    font-size: 0.95rem;
                    color: #e2e8f0;
                    font-family: 'Kanit', sans-serif;
                    animation: textFade 0.3s ease-out;
                }

                @keyframes textFade {
                    from { opacity: 0; }
                    to { opacity: 1; }
                }

                .thinking-indicator-container.hiding {
                    animation: fadeOutDown 0.3s ease-out forwards;
                }

                @keyframes fadeOutDown {
                    from {
                        opacity: 1;
                        transform: translateY(0);
                    }
                    to {
                        opacity: 0;
                        transform: translateY(10px);
                    }
                }

                .thinking-timer {
                    font-size: 0.75rem;
                    color: #94a3b8;
                    font-family: 'Courier New', monospace;
                    background: rgba(0, 0, 0, 0.1);
                    padding: 2px 6px;
                    border-radius: 8px;
                    font-weight: bold;
                }
            `;

        return container;
    }

    /**
     * เริ่มแสดง indicator
     * @param {HTMLElement} container - container ที่จะใส่ indicator (เช่น messageArea)
     */
    show(container) {
        if (this.isActive) return;
        this.isActive = true;

        // สุ่มข้อความเริ่มต้น
        this.currentIndex = Math.floor(Math.random() * this.messages.length);

        // สร้าง element
        this.element = this._createIndicatorElement();
        container.appendChild(this.element);

        // Scroll to bottom
        container.scrollTop = container.scrollHeight;

        // เริ่มสลับข้อความทุก 2.5 วินาที
        this.messageInterval = setInterval(() => {
            this._updateMessage();
        }, 2500);

        // สลับ emoji ทุก 1.5 วินาที
        this.emojiInterval = setInterval(() => {
            this._updateEmoji();
        }, 1500);

        // ⏱️ Start Real-time Timer
        this.startTime = Date.now();
        this.timerInterval = setInterval(() => {
            if (this.element) {
                const elapsed = (Date.now() - this.startTime) / 1000;
                const timerEl = this.element.querySelector('.thinking-timer');
                if (timerEl) {
                    timerEl.textContent = `${elapsed.toFixed(1)}s`;
                }
            }
        }, 100);
    }

    /**
     * อัพเดทข้อความ
     */
    _updateMessage() {
        if (!this.element) return;

        // สุ่มข้อความใหม่ (ไม่ซ้ำกับข้อความปัจจุบัน)
        let newIndex;
        do {
            newIndex = Math.floor(Math.random() * this.messages.length);
        } while (newIndex === this.currentIndex && this.messages.length > 1);

        this.currentIndex = newIndex;

        const messageEl = this.element.querySelector('.thinking-message');
        if (messageEl) {
            messageEl.style.animation = 'none';
            messageEl.offsetHeight; // Force reflow
            messageEl.style.animation = 'textFade 0.3s ease-out';
            messageEl.textContent = this.messages[this.currentIndex];
        }
    }

    /**
     * อัพเดท emoji
     */
    _updateEmoji() {
        if (!this.element) return;

        const emojiEl = this.element.querySelector('.thinking-emoji');
        if (emojiEl) {
            const randomEmoji = this.thinkingEmojis[Math.floor(Math.random() * this.thinkingEmojis.length)];
            emojiEl.textContent = randomEmoji;
        }
    }

    /**
     * ซ่อน indicator
     */
    hide() {
        if (!this.isActive || !this.element) return;

        // หยุด intervals
        if (this.messageInterval) {
            clearInterval(this.messageInterval);
            this.messageInterval = null;
        }
        if (this.emojiInterval) {
            clearInterval(this.emojiInterval);
            this.emojiInterval = null;
        }
        if (this.timerInterval) {
            clearInterval(this.timerInterval);
            this.timerInterval = null;
        }

        // Animation ออก
        this.element.classList.add('hiding');

        setTimeout(() => {
            if (this.element && this.element.parentNode) {
                this.element.parentNode.removeChild(this.element);
            }
            this.element = null;
            this.isActive = false;
        }, 300);
    }

    /**
     * เพิ่มข้อความ custom
     */
    addCustomMessage(message) {
        if (!this.messages.includes(message)) {
            this.messages.push(message);
        }
    }
}

// Export as global singleton
window.ThinkingIndicator = new ThinkingIndicator();
