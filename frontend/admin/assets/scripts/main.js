/**
 * /frontend/assets/scripts/main.js
 * =====================================
 * ศูนย์กลาง Frontend - เหมือน main.py ฝั่ง Backend
 * จัดการ config, constants, และ shared functions
 * =====================================
 */

// ========================================
// 🔧 INTENT CONSTANTS
// ========================================
// ใช้แทนการให้ LLM วิเคราะห์เจตนา
const INTENTS = {
    GENERAL: 'GENERAL',           // คำถามทั่วไป → RAG
    MUSIC: 'MUSIC',               // ขอฟังเพลง
    NAVIGATION: 'NAVIGATION',     // นำทางไปสถานที่
    CALCULATOR: 'CALCULATOR',     // เปิดเครื่องคิดเลข
    FAQ: 'FAQ',                   // คำถามที่พบบ่อย
    WELCOME: 'WELCOME',           // ทักทาย
};

// ========================================
// 🌐 API CONFIG (Import from config.js)
// ========================================
// config.js ถูก load ก่อน main.js ใน HTML
// ดังนั้น API_BASE_URL และ WS_BASE_URL พร้อมใช้งาน

// ========================================
// 📡 WEBSOCKET MANAGER
// ========================================
class WebSocketManager {
    constructor() {
        this.ws = null;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.onMessageCallback = null;
        this.onConnectCallback = null;
        this.onDisconnectCallback = null;
    }

    connect(endpoint = '/api/chat/ws') {
        const wsUrl = `${WS_BASE_URL}${endpoint}`;
        console.log(`🔌 [WS] Connecting to: ${wsUrl}`);

        this.ws = new WebSocket(wsUrl);

        this.ws.onopen = () => {
            console.log('✅ [WS] Connected');
            this.reconnectAttempts = 0;
            if (this.onConnectCallback) this.onConnectCallback();
        };

        this.ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                if (this.onMessageCallback) this.onMessageCallback(data);
            } catch (e) {
                console.error('❌ [WS] Parse error:', e);
            }
        };

        this.ws.onclose = () => {
            console.log('🔌 [WS] Disconnected');
            if (this.onDisconnectCallback) this.onDisconnectCallback();
            this._tryReconnect(endpoint);
        };

        this.ws.onerror = (error) => {
            console.error('❌ [WS] Error:', error);
        };

        return this;
    }

    _tryReconnect(endpoint) {
        if (this.reconnectAttempts < this.maxReconnectAttempts) {
            this.reconnectAttempts++;
            console.log(`🔄 [WS] Reconnecting... (${this.reconnectAttempts}/${this.maxReconnectAttempts})`);
            setTimeout(() => this.connect(endpoint), 2000 * this.reconnectAttempts);
        }
    }

    send(query, intent = INTENTS.GENERAL, aiMode = 'fast') {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            const payload = {
                query: query,
                intent: intent,
                ai_mode: aiMode
            };
            console.log(`📤 [WS] Sending:`, payload);
            this.ws.send(JSON.stringify(payload));
            return true;
        }
        console.warn('⚠️ [WS] Not connected');
        return false;
    }

    onMessage(callback) {
        this.onMessageCallback = callback;
        return this;
    }

    onConnect(callback) {
        this.onConnectCallback = callback;
        return this;
    }

    onDisconnect(callback) {
        this.onDisconnectCallback = callback;
        return this;
    }

    isConnected() {
        return this.ws && this.ws.readyState === WebSocket.OPEN;
    }
}

// ========================================
// 🎛️ AI MODE MANAGER
// ========================================
class AIModeManager {
    constructor() {
        this.mode = localStorage.getItem('nan_ai_mode') || 'fast';
    }

    getMode() {
        return this.mode;
    }

    setMode(mode) {
        this.mode = mode;
        localStorage.setItem('nan_ai_mode', mode);
        console.log(`🤖 [AI Mode] Set to: ${mode}`);
    }

    toggle() {
        this.mode = this.mode === 'fast' ? 'detailed' : 'fast';
        localStorage.setItem('nan_ai_mode', this.mode);
        console.log(`🤖 [AI Mode] Toggled to: ${this.mode}`);
        return this.mode;
    }

    isFast() {
        return this.mode === 'fast';
    }

    isDetailed() {
        return this.mode === 'detailed';
    }
}

// ========================================
// 🛠️ UTILITY FUNCTIONS
// ========================================
const Utils = {
    // Generate unique ID
    generateId: (prefix = 'id') => `${prefix}-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,

    // Format date
    formatDate: (date) => {
        const d = new Date(date);
        return d.toLocaleDateString('th-TH', {
            year: 'numeric',
            month: 'long',
            day: 'numeric'
        });
    },

    // Debounce function
    debounce: (func, wait) => {
        let timeout;
        return function executedFunction(...args) {
            const later = () => {
                clearTimeout(timeout);
                func(...args);
            };
            clearTimeout(timeout);
            timeout = setTimeout(later, wait);
        };
    },

    // Check if element exists
    $(selector) {
        return document.querySelector(selector);
    },

    $$(selector) {
        return document.querySelectorAll(selector);
    }
};

// ========================================
// 🌍 GLOBAL EXPORTS
// ========================================
// Export to window for use in other JS files
window.NanApp = {
    INTENTS,
    WebSocketManager,
    AIModeManager,
    Utils,

    // Easy access instances (created lazily)
    _wsManager: null,
    _aiModeManager: null,

    getWebSocketManager() {
        if (!this._wsManager) {
            this._wsManager = new WebSocketManager();
        }
        return this._wsManager;
    },

    getAIModeManager() {
        if (!this._aiModeManager) {
            this._aiModeManager = new AIModeManager();
        }
        return this._aiModeManager;
    }
};

console.log('✅ [Main.js] NanApp initialized');
console.log('📋 Available intents:', Object.keys(INTENTS));
