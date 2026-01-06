/**
 * # AI Mode Manager
 * 
 * จัดการโหมด AI (Fast / Detailed)
 * - Fast: ใช้ Llama - ตอบเร็ว
 * - Detailed: ใช้ Gemini - ตอบละเอียด
 * 
 * @example
 * const mode = aiModeManager.getMode(); // 'fast' | 'detailed'
 * aiModeManager.toggle();
 * aiModeManager.setMode('detailed');
 */

class AIModeManager {
    constructor() {
        // Load from localStorage or default to 'fast'
        this.mode = localStorage.getItem('ai_mode') || 'fast';
        this.callbacks = [];
    }

    /**
     * Get current mode
     */
    getMode() {
        return this.mode;
    }

    /**
     * Set mode
     */
    setMode(mode) {
        if (mode !== 'fast' && mode !== 'detailed') {
            console.warn('AIModeManager: Invalid mode', mode);
            return this.mode;
        }

        this.mode = mode;
        localStorage.setItem('ai_mode', mode);
        this._notifyCallbacks();

        console.log(`🧠 AI Mode: ${mode === 'fast' ? '⚡ Fast (Llama)' : '🧠 Detailed (Gemini)'}`);
        return this.mode;
    }

    /**
     * Toggle between modes
     */
    toggle() {
        return this.setMode(this.mode === 'fast' ? 'detailed' : 'fast');
    }

    /**
     * Register callback for mode changes
     */
    onChange(callback) {
        this.callbacks.push(callback);
    }

    /**
     * Notify all callbacks
     * @private
     */
    _notifyCallbacks() {
        this.callbacks.forEach(cb => cb(this.mode));
    }

    /**
     * Get mode info for display
     */
    getModeInfo() {
        if (this.mode === 'fast') {
            return {
                mode: 'fast',
                icon: '⚡',
                label: 'คิดเร็ว',
                description: 'ใช้ Llama - ตอบเร็วทันใจ',
                color: '#fbbf24' // Yellow
            };
        }
        return {
            mode: 'detailed',
            icon: '🧠',
            label: 'คิดละเอียด',
            description: 'ใช้ Gemini - ตอบครบถ้วน',
            color: '#8b5cf6' // Purple
        };
    }
}

// Singleton
export const aiModeManager = new AIModeManager();
export default aiModeManager;
