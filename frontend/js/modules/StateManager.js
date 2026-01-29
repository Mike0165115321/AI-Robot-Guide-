/**
 * # StateManager.js
 * Centralized state management for the application.
 * Follows the Singleton pattern.
 */

class StateManager {
    constructor() {
        if (StateManager.instance) {
            return StateManager.instance;
        }

        this.state = {
            sessionId: this.generateUUID(),
            isRecording: false,
            isSpeaking: false,
            currentMode: 'voice', // 'voice' | 'chat'
            silenceTimer: null, // For auto-submit in voice mode
            longPressTimer: null,
            isLongPress: false,
            avatarMood: 'neutral',
            alertTTSEnabled: localStorage.getItem('alertTTSEnabled') === 'true' // 🆕 เปิด/ปิดเสียงแจ้งเตือน (ค่าเริ่มต้น: ปิด)
        };

        StateManager.instance = this;
    }

    /**
     * Get the entire state object (read-only recommended)
     */
    getState() {
        return this.state;
    }

    /**
     * Get a specific state value
     */
    get(key) {
        return this.state[key];
    }

    /**
     * Set a specific state value
     */
    set(key, value) {
        if (key in this.state) {
            this.state[key] = value;
            // Trigger listeners if we implement them later
        } else {
            console.warn(`[StateManager] Key '${key}' does not exist in initial state.`);
            this.state[key] = value;
        }
    }

    /**
     * Helper: Generate a unique session ID
     */
    generateUUID() {
        return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function (c) {
            var r = Math.random() * 16 | 0, v = c == 'x' ? r : (r & 0x3 | 0x8);
            return v.toString(16);
        });
    }

    /**
     * Reset specific operative flags (useful after processing)
     */
    resetOperationalFlags() {
        this.state.isRecording = false;
        this.state.isSpeaking = false;
        this.state.isLongPress = false;
    }
}

// Export a singleton instance
const stateManager = new StateManager();
export default stateManager;
