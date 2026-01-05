/**
 * AvatarStateManager - Finite State Machine for Robot Avatar
 * Helps manage the transitions between Idle, Listening, Thinking, and Speaking.
 */
class AvatarStateManager {
    constructor(callbacks) {
        this.STATES = {
            IDLE: 'IDLE',         // Waiting for Wake Word
            LISTENING: 'LISTENING', // Mic active, recording user input
            THINKING: 'THINKING',   // Processing query (Mic Muted)
            SPEAKING: 'SPEAKING'    // Avatar talking (Mic Muted, Audio Playing)
        };

        this.currentState = this.STATES.IDLE;
        this.callbacks = callbacks || {};

        // Watchdog Timer
        this.watchdogTimer = null;
        this.WATCHDOG_TIMEOUTS = {
            [this.STATES.THINKING]: 60000, // 60s timeout for thinking
            [this.STATES.LISTENING]: 15000 // 15s timeout for silence
        };

        console.log("[FSM] AvatarStateManager Initialized in IDLE state.");
    }

    getState() {
        return this.currentState;
    }

    transitionTo(newState, data = {}) {
        if (this.currentState === newState) {
            console.warn(`[FSM] Already in state ${newState}, ignoring transition.`);
            return;
        }

        console.log(`[FSM] Transition: ${this.currentState} -> ${newState}`);

        // 1. Exit Action (Cleanup previous state)
        this._handleExitState(this.currentState);

        // 2. Change State
        this.currentState = newState;

        // 3. Enter Action (Setup new state)
        this._handleEnterState(newState, data);

        // 4. Start Watchdog (if applicable)
        this._startWatchdog(newState);
    }

    _handleExitState(state) {
        switch (state) {
            case this.STATES.IDLE:
                if (this.callbacks.stopWakeWord) this.callbacks.stopWakeWord();
                break;
            case this.STATES.LISTENING:
                if (this.callbacks.stopListening) this.callbacks.stopListening();
                break;
            case this.STATES.SPEAKING:
                if (this.callbacks.stopSpeaking) this.callbacks.stopSpeaking();
                break;
            // THINKING doesn't need specific exit cleanup usually
        }
        this._stopWatchdog();
    }

    _handleEnterState(state, data) {
        switch (state) {
            case this.STATES.IDLE:
                if (this.callbacks.onIdle) this.callbacks.onIdle();
                break;
            case this.STATES.LISTENING:
                if (this.callbacks.onListening) this.callbacks.onListening();
                break;
            case this.STATES.THINKING:
                if (this.callbacks.onThinking) this.callbacks.onThinking(data.text);
                break;
            case this.STATES.SPEAKING:
                if (this.callbacks.onSpeaking) this.callbacks.onSpeaking();
                break;
        }
    }

    _startWatchdog(state) {
        const timeoutMs = this.WATCHDOG_TIMEOUTS[state];
        if (timeoutMs) {
            this.watchdogTimer = setTimeout(() => {
                console.warn(`[FSM] Watchdog Timeout for state: ${state}`);
                if (this.callbacks.onError) {
                    this.callbacks.onError(`Timeout in ${state}`);
                }
                // Force reset to IDLE on timeout
                this.transitionTo(this.STATES.IDLE);
            }, timeoutMs);
        }
    }

    _stopWatchdog() {
        if (this.watchdogTimer) {
            clearTimeout(this.watchdogTimer);
            this.watchdogTimer = null;
        }
    }

    // Helper for "Force Stop" button
    forceReset() {
        console.warn("[FSM] Force Reset Triggered!");
        this.transitionTo(this.STATES.IDLE);
        if (this.callbacks.onForceStop) this.callbacks.onForceStop();
    }
}

// Global Export
window.AvatarStateManager = AvatarStateManager;
