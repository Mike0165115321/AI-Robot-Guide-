/**
 * AudioStreamManager.js
 * A "Smart" Audio Controller for handling streaming TTS chunks.
 * 
 * Features:
 * - Managed Audio Queue (Buffer)
 * - State Machine (IDLE, PLAYING, INTERRUPTED)
 * - Robust Error Handling
 * - Lip Sync Event Emission
 */

class AudioStreamManager {
    constructor() {
        this.audioQueue = []; // The Buffer
        this.isPlaying = false;
        this.state = 'IDLE'; // IDLE, PLAYING, INTERRUPTED
        this.audioContext = null;
        this.currentSource = null;

        this.onStartSpeaking = null; // Callback for Avatar Lip Sync
        this.onStopSpeaking = null;  // Callback for Avatar Lip Sync
        this.onStatusChange = null;  // Callback for status text

        this._initContext();
    }

    _initContext() {
        if (!this.audioContext) {
            const AudioContext = window.AudioContext || window.webkitAudioContext;
            this.audioContext = new AudioContext();
        }
    }

    /**
     * Enqueue a raw audio chunk (ArrayBuffer/Blob) from WebSocket
     */
    async enqueue(chunk) {
        // 🧠 Brain Logic: If we are INTERRUPTED, we still "accept" the chunk 
        // to keep the network flow passing, but we silently discard it
        // so it doesn't build up in memory for no reason.
        if (this.state === 'INTERRUPTED') {
            console.log("AudioStreamManager: Dropping chunk (Interrupted State)");
            return;
        }

        // Standard Queue
        this.audioQueue.push(chunk);
        this.processQueue();
    }

    /**
     * Process the queue one by one
     */
    async processQueue() {
        if (this.isPlaying || this.audioQueue.length === 0) return;

        // Safety check
        if (this.state === 'INTERRUPTED') {
            this.clear();
            return;
        }

        this.isPlaying = true;
        this.state = 'PLAYING';

        const chunk = this.audioQueue.shift();

        try {
            await this._playChunk(chunk);
        } catch (e) {
            console.error("AudioStreamManager: Play error", e);
            this.isPlaying = false;
            this.processQueue(); // Try next
        }
    }

    async _playChunk(chunk) {
        if (this.audioContext.state === 'suspended') {
            await this.audioContext.resume();
        }

        // Decode
        const buffer = await (chunk instanceof Blob ? chunk.arrayBuffer() : chunk);
        const audioBuffer = await this.audioContext.decodeAudioData(buffer);

        // Create Source
        const source = this.audioContext.createBufferSource();
        source.buffer = audioBuffer;
        source.connect(this.audioContext.destination);

        this.currentSource = source;

        // 👄 Lip Sync Start
        if (this.onStartSpeaking) this.onStartSpeaking();

        return new Promise((resolve) => {
            source.onended = () => {
                this.currentSource = null;
                this.isPlaying = false;

                // Check if more items in queue
                if (this.audioQueue.length === 0) {
                    // 👄 Lip Sync Stop (Idle) if no more chunks
                    // Delay slightly to prevent jitter
                    setTimeout(() => {
                        if (this.audioQueue.length === 0 && !this.isPlaying) {
                            if (this.onStopSpeaking) this.onStopSpeaking();
                            this.state = 'IDLE';
                        }
                    }, 100);
                }

                // Continue loop
                resolve();
                this.processQueue();
            };
            source.start(0);
        });
    }

    /**
     * Stop Everything (User Pressed Stop)
     * - Clears Queue
     * - Stops Current Audio
     * - Sets State to INTERRUPTED (ignoring incoming chunks until reset)
     */
    stop() {
        console.log("AudioStreamManager: STOP Command Received");
        this.state = 'INTERRUPTED';
        this.clear();

        if (this.onStopSpeaking) this.onStopSpeaking();
    }

    /**
     * Reset for New Conversation
     * - Sets State to IDLE (ready to accept chunks)
     * - Clears any old residue
     */
    reset() {
        console.log("AudioStreamManager: RESET (Ready for new stream)");
        this.state = 'IDLE';
        this.clear();
    }

    clear() {
        this.audioQueue = [];
        if (this.currentSource) {
            try { this.currentSource.stop(); } catch (e) { }
            this.currentSource = null;
        }
        this.isPlaying = false;
    }
}

// Export global instance
window.audioStreamManager = new AudioStreamManager();
