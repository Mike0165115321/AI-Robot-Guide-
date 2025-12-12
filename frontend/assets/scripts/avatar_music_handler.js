// /frontend/assets/scripts/avatar_music_handler.js (V-Music Mod 7.1 - Fixed Controls)

class AvatarMusicHandler {
    constructor(websocket, uiController, voiceHandler, timerManager, callbacks) {
        this.websocket = websocket;
        this.uiController = uiController;
        this.voiceHandler = voiceHandler;
        this.timerManager = timerManager;
        this.callbacks = callbacks; // callbacks จะมี musicControls แล้ว

        this.isAwaitingUserInput = false;
        this.isPlayingMusic = false;

        // Initialize Music Controls
        if (this.callbacks.musicControls) {
            const playPauseBtn = this.callbacks.musicControls.querySelector('#play-pause-btn');
            const stopBtn = this.callbacks.musicControls.querySelector('#stop-btn');

            if (playPauseBtn) {
                playPauseBtn.addEventListener('click', () => {
                    // Logic to toggle play/pause (if supported by iframe API)
                    // For now, we might just re-enable speech or something, 
                    // but usually YouTube iframe API is needed for true control.
                    // Given the constraints, we might just focus on STOP.
                });
            }

            if (stopBtn) {
                stopBtn.addEventListener('click', () => {
                    console.log("[Music] Stop button clicked.");
                    this.reset(); // Stop music and reset state
                    // this.goToMusicIdleState(); // <-- REMOVED: This was re-showing the controls!

                    // Optionally restart listening loop if needed
                    if (this.callbacks.resetToListeningState) {
                        this.callbacks.resetToListeningState();
                    }
                });
            }
        }

        console.log("🎵 Avatar Music Handler initialized.");
    }

    isWaiting() {
        return this.isAwaitingUserInput;
    }

    isPlaying() {
        return this.isPlayingMusic;
    }

    reset() {
        const wasPlaying = this.isPlayingMusic;
        this.isAwaitingUserInput = false;
        this.isPlayingMusic = false;

        // 🚀 [แก้ไข] ซ่อนปุ่ม musicControls เมื่อรีเซ็ต
        if (this.callbacks.musicControls) {
            this.callbacks.musicControls.style.display = 'none';
        }

        if (wasPlaying) {
            this.uiController.exitPresentation();
        }
    }

    handleMessage(data) {
        if (data.action === 'PROMPT_FOR_SONG_INPUT') {
            console.log("[Music] Showing song search prompt.");
            this.isAwaitingUserInput = true;

            // Use centralised UI
            const inputHtml = musicPlayer.getSearchPromptHTML();
            this._renderHTML(data, inputHtml);
            this.uiController.setEmotion('listening');

            // Bind events
            setTimeout(() => {
                const container = document.querySelector('.music-search-prompt').parentElement;
                if (container) {
                    musicPlayer.bindSearchEvents(container, (song) => {
                        this._playVideoInPresentation(song, "จัดให้ตามคำขอครับ!");
                        this.goToMusicIdleState();
                    });
                    // Auto focus
                    const input = container.querySelector('input');
                    if (input) input.focus();
                }
            }, 100);

            return true;
        }
        return false;
    }

    goToMusicIdleState() {
        if (this.callbacks.stopAISpeechAudio) this.callbacks.stopAISpeechAudio();
        this.voiceHandler.stop(true);
        if (this.callbacks.stopSpeechButton) this.callbacks.stopSpeechButton.classList.remove('visible');
        if (this.callbacks.musicControls) this.callbacks.musicControls.style.display = 'flex';
        this.uiController.setEmotion('normal');
        this.uiController.setStatus("กำลังเล่นเพลง... (กดปุ่ม ⏹️ เพื่อหยุด)");
    }

    _playVideoInPresentation(song, originalAnswer) {
        console.log(`[Music] Playing video: ${song.title}`);
        this.isPlayingMusic = true;

        const infoDisplay = document.getElementById('info-display');
        const searchUI = document.querySelector('.music-search-prompt');
        if (searchUI) searchUI.style.display = 'none';

        // Container needs to be inside infoDisplay but handled carefully
        // We'll prepend a wrapper
        let playerContainer = document.getElementById('avatar-player-wrapper');
        if (!playerContainer) {
            playerContainer = document.createElement('div');
            playerContainer.id = 'avatar-player-wrapper';
            infoDisplay.prepend(playerContainer);
        }

        // Use Shared MusicPlayer to create the player
        musicPlayer.createPlayer(song, playerContainer);

        if (this.timerManager) this.timerManager.clearPresentationTimeout();
    }

    _renderHTML(data, infoHtml) {
        const resultText = document.getElementById('result-text');
        const infoDisplay = document.getElementById('info-display');
        if (resultText) resultText.innerHTML = data.answer ? (typeof marked !== 'undefined' ? marked.parse(data.answer) : data.answer) : '';
        if (infoDisplay) infoDisplay.innerHTML = infoHtml;
        data.html_is_pre_rendered = true;
        this.uiController.enterPresentation(data);
    }
}