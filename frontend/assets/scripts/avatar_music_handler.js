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
        const messageId = `msg-${Date.now()}`;

        if (data.action === 'PROMPT_FOR_SONG_INPUT') {
            console.log("[Music] Music prompt blocked by user request.");
            // Block the UI from appearing
            return true;

        } else if (data.action === 'SHOW_SONG_CHOICES' && Array.isArray(data.action_payload)) {
            console.log("[Music] Showing song choices.");
            this.isAITalking = true; // [แก้ไข] เปลี่ยนเป็น true เพราะ AI ต้องพูดก่อน

            const songsHtml = data.action_payload.map((song, index) => `
                <button class="song-choice-btn" data-song-index="${index}">
                    🎵 ${song.title.replace(/</g, "&lt;").replace(/>/g, "&gt;")}
                </button>
            `).join('');

            this._renderHTML(data, `<div style="margin-top: 20px;">${songsHtml}</div>`);

            document.querySelectorAll('.song-choice-btn').forEach(button => {
                button.addEventListener('click', () => {
                    const songIndex = parseInt(button.dataset.songIndex, 10);
                    const selectedSong = data.action_payload[songIndex];

                    this.isAwaitingUserInput = false;
                    this._playVideoInPresentation(selectedSong, data.answer);
                    this.goToMusicIdleState();
                });
            });
            return true;
        }

        return false;
    }


    goToMusicIdleState() {
        if (this.callbacks.stopAISpeechAudio) {
            this.callbacks.stopAISpeechAudio();
        }
        this.voiceHandler.stop(true);

        // 🚀 [แก้ไข] เปลี่ยนไปแสดงปุ่ม musicControls แทนปุ่มแดง
        if (this.callbacks.stopSpeechButton) {
            this.callbacks.stopSpeechButton.classList.remove('visible');
        }
        if (this.callbacks.musicControls) {
            this.callbacks.musicControls.style.display = 'flex'; // 👈 แสดงปุ่มที่ถูกต้อง
        }
        // 🚀 [สิ้นสุดการแก้ไข]

        console.log("State => Music Idle (Not Listening)");
        this.uiController.setEmotion('normal');
        this.uiController.setStatus("กำลังเล่นเพลง... (กดปุ่ม ⏹️ เพื่อหยุด)");
    }

    _playVideoInPresentation(song, originalAnswer) {
        console.log(`[Music] Playing video: ${song.title}`);
        this.isPlayingMusic = true;

        const answerHtml = (typeof marked !== 'undefined' ? marked.parse(originalAnswer) : originalAnswer) + `<p>กำลังเล่นเพลง: <strong>${song.title.replace(/</g, "&lt;")}</strong></p>`;
        const iframeHtml = `
            <div class="youtube-player-container" style="display: block; width: 100%; aspect-ratio: 16/9; margin-top: 20px; border-radius: 12px; overflow: hidden; border: 1px solid rgba(255, 255, 255, 0.2);">
                <iframe 
                    width="100%" height="100%" 
                    src="https://www.youtube.com/embed/${song.video_id}?autoplay=1&rel=0" 
                    title="${song.title}" frameborder="0" 
                    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" 
                    allowfullscreen>
                </iframe>
            </div>`;

        this._renderHTML({ answer: answerHtml }, iframeHtml, true);

        if (this.timerManager) {
            this.timerManager.clearPresentationTimeout();
        }
        console.log("[Music] Presentation timer cleared for music playback.");
    }

    _renderHTML(data, infoHtml, isVideo = false) {
        const resultText = document.getElementById('result-text');
        const infoDisplay = document.getElementById('info-display');

        if (resultText) {
            if (isVideo) {
                resultText.innerHTML = data.answer;
            } else {
                resultText.innerHTML = data.answer ? (typeof marked !== 'undefined' ? marked.parse(data.answer) : data.answer) : '';
            }
        }
        if (infoDisplay) {
            infoDisplay.innerHTML = infoHtml;
        }

        data.html_is_pre_rendered = true;
        this.uiController.enterPresentation(data);
    }
}