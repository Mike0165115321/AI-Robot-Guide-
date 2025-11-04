// /frontend/assets/scripts/avatar_music_handler.js (V-Music Mod 7.0 - Refactored Class)

class AvatarMusicHandler {
    constructor(websocket, uiController, voiceHandler, timerManager, callbacks) {
        this.websocket = websocket;
        this.uiController = uiController;
        this.voiceHandler = voiceHandler;
        this.timerManager = timerManager;
        this.callbacks = callbacks; 

        this.isAwaitingUserInput = false;
        this.isPlayingMusic = false;

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
        
        if (wasPlaying) {
            this.uiController.exitPresentation();
        }
    }

    handleMessage(data) {
        const messageId = `msg-${Date.now()}`;
        
        if (data.action === 'PROMPT_FOR_SONG_INPUT') {
            console.log("[Music] Prompting for song input.");
            this.isAwaitingUserInput = true; 
            const placeholder = data.action_payload?.placeholder || 'พิมพ์ชื่อเพลง...';
            
            const promptHtml = `
                <div id="song-input-wrapper-${messageId}" class="music-prompt-wrapper">
                    <input type="text" placeholder="${placeholder}" class="song-input-field">
                    <button class="song-submit-btn">ค้นหา</button>
                </div>`;
            
            this._renderHTML(data, promptHtml);

            const wrapper = document.getElementById(`song-input-wrapper-${messageId}`);
            if (wrapper) {
                const inputField = wrapper.querySelector('.song-input-field');
                const submitButton = wrapper.querySelector('.song-submit-btn');
                
                const submitSongQuery = () => {
                    const query = inputField.value.trim();
                    if (query && this.websocket && this.websocket.readyState === WebSocket.OPEN) {
                        console.log(`[Music] Sending text query for song: ${query}`);
                        this.isAwaitingUserInput = false; 
                        this.websocket.send(JSON.stringify({ "query": query }));
                        this.uiController.exitPresentation();
                        this.uiController.setEmotion('thinking');
                        this.uiController.setStatus("กำลังค้นหาเพลง...");
                    }
                };
                submitButton.addEventListener('click', submitSongQuery);
                inputField.addEventListener('keydown', (e) => {
                    if (e.key === 'Enter') { e.preventDefault(); submitSongQuery(); }
                });
                inputField.focus();
            }
            return true; 

        } else if (data.action === 'SHOW_SONG_CHOICES' && Array.isArray(data.action_payload)) {
            console.log("[Music] Showing song choices.");
            this.isAwaitingUserInput = true; 

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

        if (this.callbacks.stopSpeechButton) {
            this.callbacks.stopSpeechButton.classList.add('visible'); 
        }

        console.log("State => Music Idle (Not Listening)");
        this.uiController.setEmotion('normal'); 
        this.uiController.setStatus("กำลังเล่นเพลง... (กดปุ่ม 🟥 เพื่อหยุด)");
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