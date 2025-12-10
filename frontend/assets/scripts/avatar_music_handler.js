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
            console.log("[Music] Showing song search prompt.");
            this.isAwaitingUserInput = true;

            // สร้าง UI สำหรับค้นหาเพลง
            const inputHtml = `
                <div class="music-search-prompt" style="margin-top: 20px;">
                    <div style="display: flex; flex-wrap: wrap; gap: 8px; margin-bottom: 15px;">
                        <button class="genre-quick-btn" data-query="เปิดเพลงเศร้าให้หน่อย" style="padding: 8px 16px; background: rgba(16, 185, 129, 0.2); border: 1px solid rgba(16, 185, 129, 0.4); border-radius: 20px; color: #10b981; cursor: pointer;">😢 เพลงเศร้า</button>
                        <button class="genre-quick-btn" data-query="เปิดเพลงสนุกๆ ให้หน่อย" style="padding: 8px 16px; background: rgba(16, 185, 129, 0.2); border: 1px solid rgba(16, 185, 129, 0.4); border-radius: 20px; color: #10b981; cursor: pointer;">🎉 เพลงสนุก</button>
                        <button class="genre-quick-btn" data-query="เปิดเพลงรักหวานๆ" style="padding: 8px 16px; background: rgba(16, 185, 129, 0.2); border: 1px solid rgba(16, 185, 129, 0.4); border-radius: 20px; color: #10b981; cursor: pointer;">💕 เพลงรัก</button>
                    </div>
                    <div style="display: flex; gap: 8px;">
                        <input type="text" id="avatar-song-input" placeholder="หรือพิมพ์ชื่อเพลง/ศิลปิน..." style="
                            flex: 1;
                            padding: 12px 16px;
                            border: 1px solid rgba(255,255,255,0.2);
                            border-radius: 8px;
                            background: rgba(0,0,0,0.3);
                            color: white;
                            font-size: 1rem;
                        ">
                        <button id="avatar-song-search-btn" style="
                            padding: 12px 24px;
                            background: linear-gradient(135deg, #10b981, #059669);
                            border: none;
                            border-radius: 8px;
                            color: white;
                            cursor: pointer;
                            font-weight: bold;
                        ">🔍 ค้นหา</button>
                    </div>
                </div>
            `;

            this._renderHTML(data, inputHtml);
            this.uiController.setEmotion('listening');

            // Event listeners for genre buttons
            document.querySelectorAll('.genre-quick-btn').forEach(btn => {
                btn.addEventListener('click', () => {
                    const query = btn.dataset.query;
                    this.isAwaitingUserInput = false;
                    if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
                        this.websocket.send(JSON.stringify({ query: query }));
                        this.uiController.setEmotion('thinking');
                        this.uiController.setStatus("กำลังค้นหาเพลง...");
                    }
                });
            });

            // Event listener for search input
            const searchInput = document.getElementById('avatar-song-input');
            const searchBtn = document.getElementById('avatar-song-search-btn');

            if (searchBtn && searchInput) {
                const doSearch = () => {
                    const songName = searchInput.value.trim();
                    if (songName && this.websocket && this.websocket.readyState === WebSocket.OPEN) {
                        this.isAwaitingUserInput = false;
                        this.websocket.send(JSON.stringify({ query: `เปิดเพลง ${songName}` }));
                        this.uiController.setEmotion('thinking');
                        this.uiController.setStatus("กำลังค้นหาเพลง...");
                    }
                };
                searchBtn.addEventListener('click', doSearch);
                searchInput.addEventListener('keypress', (e) => {
                    if (e.key === 'Enter') doSearch();
                });
                setTimeout(() => searchInput.focus(), 100);
            }

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

        // ใช้ MusicPlayer class ใหม่ (ถ้ามี)
        let playerHtml = '';
        if (typeof musicPlayer !== 'undefined' && musicPlayer.createPlayer) {
            // สร้าง container สำหรับ music player
            playerHtml = `<div id="avatar-music-player-container" style="margin-top: 20px;"></div>`;
        } else {
            // Fallback: ใช้ iframe เดิม
            playerHtml = `
                <div class="youtube-player-container" style="display: block; width: 100%; aspect-ratio: 16/9; margin-top: 20px; border-radius: 12px; overflow: hidden; border: 1px solid rgba(255, 255, 255, 0.2);">
                    <iframe 
                        width="100%" height="100%" 
                        src="https://www.youtube.com/embed/${song.video_id}?autoplay=1&rel=0" 
                        title="${song.title}" frameborder="0" 
                        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" 
                        allowfullscreen>
                    </iframe>
                </div>`;
        }

        this._renderHTML({ answer: answerHtml }, playerHtml, true);

        // ถ้าใช้ MusicPlayer ให้ initialize หลังจาก render
        if (typeof musicPlayer !== 'undefined' && musicPlayer.createPlayer) {
            setTimeout(() => {
                const container = document.getElementById('avatar-music-player-container');
                if (container) {
                    const normalizedSong = {
                        video_id: song.video_id,
                        title: song.title,
                        channel: song.channel || 'Unknown',
                        url: `https://www.youtube.com/watch?v=${song.video_id}`
                    };
                    musicPlayer.createPlayer(normalizedSong, container);
                }
            }, 100);
        }

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