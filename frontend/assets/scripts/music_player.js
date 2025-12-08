// /frontend/assets/scripts/music_player.js
// Hybrid Music Player - YouTube Embed with Audio Fallback

class MusicPlayer {
    constructor() {
        this.currentAudio = null;
        this.isPlaying = false;
        this.currentSong = null;
    }

    /**
     * สร้าง Music Player UI พร้อม YouTube embed และ Audio fallback
     * @param {Object} song - ข้อมูลเพลง {id/video_id, title, channel, url}
     * @param {HTMLElement} container - container สำหรับแสดง player
     * @returns {HTMLElement} - player element
     */
    createPlayer(song, container) {
        const videoId = song.video_id || song.id;
        const videoUrl = song.url || `https://www.youtube.com/watch?v=${videoId}`;

        this.currentSong = song;

        // สร้าง Player Container
        const playerContainer = document.createElement('div');
        playerContainer.className = 'music-player-container';
        playerContainer.style.cssText = `
            width: 100%; 
            border-radius: 12px; 
            overflow: hidden; 
            border: 1px solid rgba(255, 255, 255, 0.2);
            background: rgba(0, 0, 0, 0.6);
        `;

        // Header
        const header = document.createElement('div');
        header.style.cssText = `
            background: rgba(0, 0, 0, 0.5); 
            padding: 10px 15px; 
            display: flex; 
            justify-content: space-between; 
            align-items: center;
        `;
        header.innerHTML = `
            <span style="font-size: 0.85rem; color: #10b981; font-weight: bold;">
                <i class="fa-solid fa-music" style="margin-right: 8px;"></i>กำลังเล่น: ${this._escapeHtml(song.title)}
            </span>
            <button class="close-player-btn" style="background: none; border: none; color: #f87171; cursor: pointer; font-size: 0.8rem;">
                <i class="fa-solid fa-times"></i> ปิด
            </button>
        `;
        playerContainer.appendChild(header);

        // Content Area - จะเปลี่ยนระหว่าง YouTube embed และ Audio player
        const contentArea = document.createElement('div');
        contentArea.className = 'player-content';
        playerContainer.appendChild(contentArea);

        // แสดง Loading ก่อน
        this._showLoading(contentArea, song.title);

        // ลอง YouTube embed ก่อน
        this._tryYouTubeEmbed(contentArea, videoId, song, videoUrl);

        // Close button handler
        header.querySelector('.close-player-btn').onclick = () => {
            this.stop();
            playerContainer.remove();
        };

        // เพิ่มเข้า container
        container.innerHTML = '';
        container.appendChild(playerContainer);

        return playerContainer;
    }

    _showLoading(contentArea, title) {
        contentArea.innerHTML = `
            <div style="padding: 40px; text-align: center; color: #9ca3af;">
                <i class="fa-solid fa-spinner fa-spin" style="font-size: 2rem; margin-bottom: 15px;"></i>
                <p style="margin: 0;">กำลังโหลด "${this._escapeHtml(title)}"...</p>
            </div>
        `;
    }

    _tryYouTubeEmbed(contentArea, videoId, song, videoUrl) {
        // สร้าง YouTube embed พร้อม autoplay
        const embedUrl = `https://www.youtube.com/embed/${videoId}?autoplay=1&rel=0&enablejsapi=1`;

        const embedContainer = document.createElement('div');
        embedContainer.style.cssText = 'position: relative; width: 100%; aspect-ratio: 16/9;';

        const iframe = document.createElement('iframe');
        iframe.width = '100%';
        iframe.height = '100%';
        iframe.style.border = 'none';
        iframe.allow = 'accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture';
        iframe.allowFullscreen = true;
        iframe.title = song.title;

        // ตั้ง timeout สำหรับ fallback
        let loadedSuccessfully = false;
        const fallbackTimeout = setTimeout(() => {
            if (!loadedSuccessfully) {
                console.log('[MusicPlayer] YouTube embed timeout, trying audio fallback...');
                this._tryAudioFallback(contentArea, song, videoUrl);
            }
        }, 10000); // 10 วินาที timeout

        iframe.onload = () => {
            loadedSuccessfully = true;
            clearTimeout(fallbackTimeout);
            console.log('[MusicPlayer] YouTube embed loaded successfully');
        };

        iframe.onerror = () => {
            clearTimeout(fallbackTimeout);
            console.log('[MusicPlayer] YouTube embed failed, trying audio fallback...');
            this._tryAudioFallback(contentArea, song, videoUrl);
        };

        // Set src after setting up handlers
        iframe.src = embedUrl;

        embedContainer.appendChild(iframe);
        contentArea.innerHTML = '';
        contentArea.appendChild(embedContainer);

        // เพิ่ม Audio Fallback button ด้านล่าง iframe
        const fallbackBtn = document.createElement('button');
        fallbackBtn.style.cssText = `
            width: 100%; 
            padding: 10px; 
            background: rgba(59, 130, 246, 0.2); 
            border: 1px solid rgba(59, 130, 246, 0.3);
            border-radius: 0 0 12px 12px;
            color: #60a5fa; 
            cursor: pointer;
            font-size: 0.8rem;
            transition: all 0.3s;
        `;
        fallbackBtn.innerHTML = '<i class="fa-solid fa-headphones"></i> เล่นไม่ได้? คลิกเพื่อใช้โหมดเสียงอย่างเดียว';
        fallbackBtn.onmouseover = () => fallbackBtn.style.background = 'rgba(59, 130, 246, 0.4)';
        fallbackBtn.onmouseout = () => fallbackBtn.style.background = 'rgba(59, 130, 246, 0.2)';
        fallbackBtn.onclick = () => {
            clearTimeout(fallbackTimeout);
            this._tryAudioFallback(contentArea, song, videoUrl);
        };
        contentArea.appendChild(fallbackBtn);
    }

    async _tryAudioFallback(contentArea, song, videoUrl) {
        this._showLoading(contentArea, song.title + ' (Audio Mode)');

        try {
            // เรียก API เพื่อดึง audio stream URL
            const apiUrl = typeof API_BASE_URL !== 'undefined' ? API_BASE_URL : '';
            const response = await fetch(`${apiUrl}/api/stream?video_url=${encodeURIComponent(videoUrl)}`);

            if (!response.ok) {
                throw new Error(`API response: ${response.status}`);
            }

            const data = await response.json();

            if (data.stream_url) {
                this._createAudioPlayer(contentArea, song, data.stream_url);
            } else {
                throw new Error('No stream URL in response');
            }
        } catch (error) {
            console.error('[MusicPlayer] Audio fallback failed:', error);
            this._showError(contentArea, song, videoUrl);
        }
    }

    _createAudioPlayer(contentArea, song, streamUrl) {
        // สร้าง Custom Audio Player UI
        contentArea.innerHTML = `
            <div class="audio-player" style="padding: 20px;">
                <div style="display: flex; align-items: center; gap: 15px; margin-bottom: 15px;">
                    <div style="width: 60px; height: 60px; background: linear-gradient(135deg, #10b981, #059669); border-radius: 12px; display: flex; align-items: center; justify-content: center;">
                        <i class="fa-solid fa-music" style="font-size: 1.5rem; color: white;"></i>
                    </div>
                    <div style="flex: 1; min-width: 0;">
                        <div style="font-weight: bold; color: white; font-size: 0.95rem; white-space: nowrap; overflow: hidden; text-overflow: ellipsis;">
                            ${this._escapeHtml(song.title)}
                        </div>
                        <div style="font-size: 0.8rem; color: #9ca3af;">${this._escapeHtml(song.channel || 'Unknown Artist')}</div>
                    </div>
                </div>
                
                <div style="display: flex; align-items: center; gap: 10px;">
                    <span class="current-time" style="font-size: 0.75rem; color: #9ca3af; min-width: 40px;">0:00</span>
                    <input type="range" class="progress-bar" value="0" min="0" max="100" style="flex: 1; accent-color: #10b981; cursor: pointer;">
                    <span class="duration" style="font-size: 0.75rem; color: #9ca3af; min-width: 40px;">0:00</span>
                </div>
                
                <div style="display: flex; justify-content: center; gap: 20px; margin-top: 15px;">
                    <button class="audio-btn prev-btn" style="background: none; border: none; color: #9ca3af; font-size: 1.2rem; cursor: pointer; padding: 10px;">
                        <i class="fa-solid fa-backward-step"></i>
                    </button>
                    <button class="audio-btn play-pause-btn" style="background: linear-gradient(135deg, #10b981, #059669); border: none; color: white; font-size: 1.5rem; cursor: pointer; width: 55px; height: 55px; border-radius: 50%; display: flex; align-items: center; justify-content: center;">
                        <i class="fa-solid fa-play"></i>
                    </button>
                    <button class="audio-btn next-btn" style="background: none; border: none; color: #9ca3af; font-size: 1.2rem; cursor: pointer; padding: 10px;">
                        <i class="fa-solid fa-forward-step"></i>
                    </button>
                </div>
                
                <audio class="hidden-audio" src="${streamUrl}" preload="auto"></audio>
            </div>
        `;

        // Setup audio controls
        const audio = contentArea.querySelector('.hidden-audio');
        const playPauseBtn = contentArea.querySelector('.play-pause-btn');
        const progressBar = contentArea.querySelector('.progress-bar');
        const currentTimeEl = contentArea.querySelector('.current-time');
        const durationEl = contentArea.querySelector('.duration');

        this.currentAudio = audio;

        // Auto-play
        audio.play().then(() => {
            this.isPlaying = true;
            playPauseBtn.innerHTML = '<i class="fa-solid fa-pause"></i>';
        }).catch(e => {
            console.log('[MusicPlayer] Autoplay blocked, user must click play');
        });

        // Play/Pause toggle
        playPauseBtn.onclick = () => {
            if (audio.paused) {
                audio.play();
                this.isPlaying = true;
                playPauseBtn.innerHTML = '<i class="fa-solid fa-pause"></i>';
            } else {
                audio.pause();
                this.isPlaying = false;
                playPauseBtn.innerHTML = '<i class="fa-solid fa-play"></i>';
            }
        };

        // Update progress
        audio.ontimeupdate = () => {
            const progress = (audio.currentTime / audio.duration) * 100;
            progressBar.value = progress || 0;
            currentTimeEl.textContent = this._formatTime(audio.currentTime);
        };

        // Set duration when loaded
        audio.onloadedmetadata = () => {
            durationEl.textContent = this._formatTime(audio.duration);
        };

        // Seek
        progressBar.oninput = () => {
            audio.currentTime = (progressBar.value / 100) * audio.duration;
        };

        // When song ends
        audio.onended = () => {
            this.isPlaying = false;
            playPauseBtn.innerHTML = '<i class="fa-solid fa-play"></i>';
            progressBar.value = 0;
        };

        console.log('[MusicPlayer] Audio player created successfully');
    }

    _showError(contentArea, song, videoUrl) {
        contentArea.innerHTML = `
            <div style="padding: 30px; text-align: center;">
                <i class="fa-solid fa-face-sad-tear" style="font-size: 2rem; color: #f87171; margin-bottom: 15px;"></i>
                <p style="color: #f87171; margin-bottom: 15px;">ไม่สามารถเล่นเพลงนี้ได้ค่ะ</p>
                <a href="${videoUrl}" target="_blank" rel="noopener" style="
                    display: inline-block;
                    padding: 10px 20px;
                    background: #ef4444;
                    color: white;
                    text-decoration: none;
                    border-radius: 8px;
                    font-weight: bold;
                ">
                    <i class="fa-brands fa-youtube"></i> เปิดใน YouTube
                </a>
            </div>
        `;
    }

    _formatTime(seconds) {
        if (isNaN(seconds)) return '0:00';
        const mins = Math.floor(seconds / 60);
        const secs = Math.floor(seconds % 60);
        return `${mins}:${secs.toString().padStart(2, '0')}`;
    }

    _escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text || '';
        return div.innerHTML;
    }

    stop() {
        if (this.currentAudio) {
            this.currentAudio.pause();
            this.currentAudio.currentTime = 0;
            this.currentAudio = null;
        }
        this.isPlaying = false;
        this.currentSong = null;
    }
}

// Global instance
const musicPlayer = new MusicPlayer();
