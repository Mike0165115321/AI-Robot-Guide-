// /frontend/assets/scripts/music_player.js
// Centralized Music Manager - Handles Search, UI, and Playback
// V2.0 - Unified Logic for Chat & Avatar

class MusicPlayer {
    constructor() {
        this.currentAudio = null;
        this.isPlaying = false;
        this.currentSong = null;
        this.apiBaseUrl = typeof API_BASE_URL !== 'undefined' ? API_BASE_URL : '';
    }

    /**
     * ค้นหาเพลงจาก API
     * @param {string} query - ชื่อเพลงหรือคีย์เวิร์ด
     * @returns {Promise<Array>} - รายการเพลง
     */
    async searchMusic(query) {
        if (!query) return [];
        try {
            console.log(`[MusicManager] Searching for: ${query}`);
            const response = await fetch(`${this.apiBaseUrl}/api/chat/music-search`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ song_name: query })
            });
            const data = await response.json();
            if (data.success && data.results) {
                return data.results;
            }
            return [];
        } catch (error) {
            console.error("[MusicManager] Search failed:", error);
            throw error;
        }
    }

    /**
     * สร้าง HTML สำหรับ Prompt ค้นหาเพลง (Genre Buttons + Search Input)
     * @returns {string} - HTML string
     */
    getSearchPromptHTML() {
        return `
            <div class="music-search-prompt" style="margin-top: 15px;">
                <div style="display: flex; flex-wrap: wrap; gap: 8px; margin-bottom: 15px;">
                    <button class="genre-quick-btn" data-query="เพลงเศร้า" style="padding: 8px 16px; background: rgba(16, 185, 129, 0.2); border: 1px solid rgba(16, 185, 129, 0.4); border-radius: 20px; color: #10b981; cursor: pointer; transition: all 0.2s;">😢 เพลงเศร้า</button>
                    <button class="genre-quick-btn" data-query="เพลงสนุก" style="padding: 8px 16px; background: rgba(16, 185, 129, 0.2); border: 1px solid rgba(16, 185, 129, 0.4); border-radius: 20px; color: #10b981; cursor: pointer; transition: all 0.2s;">🎉 เพลงสนุก</button>
                    <button class="genre-quick-btn" data-query="เพลงรัก" style="padding: 8px 16px; background: rgba(16, 185, 129, 0.2); border: 1px solid rgba(16, 185, 129, 0.4); border-radius: 20px; color: #10b981; cursor: pointer; transition: all 0.2s;">💕 เพลงรัก</button>
                </div>
                <div style="display: flex; gap: 8px;">
                    <input type="text" class="music-search-input" placeholder="พิมพ์ชื่อเพลง..." style="flex: 1; padding: 10px 15px; border: 1px solid rgba(255,255,255,0.2); border-radius: 20px; background: rgba(0,0,0,0.3); color: white; outline: none;">
                    <button class="music-search-btn" style="padding: 8px 20px; background: linear-gradient(135deg, #10b981, #059669); border: none; border-radius: 20px; color: white; cursor: pointer; font-weight: bold; transition: transform 0.2s;">
                        <i class="fas fa-search"></i>
                    </button>
                </div>
                <div class="music-results-area" style="margin-top: 15px;"></div>
            </div>
        `;
    }

    /**
     * ผูก Event Handlers ให้กับ Search UI ที่สร้างจาก getSearchPromptHTML
     * @param {HTMLElement} containerElement - Element ที่บรรจุ HTML ของ Prompt
     * @param {Function} onPlayCallback - (song) => void เรียกเมื่อผู้ใช้เลือกเพลง
     */
    bindSearchEvents(containerElement, onPlayCallback) {
        if (!containerElement) return;

        const resultsArea = containerElement.querySelector('.music-results-area');
        const input = containerElement.querySelector('.music-search-input');
        const searchBtn = containerElement.querySelector('.music-search-btn');

        const performSearch = async (query) => {
            if (!query) return;
            resultsArea.innerHTML = '<div style="color:#aaa; text-align:center; padding: 10px;"><i class="fas fa-spinner fa-spin"></i> กำลังค้นหา...</div>';

            try {
                const songs = await this.searchMusic(query);
                if (songs.length > 0) {
                    this.renderSongList(songs, resultsArea, onPlayCallback);
                } else {
                    resultsArea.innerHTML = `<div style="color:#ef4444; text-align:center; padding: 10px;">❌ ไม่พบเพลง "${query}"</div>`;
                }
            } catch (e) {
                resultsArea.innerHTML = `<div style="color:#ef4444; text-align:center; padding: 10px;">❌ เกิดข้อผิดพลาด</div>`;
            }
        };

        // Genre Buttons
        containerElement.querySelectorAll('.genre-quick-btn').forEach(btn => {
            btn.onclick = () => performSearch(btn.dataset.query);
        });

        // Search Button & Enter Key
        if (searchBtn && input) {
            searchBtn.onclick = () => performSearch(input.value.trim());
            input.onkeypress = (e) => {
                if (e.key === 'Enter') performSearch(input.value.trim());
            };
        }
    }

    /**
     * แสดงรายการเพลง (Song Cards)
     * @param {Array} songs - รายการเพลง
     * @param {HTMLElement} targetElement - Element ที่จะแสดงผล
     * @param {Function} onSelect - Callback เมื่อเลือกเพลง
     */
    renderSongList(songs, targetElement, onSelect) {
        if (!targetElement) return;

        targetElement.innerHTML = `
            <div style="display: grid; gap: 8px;">
                ${songs.map((song, index) => `
                    <div class="song-card-item" data-index="${index}" style="
                        display: flex; gap: 10px; padding: 8px; 
                        background: rgba(255,255,255,0.05); border-radius: 10px; cursor: pointer; 
                        border: 1px solid rgba(255,255,255,0.1); transition: all 0.2s; align-items: center;">
                        <img src="${song.thumbnail}" style="width: 50px; height: 38px; border-radius: 6px; object-fit: cover;">
                        <div style="flex: 1; overflow: hidden;">
                            <div style="color: white; font-weight: 500; font-size: 0.9rem; white-space: nowrap; overflow: hidden; text-overflow: ellipsis;">${song.title}</div>
                            <div style="color: #9ca3af; font-size: 0.75rem;">${song.channel || 'Youtube'}</div>
                        </div>
                        <div style="width: 30px; height: 30px; border-radius: 50%; background: rgba(16, 185, 129, 0.1); display: flex; align-items: center; justify-content: center; color: #10b981;">
                            <i class="fas fa-play" style="font-size: 0.8rem;"></i>
                        </div>
                    </div>
                `).join('')}
            </div>
        `;

        // Add Hover Effects & Click Handlers
        targetElement.querySelectorAll('.song-card-item').forEach(card => {
            card.onmouseover = () => { card.style.background = 'rgba(255,255,255,0.1)'; card.style.transform = 'translateY(-2px)'; };
            card.onmouseout = () => { card.style.background = 'rgba(255,255,255,0.05)'; card.style.transform = 'translateY(0)'; };

            card.onclick = () => {
                const index = card.dataset.index;
                if (onSelect) onSelect(songs[index]);
            };
        });
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
            margin-top: 15px;
        `;

        // Content Area - จะเปลี่ยนระหว่าง YouTube embed และ Audio player
        const contentArea = document.createElement('div');
        contentArea.className = 'player-content';
        playerContainer.appendChild(contentArea);

        // แสดง Loading ก่อน
        this._showLoading(contentArea, song.title);

        // ลอง YouTube embed ก่อน
        this._tryYouTubeEmbed(contentArea, videoId, song, videoUrl);

        // เพิ่มเข้า container
        if (container) {
            container.innerHTML = ''; // Clear previous content
            container.appendChild(playerContainer);
        }

        return playerContainer;
    }

    _showLoading(contentArea, title) {
        contentArea.innerHTML = `
            <div style="padding: 30px; text-align: center; color: #9ca3af;">
                <i class="fa-solid fa-spinner fa-spin" style="font-size: 1.5rem; margin-bottom: 10px;"></i>
                <p style="margin: 0; font-size: 0.9rem;">กำลังโหลด "${this._escapeHtml(title)}"...</p>
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
        iframe.src = embedUrl; // Set src directly

        embedContainer.appendChild(iframe);
        contentArea.innerHTML = '';
        contentArea.appendChild(embedContainer);

        // เพิ่ม Audio Fallback button ด้านล่าง iframe
        const fallbackBtn = document.createElement('button');
        fallbackBtn.style.cssText = `
            width: 100%; 
            padding: 8px; 
            background: rgba(59, 130, 246, 0.2); 
            border: none;
            border-top: 1px solid rgba(59, 130, 246, 0.3);
            color: #60a5fa; 
            cursor: pointer;
            font-size: 0.8rem;
            transition: all 0.3s;
            display: flex; align-items: center; justify-content: center; gap: 6px;
        `;
        fallbackBtn.innerHTML = '<i class="fa-solid fa-headphones"></i> ฟังแค่เสียง (ประหยัดเน็ต)';
        fallbackBtn.onmouseover = () => fallbackBtn.style.background = 'rgba(59, 130, 246, 0.3)';
        fallbackBtn.onmouseout = () => fallbackBtn.style.background = 'rgba(59, 130, 246, 0.2)';
        fallbackBtn.onclick = () => {
            this._tryAudioFallback(contentArea, song, videoUrl);
        };
        contentArea.appendChild(fallbackBtn);
    }

    async _tryAudioFallback(contentArea, song, videoUrl) {
        this._showLoading(contentArea, song.title + ' (Audio Mode)');

        try {
            // เรียก API เพื่อดึง audio stream URL
            const response = await fetch(`${this.apiBaseUrl}/api/chat/music/stream`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ video_url: videoUrl })
            });

            if (!response.ok) throw new Error(`API error: ${response.status}`);

            const data = await response.json();

            if (data.stream_url) {
                this._createAudioPlayer(contentArea, song, data.stream_url);
            } else {
                throw new Error('No stream URL');
            }
        } catch (error) {
            console.error('[MusicPlayer] Audio fallback failed:', error);
            this._showError(contentArea, song, videoUrl);
        }
    }

    _createAudioPlayer(contentArea, song, streamUrl) {
        // สร้าง Custom Audio Player UI
        contentArea.innerHTML = `
            <div class="audio-player" style="padding: 15px; background: linear-gradient(to bottom, #111827, #000);">
                <div style="display: flex; align-items: center; gap: 12px; margin-bottom: 15px;">
                    <div style="width: 50px; height: 50px; background: linear-gradient(135deg, #10b981, #059669); border-radius: 10px; display: flex; align-items: center; justify-content: center;">
                        <i class="fa-solid fa-music" style="font-size: 1.2rem; color: white;"></i>
                    </div>
                    <div style="flex: 1; min-width: 0;">
                        <div style="font-weight: bold; color: white; font-size: 0.9rem; white-space: nowrap; overflow: hidden; text-overflow: ellipsis;">
                            ${this._escapeHtml(song.title)}
                        </div>
                        <div style="font-size: 0.75rem; color: #9ca3af;">${this._escapeHtml(song.channel || 'Artist')}</div>
                    </div>
                </div>
                
                <div style="display: flex; align-items: center; gap: 8px; margin-bottom: 10px;">
                    <span class="current-time" style="font-size: 0.7rem; color: #6b7280; min-width: 30px;">0:00</span>
                    <input type="range" class="progress-bar" value="0" min="0" max="100" style="flex: 1; accent-color: #10b981; cursor: pointer; height: 4px;">
                    <span class="duration" style="font-size: 0.7rem; color: #6b7280; min-width: 30px;">0:00</span>
                </div>
                
                <div style="display: flex; justify-content: center; align-items: center; gap: 20px;">
                    <button class="audio-btn play-pause-btn" style="background: white; border: none; color: #000; font-size: 1.2rem; cursor: pointer; width: 40px; height: 40px; border-radius: 50%; display: flex; align-items: center; justify-content: center; box-shadow: 0 0 10px rgba(255,255,255,0.2);">
                        <i class="fa-solid fa-play" style="margin-left: 2px;"></i>
                    </button>
                </div>
                
                <audio class="hidden-audio" src="${streamUrl}" autoplay></audio>
            </div>
        `;

        // Logic Controls
        const audio = contentArea.querySelector('.hidden-audio');
        const playBtn = contentArea.querySelector('.play-pause-btn');
        const progress = contentArea.querySelector('.progress-bar');
        const timeEl = contentArea.querySelector('.current-time');
        const durEl = contentArea.querySelector('.duration');

        this.currentAudio = audio;
        this.isPlaying = true;
        playBtn.innerHTML = '<i class="fa-solid fa-pause"></i>';

        playBtn.onclick = () => {
            if (audio.paused) {
                audio.play();
                playBtn.innerHTML = '<i class="fa-solid fa-pause"></i>';
                this.isPlaying = true;
            } else {
                audio.pause();
                playBtn.innerHTML = '<i class="fa-solid fa-play" style="margin-left: 2px;"></i>';
                this.isPlaying = false;
            }
        };

        audio.ontimeupdate = () => {
            if (audio.duration) {
                progress.value = (audio.currentTime / audio.duration) * 100;
                timeEl.textContent = this._formatTime(audio.currentTime);
                durEl.textContent = this._formatTime(audio.duration);
            }
        };

        progress.oninput = () => {
            if (audio.duration) {
                audio.currentTime = (progress.value / 100) * audio.duration;
            }
        };

        audio.onended = () => {
            playBtn.innerHTML = '<i class="fa-solid fa-play" style="margin-left: 2px;"></i>';
            this.isPlaying = false;
        };
    }

    _showError(contentArea, song, videoUrl) {
        contentArea.innerHTML = `
            <div style="padding: 20px; text-align: center;">
                <p style="color: #f87171; margin-bottom: 10px;">เกิดข้อผิดพลาดในการเล่น</p>
                <a href="${videoUrl}" target="_blank" style="color: #60a5fa; text-decoration: none; border-bottom: 1px dotted #60a5fa;">เปิดใน YouTube</a>
            </div>
        `;
    }

    _formatTime(s) {
        if (!s) return '0:00';
        const m = Math.floor(s / 60);
        const sec = Math.floor(s % 60);
        return `${m}:${sec.toString().padStart(2, '0')}`;
    }

    _escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text || '';
        return div.innerHTML;
    }

    stop() {
        if (this.currentAudio) {
            this.currentAudio.pause();
            this.currentAudio = null;
        }
        this.isPlaying = false;
        // Also clear any iframes if possible, or leave it to UI removal
    }
}

// Global Instance
const musicPlayer = new MusicPlayer();
