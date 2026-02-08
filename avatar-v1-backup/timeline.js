/**
 * timeline.js - Animation Timeline Controller
 * Create and manage custom animation sequences
 */

class TimelineController {
    constructor() {
        this.tracks = {
            mood: [],
            animation: [],
            sound: []
        };
        this.currentTime = 0;
        this.duration = 10; // seconds
        this.isPlaying = false;
        this.playhead = null;
        this.savedSequences = [];
    }

    init() {
        this.playhead = document.getElementById('playhead');
        this.setupEventListeners();
        this.loadSavedSequences();
        console.log('✅ Timeline Controller initialized');
    }

    setupEventListeners() {
        // Toolbar buttons
        document.getElementById('playSequence')?.addEventListener('click', () => this.play());
        document.getElementById('pauseSequence')?.addEventListener('click', () => this.pause());
        document.getElementById('stopSequence')?.addEventListener('click', () => this.stop());
        document.getElementById('clearSequence')?.addEventListener('click', () => this.clear());
        document.getElementById('saveSequence')?.addEventListener('click', () => this.save());
        document.getElementById('loadSequence')?.addEventListener('click', () => this.showLoadDialog());

        // Make tracks droppable
        document.querySelectorAll('.track-content').forEach(track => {
            track.addEventListener('dragover', (e) => this.handleDragOver(e));
            track.addEventListener('drop', (e) => this.handleDrop(e));
        });
    }

    addKeyframe(track, time, data) {
        if (!this.tracks[track]) return;

        const keyframe = {
            time: time,
            data: data,
            id: Date.now() + Math.random()
        };

        this.tracks[track].push(keyframe);
        this.tracks[track].sort((a, b) => a.time - b.time);

        this.renderTrack(track);
    }

    removeKeyframe(track, id) {
        if (!this.tracks[track]) return;

        this.tracks[track] = this.tracks[track].filter(kf => kf.id !== id);
        this.renderTrack(track);
    }

    renderTrack(trackName) {
        const trackElement = document.querySelector(`[data-track="${trackName}"]`);
        if (!trackElement) return;

        trackElement.innerHTML = '';

        this.tracks[trackName].forEach(keyframe => {
            const keyframeElement = document.createElement('div');
            keyframeElement.className = 'timeline-keyframe';
            keyframeElement.style.left = `${(keyframe.time / this.duration) * 100}%`;
            keyframeElement.dataset.id = keyframe.id;
            keyframeElement.draggable = true;

            // Icon based on track type
            let icon = '●';
            if (trackName === 'mood') icon = '😊';
            else if (trackName === 'animation') icon = '🎬';
            else if (trackName === 'sound') icon = '🔊';

            keyframeElement.innerHTML = `
                <span class="keyframe-icon">${icon}</span>
                <span class="keyframe-label">${keyframe.data.name || keyframe.data}</span>
                <button class="keyframe-delete" onclick="TimelineController.removeKeyframe('${trackName}', ${keyframe.id})">×</button>
            `;

            // Drag events
            keyframeElement.addEventListener('dragstart', (e) => this.handleKeyframeDragStart(e, keyframe));

            trackElement.appendChild(keyframeElement);
        });
    }

    handleDragOver(e) {
        e.preventDefault();
        e.dataTransfer.dropEffect = 'move';
    }

    handleDrop(e) {
        e.preventDefault();
        const trackElement = e.currentTarget;
        const trackName = trackElement.dataset.track;
        const rect = trackElement.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const time = (x / rect.width) * this.duration;

        // Get data from drag
        const data = e.dataTransfer.getData('text/plain');
        if (data) {
            try {
                const parsedData = JSON.parse(data);
                this.addKeyframe(trackName, time, parsedData);
            } catch (err) {
                console.error('Invalid drag data', err);
            }
        }
    }

    handleKeyframeDragStart(e, keyframe) {
        e.dataTransfer.effectAllowed = 'move';
        e.dataTransfer.setData('text/plain', JSON.stringify(keyframe));
    }

    play() {
        if (this.isPlaying) return;
        this.isPlaying = true;

        const startTime = Date.now();
        const animate = () => {
            if (!this.isPlaying) return;

            const elapsed = (Date.now() - startTime) / 1000;
            this.currentTime = elapsed % this.duration;

            // Update playhead
            if (this.playhead) {
                this.playhead.style.left = `${(this.currentTime / this.duration) * 100}%`;
            }

            // Execute keyframes at current time
            this.executeKeyframesAt(this.currentTime);

            requestAnimationFrame(animate);
        };

        animate();
    }

    pause() {
        this.isPlaying = false;
    }

    stop() {
        this.isPlaying = false;
        this.currentTime = 0;
        if (this.playhead) {
            this.playhead.style.left = '0%';
        }
    }

    clear() {
        if (confirm('Clear all keyframes?')) {
            this.tracks.mood = [];
            this.tracks.animation = [];
            this.tracks.sound = [];

            Object.keys(this.tracks).forEach(track => this.renderTrack(track));
        }
    }

    executeKeyframesAt(time) {
        const tolerance = 0.1; // 100ms tolerance

        Object.keys(this.tracks).forEach(trackName => {
            this.tracks[trackName].forEach(keyframe => {
                if (Math.abs(keyframe.time - time) < tolerance && !keyframe.executed) {
                    this.executeKeyframe(trackName, keyframe);
                    keyframe.executed = true;

                    // Reset executed flag after tolerance
                    setTimeout(() => {
                        keyframe.executed = false;
                    }, tolerance * 1000);
                }
            });
        });
    }

    executeKeyframe(track, keyframe) {
        switch (track) {
            case 'mood':
                if (window.NanAvatar) {
                    window.NanAvatar.setMood(keyframe.data.mood || keyframe.data);
                }
                break;

            case 'animation':
                if (window.AnimationController) {
                    const animName = keyframe.data.animation || keyframe.data;
                    if (typeof window.AnimationController[animName] === 'function') {
                        window.AnimationController[animName]();
                    }
                }
                break;

            case 'sound':
                if (window.SoundManager) {
                    window.SoundManager.play(keyframe.data.sound || keyframe.data);
                }
                break;
        }
    }

    save() {
        const name = prompt('Enter sequence name:');
        if (!name) return;

        const sequence = {
            name: name,
            date: new Date().toISOString(),
            tracks: JSON.parse(JSON.stringify(this.tracks)),
            duration: this.duration
        };

        this.savedSequences.push(sequence);
        localStorage.setItem('avatarSequences', JSON.stringify(this.savedSequences));

        this.renderSequenceList();
        alert('Sequence saved!');
    }

    load(index) {
        if (!this.savedSequences[index]) return;

        const sequence = this.savedSequences[index];
        this.tracks = JSON.parse(JSON.stringify(sequence.tracks));
        this.duration = sequence.duration;

        Object.keys(this.tracks).forEach(track => this.renderTrack(track));
    }

    loadSavedSequences() {
        const saved = localStorage.getItem('avatarSequences');
        if (saved) {
            try {
                this.savedSequences = JSON.parse(saved);
                this.renderSequenceList();
            } catch (e) {
                console.error('Failed to load sequences', e);
            }
        }
    }

    renderSequenceList() {
        const listElement = document.getElementById('sequenceList');
        if (!listElement) return;

        listElement.innerHTML = '';

        this.savedSequences.forEach((seq, index) => {
            const item = document.createElement('div');
            item.className = 'sequence-item';
            item.innerHTML = `
                <div class="sequence-info">
                    <strong>${seq.name}</strong>
                    <small>${new Date(seq.date).toLocaleDateString()}</small>
                </div>
                <div class="sequence-actions">
                    <button onclick="window.TimelineController.load(${index})">Load</button>
                    <button onclick="window.TimelineController.deleteSequence(${index})">Delete</button>
                </div>
            `;
            listElement.appendChild(item);
        });
    }

    deleteSequence(index) {
        if (confirm('Delete this sequence?')) {
            this.savedSequences.splice(index, 1);
            localStorage.setItem('avatarSequences', JSON.stringify(this.savedSequences));
            this.renderSequenceList();
        }
    }

    showLoadDialog() {
        // Simple implementation - could be enhanced with a modal
        if (this.savedSequences.length === 0) {
            alert('No saved sequences');
            return;
        }

        const names = this.savedSequences.map((seq, i) => `${i + 1}. ${seq.name}`).join('\n');
        const choice = prompt(`Select sequence to load:\n${names}\n\nEnter number:`);

        if (choice) {
            const index = parseInt(choice) - 1;
            if (index >= 0 && index < this.savedSequences.length) {
                this.load(index);
            }
        }
    }
}

// Create global instance
window.TimelineController = new TimelineController();