/**
 * sounds.js - Sound Effects Manager
 * Handles all audio feedback and music features
 */

class SoundManager {
    constructor() {
        this.enabled = true;
        this.volume = 0.5;
        this.sounds = {};
        this.audioContext = null;
        this.isSinging = false;
        this.visualizerInterval = null;
    }

    init() {
        // Initialize Web Audio API
        try {
            this.audioContext = new (window.AudioContext || window.webkitAudioContext)();
            console.log('✅ Sound Manager initialized');
        } catch (e) {
            console.warn('Web Audio API not supported');
        }

        // Load sound effects (using simple oscillator for now)
        this.createSounds();
    }

    createSounds() {
        // Define sound patterns
        this.soundPatterns = {
            'click': { freq: 800, duration: 0.05 },
            'dance': { freq: 440, duration: 0.2 },
            'jump': { freq: 600, duration: 0.15 },
            'whoosh': { freq: 200, duration: 0.3 },
            'wave': { freq: 500, duration: 0.1 },
            'bow': { freq: 350, duration: 0.2 },
            'celebrate': { freq: 880, duration: 0.3 },
            'sleep': { freq: 250, duration: 0.5 },
            'exercise': { freq: 450, duration: 0.1 },
            'fly': { freq: 700, duration: 0.4 },
            'robot': { freq: 300, duration: 0.1 }
        };
    }

    play(soundName) {
        if (!this.enabled || !this.audioContext) return;

        const pattern = this.soundPatterns[soundName];
        if (!pattern) return;

        const oscillator = this.audioContext.createOscillator();
        const gainNode = this.audioContext.createGain();

        oscillator.connect(gainNode);
        gainNode.connect(this.audioContext.destination);

        oscillator.frequency.value = pattern.freq;
        oscillator.type = 'sine';

        gainNode.gain.setValueAtTime(this.volume, this.audioContext.currentTime);
        gainNode.gain.exponentialRampToValueAtTime(
            0.01,
            this.audioContext.currentTime + pattern.duration
        );

        oscillator.start(this.audioContext.currentTime);
        oscillator.stop(this.audioContext.currentTime + pattern.duration);
    }

    playMelody(notes, tempo = 500) {
        if (!this.enabled || !this.audioContext) return;

        notes.forEach((note, index) => {
            setTimeout(() => {
                this.playNote(note.freq, note.duration || 0.2);
            }, index * tempo);
        });
    }

    playNote(frequency, duration) {
        if (!this.audioContext) return;

        const oscillator = this.audioContext.createOscillator();
        const gainNode = this.audioContext.createGain();

        oscillator.connect(gainNode);
        gainNode.connect(this.audioContext.destination);

        oscillator.frequency.value = frequency;
        oscillator.type = 'sine';

        gainNode.gain.setValueAtTime(this.volume, this.audioContext.currentTime);
        gainNode.gain.exponentialRampToValueAtTime(
            0.01,
            this.audioContext.currentTime + duration
        );

        oscillator.start(this.audioContext.currentTime);
        oscillator.stop(this.audioContext.currentTime + duration);
    }

    startSinging() {
        if (this.isSinging) return;
        this.isSinging = true;

        // Happy melody pattern
        const melody = [
            { freq: 523.25, duration: 0.3 }, // C5
            { freq: 587.33, duration: 0.3 }, // D5
            { freq: 659.25, duration: 0.3 }, // E5
            { freq: 698.46, duration: 0.3 }, // F5
            { freq: 783.99, duration: 0.3 }, // G5
            { freq: 880.00, duration: 0.3 }, // A5
            { freq: 987.77, duration: 0.3 }, // B5
            { freq: 1046.50, duration: 0.5 } // C6
        ];

        let currentNote = 0;

        const singInterval = setInterval(() => {
            if (!this.isSinging) {
                clearInterval(singInterval);
                return;
            }

            this.playNote(melody[currentNote].freq, melody[currentNote].duration);
            this.createMusicalNote(melody[currentNote].freq);
            
            currentNote = (currentNote + 1) % melody.length;
        }, 400);

        this.singInterval = singInterval;

        // Start visualizer
        this.startVisualizer();

        // Set avatar to singing mood
        if (window.NanAvatar) {
            window.NanAvatar.setMood('speaking');
        }
    }

    stopSinging() {
        this.isSinging = false;
        if (this.singInterval) {
            clearInterval(this.singInterval);
            this.singInterval = null;
        }

        this.stopVisualizer();

        // Reset avatar mood
        if (window.NanAvatar) {
            window.NanAvatar.setMood('normal');
        }
    }

    startVisualizer() {
        const visualizer = document.getElementById('musicVisualizer');
        if (!visualizer) return;

        visualizer.style.display = 'flex';
        const bars = visualizer.querySelectorAll('.visualizer-bar');

        this.visualizerInterval = setInterval(() => {
            bars.forEach(bar => {
                const height = Math.random() * 80 + 20;
                bar.style.height = `${height}%`;
            });
        }, 100);
    }

    stopVisualizer() {
        const visualizer = document.getElementById('musicVisualizer');
        if (visualizer) {
            visualizer.style.display = 'none';
        }

        if (this.visualizerInterval) {
            clearInterval(this.visualizerInterval);
            this.visualizerInterval = null;
        }
    }

    createMusicalNote(frequency) {
        const notesContainer = document.getElementById('musicalNotes');
        if (!notesContainer) return;

        const note = document.createElement('div');
        note.className = 'musical-note';
        note.textContent = ['♪', '♫', '♬'][Math.floor(Math.random() * 3)];
        note.style.position = 'absolute';
        note.style.left = `${40 + Math.random() * 20}%`;
        note.style.bottom = '40%';
        note.style.fontSize = `${20 + Math.random() * 20}px`;
        note.style.color = `hsl(${(frequency / 1000) * 360}, 70%, 60%)`;
        note.style.zIndex = '50';
        note.style.pointerEvents = 'none';
        notesContainer.appendChild(note);

        gsap.to(note, {
            y: -150,
            x: (Math.random() - 0.5) * 100,
            rotation: (Math.random() - 0.5) * 360,
            opacity: 0,
            duration: 2,
            ease: 'power1.out',
            onComplete: () => note.remove()
        });
    }

    setVolume(value) {
        this.volume = value / 100;
    }

    toggle() {
        this.enabled = !this.enabled;
        return this.enabled;
    }

    setEnabled(enabled) {
        this.enabled = enabled;
    }
}

// Create global instance
window.SoundManager = new SoundManager();