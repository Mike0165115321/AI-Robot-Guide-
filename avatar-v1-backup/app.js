/**
 * app.js - Main Application Controller
 * Integrates all systems and handles UI interactions
 */

class AvatarStudioApp {
    constructor() {
        this.currentTheme = 'dark';
        this.currentSection = 'playground';
        this.isDragging = false;
        this.dragOffset = { x: 0, y: 0 };
    }

    init() {
        console.log('🚀 Initializing Avatar Studio...');

        // Initialize all systems
        this.initTheme();
        this.initNavigation();
        this.initAvatar();
        this.initAnimations();
        this.initParticles();
        this.initSounds();
        this.initTimeline();
        this.initControls();
        this.initDragDrop();
        this.initGallery();
        this.initAnimationLibrary();

        console.log('✅ Avatar Studio Ready!');
    }

    // ==========================================
    // THEME SYSTEM
    // ==========================================

    initTheme() {
        const savedTheme = localStorage.getItem('avatarTheme') || 'dark';
        this.setTheme(savedTheme);

        document.getElementById('themeToggle')?.addEventListener('click', () => {
            this.toggleTheme();
        });
    }

    setTheme(theme) {
        this.currentTheme = theme;
        document.documentElement.setAttribute('data-theme', theme);
        localStorage.setItem('avatarTheme', theme);

        if (window.ParticleSystem) {
            window.ParticleSystem.setTheme(theme);
        }
    }

    toggleTheme() {
        const newTheme = this.currentTheme === 'light' ? 'dark' : 'light';
        this.setTheme(newTheme);
        this.playSound('click');
    }

    // ==========================================
    // NAVIGATION
    // ==========================================

    initNavigation() {
        document.querySelectorAll('.nav-link').forEach(link => {
            link.addEventListener('click', (e) => {
                e.preventDefault();
                const sectionId = link.getAttribute('href').substring(1);
                this.showSection(sectionId);
            });
        });
    }

    showSection(sectionId) {
        // Hide all sections
        document.querySelectorAll('.section').forEach(section => {
            section.classList.remove('active');
        });

        // Show target section
        const targetSection = document.getElementById(sectionId);
        if (targetSection) {
            targetSection.classList.add('active');
            this.currentSection = sectionId;
        }

        // Update nav links
        document.querySelectorAll('.nav-link').forEach(link => {
            link.classList.remove('active');
            if (link.getAttribute('href') === `#${sectionId}`) {
                link.classList.add('active');
            }
        });

        this.playSound('click');
    }

    // ==========================================
    // AVATAR INITIALIZATION
    // ==========================================

    initAvatar() {
        if (window.NanAvatar) {
            window.NanAvatar.init();
        }
    }

    initAnimations() {
        if (window.AnimationController) {
            window.AnimationController.init();
        }
    }

    initParticles() {
        if (window.ParticleSystem) {
            window.ParticleSystem.init();
        }
    }

    initSounds() {
        if (window.SoundManager) {
            window.SoundManager.init();
        }
    }

    initTimeline() {
        if (window.TimelineController) {
            window.TimelineController.init();
        }
    }

    // ==========================================
    // CONTROL PANEL
    // ==========================================

    initControls() {
        // Panel toggle
        document.getElementById('panelToggle')?.addEventListener('click', () => {
            const panel = document.querySelector('.control-panel');
            panel?.classList.toggle('collapsed');
        });

        // Mood buttons
        document.querySelectorAll('.mood-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const mood = btn.dataset.mood;
                this.setMood(mood);

                // Update active state
                document.querySelectorAll('.mood-btn').forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
            });
        });

        // Skin buttons
        document.querySelectorAll('.skin-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const skin = btn.dataset.skin;
                this.setSkin(skin);

                // Update active state
                document.querySelectorAll('.skin-btn').forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
            });
        });

        // Animation buttons
        document.querySelectorAll('.animation-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const animation = btn.dataset.animation;
                this.playAnimation(animation);
            });
        });

        // Music controls
        document.getElementById('singBtn')?.addEventListener('click', () => {
            if (window.SoundManager) {
                window.SoundManager.startSinging();
            }
        });

        document.getElementById('stopSingBtn')?.addEventListener('click', () => {
            if (window.SoundManager) {
                window.SoundManager.stopSinging();
            }
        });

        // Volume slider
        document.getElementById('volumeSlider')?.addEventListener('input', (e) => {
            if (window.SoundManager) {
                window.SoundManager.setVolume(e.target.value);
            }
        });

        // Sound toggle
        document.getElementById('soundToggle')?.addEventListener('change', (e) => {
            if (window.SoundManager) {
                window.SoundManager.setEnabled(e.target.checked);
            }
        });
    }

    setMood(mood) {
        if (window.NanAvatar) {
            window.NanAvatar.setMood(mood);
        }

        // Update info display
        const moodNames = {
            normal: 'Normal',
            speaking: 'Speaking',
            thinking: 'Thinking',
            listening: 'Listening',
            happy: 'Happy',
            curious: 'Curious',
            sleepy: 'Sleepy'
        };

        document.getElementById('currentMood').textContent = moodNames[mood] || mood;
        this.playSound('click');
    }

    setSkin(skinName) {
        if (window.NanAvatar && window.NanAvatar.moodColors) {
            // Update mood colors for the new skin
            const skinColors = this.getSkinColors(skinName);
            if (skinColors) {
                window.NanAvatar.moodColors = skinColors;
                // Re-apply current mood with new colors
                window.NanAvatar.setMood(window.NanAvatar.currentMood);
            }
        }

        // Update info display
        const skinNames = {
            NanRobot: 'น่าน',
            GoldenNan: 'ทอง',
            NightNan: 'กลางคืน',
            SakuraNan: 'ซากุระ',
            CyberNan: 'ไซเบอร์',
            ForestNan: 'ป่า',
            OceanNan: 'ทะเล',
            SunsetNan: 'ตะวันตก',
            FestivalNan: 'เทศกาล'
        };

        document.getElementById('currentSkin').textContent = skinNames[skinName] || skinName;
        this.playSound('click');
    }

    getSkinColors(skinName) {
        const SKINS = {
            NanRobot: {
                normal: { eye: '#40c4ff', accent: '#00e5ff', glow: 'rgba(64, 196, 255, 0.5)' },
                speaking: { eye: '#4ade80', accent: '#22c55e', glow: 'rgba(74, 222, 128, 0.6)' },
                thinking: { eye: '#fbbf24', accent: '#f59e0b', glow: 'rgba(251, 191, 36, 0.5)' },
                listening: { eye: '#f472b6', accent: '#ec4899', glow: 'rgba(244, 114, 182, 0.5)' },
                happy: { eye: '#34d399', accent: '#10b981', glow: 'rgba(52, 211, 153, 0.6)' },
                curious: { eye: '#a78bfa', accent: '#8b5cf6', glow: 'rgba(167, 139, 250, 0.5)' },
                sleepy: { eye: '#64748b', accent: '#475569', glow: 'rgba(100, 116, 139, 0.3)' }
            },
            GoldenNan: {
                normal: { eye: '#ffd700', accent: '#ffb800', glow: 'rgba(255, 215, 0, 0.5)' },
                speaking: { eye: '#ffe066', accent: '#ffc107', glow: 'rgba(255, 193, 7, 0.6)' },
                thinking: { eye: '#f0c14b', accent: '#d4a017', glow: 'rgba(212, 160, 23, 0.5)' },
                listening: { eye: '#ffdf6f', accent: '#e6b800', glow: 'rgba(230, 184, 0, 0.5)' },
                happy: { eye: '#fff176', accent: '#fdd835', glow: 'rgba(253, 216, 53, 0.6)' },
                curious: { eye: '#ffcc80', accent: '#ffa726', glow: 'rgba(255, 167, 38, 0.5)' },
                sleepy: { eye: '#c9a227', accent: '#9a7b0a', glow: 'rgba(154, 123, 10, 0.3)' }
            },
            NightNan: {
                normal: { eye: '#8b9dc3', accent: '#6a7fdb', glow: 'rgba(106, 127, 219, 0.5)' },
                speaking: { eye: '#a8c0ff', accent: '#8fadff', glow: 'rgba(143, 173, 255, 0.6)' },
                thinking: { eye: '#c4b7ff', accent: '#9d8cff', glow: 'rgba(157, 140, 255, 0.5)' },
                listening: { eye: '#e0b3ff', accent: '#c87dff', glow: 'rgba(200, 125, 255, 0.5)' },
                happy: { eye: '#b8d4ff', accent: '#96c3ff', glow: 'rgba(150, 195, 255, 0.6)' },
                curious: { eye: '#dbb3ff', accent: '#bb8cff', glow: 'rgba(187, 140, 255, 0.5)' },
                sleepy: { eye: '#5c6378', accent: '#3d4555', glow: 'rgba(61, 69, 85, 0.3)' }
            },
            SakuraNan: {
                normal: { eye: '#ffb6c1', accent: '#ff91a4', glow: 'rgba(255, 182, 193, 0.5)' },
                speaking: { eye: '#ffc1cc', accent: '#ff8fa3', glow: 'rgba(255, 143, 163, 0.6)' },
                thinking: { eye: '#e8b4d8', accent: '#d18ec4', glow: 'rgba(209, 142, 196, 0.5)' },
                listening: { eye: '#ffaec9', accent: '#ff85a1', glow: 'rgba(255, 133, 161, 0.5)' },
                happy: { eye: '#ffd1dc', accent: '#ffb3c6', glow: 'rgba(255, 179, 198, 0.6)' },
                curious: { eye: '#e8a4c4', accent: '#d77fa1', glow: 'rgba(215, 127, 161, 0.5)' },
                sleepy: { eye: '#b8a0a8', accent: '#8f7880', glow: 'rgba(143, 120, 128, 0.3)' }
            },
            CyberNan: {
                normal: { eye: '#00ffff', accent: '#ff00ff', glow: 'rgba(0, 255, 255, 0.7)' },
                speaking: { eye: '#00ff9f', accent: '#00ffff', glow: 'rgba(0, 255, 159, 0.7)' },
                thinking: { eye: '#ff00ff', accent: '#bf00ff', glow: 'rgba(255, 0, 255, 0.6)' },
                listening: { eye: '#ff3366', accent: '#ff0055', glow: 'rgba(255, 51, 102, 0.6)' },
                happy: { eye: '#39ff14', accent: '#00ff00', glow: 'rgba(57, 255, 20, 0.7)' },
                curious: { eye: '#ffff00', accent: '#ff9900', glow: 'rgba(255, 255, 0, 0.6)' },
                sleepy: { eye: '#4a5568', accent: '#2d3748', glow: 'rgba(74, 85, 104, 0.4)' }
            },
            ForestNan: {
                normal: { eye: '#4ade80', accent: '#22c55e', glow: 'rgba(74, 222, 128, 0.5)' },
                speaking: { eye: '#86efac', accent: '#4ade80', glow: 'rgba(134, 239, 172, 0.6)' },
                thinking: { eye: '#a3e635', accent: '#84cc16', glow: 'rgba(163, 230, 53, 0.5)' },
                listening: { eye: '#6ee7b7', accent: '#34d399', glow: 'rgba(110, 231, 183, 0.5)' },
                happy: { eye: '#bef264', accent: '#a3e635', glow: 'rgba(190, 242, 100, 0.6)' },
                curious: { eye: '#fde047', accent: '#facc15', glow: 'rgba(253, 224, 71, 0.5)' },
                sleepy: { eye: '#6b7280', accent: '#4b5563', glow: 'rgba(107, 114, 128, 0.3)' }
            },
            OceanNan: {
                normal: { eye: '#38bdf8', accent: '#0ea5e9', glow: 'rgba(56, 189, 248, 0.5)' },
                speaking: { eye: '#7dd3fc', accent: '#38bdf8', glow: 'rgba(125, 211, 252, 0.6)' },
                thinking: { eye: '#22d3ee', accent: '#06b6d4', glow: 'rgba(34, 211, 238, 0.5)' },
                listening: { eye: '#67e8f9', accent: '#22d3ee', glow: 'rgba(103, 232, 249, 0.5)' },
                happy: { eye: '#a5f3fc', accent: '#67e8f9', glow: 'rgba(165, 243, 252, 0.6)' },
                curious: { eye: '#c4b5fd', accent: '#a78bfa', glow: 'rgba(196, 181, 253, 0.5)' },
                sleepy: { eye: '#64748b', accent: '#475569', glow: 'rgba(100, 116, 139, 0.3)' }
            },
            SunsetNan: {
                normal: { eye: '#fb923c', accent: '#f97316', glow: 'rgba(251, 146, 60, 0.5)' },
                speaking: { eye: '#fdba74', accent: '#fb923c', glow: 'rgba(253, 186, 116, 0.6)' },
                thinking: { eye: '#c084fc', accent: '#a855f7', glow: 'rgba(192, 132, 252, 0.5)' },
                listening: { eye: '#f472b6', accent: '#ec4899', glow: 'rgba(244, 114, 182, 0.5)' },
                happy: { eye: '#fcd34d', accent: '#fbbf24', glow: 'rgba(252, 211, 77, 0.6)' },
                curious: { eye: '#e879f9', accent: '#d946ef', glow: 'rgba(232, 121, 249, 0.5)' },
                sleepy: { eye: '#78716c', accent: '#57534e', glow: 'rgba(120, 113, 108, 0.3)' }
            },
            FestivalNan: {
                normal: { eye: '#f43f5e', accent: '#e11d48', glow: 'rgba(244, 63, 94, 0.6)' },
                speaking: { eye: '#8b5cf6', accent: '#7c3aed', glow: 'rgba(139, 92, 246, 0.6)' },
                thinking: { eye: '#06b6d4', accent: '#0891b2', glow: 'rgba(6, 182, 212, 0.6)' },
                listening: { eye: '#10b981', accent: '#059669', glow: 'rgba(16, 185, 129, 0.6)' },
                happy: { eye: '#eab308', accent: '#ca8a04', glow: 'rgba(234, 179, 8, 0.7)' },
                curious: { eye: '#ec4899', accent: '#db2777', glow: 'rgba(236, 72, 153, 0.6)' },
                sleepy: { eye: '#6b7280', accent: '#4b5563', glow: 'rgba(107, 114, 128, 0.3)' }
            }
        };

        return SKINS[skinName];
    }

    playAnimation(animationName) {
        if (!window.AnimationController) return;

        // Map animation names to methods
        const animationMap = {
            'dance-hiphop': 'danceHipHop',
            'dance-ballet': 'danceBallet',
            'dance-disco': 'danceDisco',
            'dance-robot': 'danceRobot',
            'dance-breakdance': 'danceBreakdance',
            'jump': 'jump',
            'spin': 'spin',
            'wave': 'wave',
            'bow': 'bow',
            'celebrate': 'celebrate',
            'sleep': 'sleep',
            'exercise': 'exercise',
            'fly': 'fly'
        };

        const methodName = animationMap[animationName];
        if (methodName && typeof window.AnimationController[methodName] === 'function') {
            window.AnimationController[methodName]();
            document.getElementById('currentAnimation').textContent = animationName.replace(/-/g, ' ').replace(/\b\w/g, l => l.toUpperCase());
        }
    }

    // ==========================================
    // DRAG & DROP
    // ==========================================

    initDragDrop() {
        const container = document.getElementById('robot-master-container');
        if (!container) return;

        let isDragging = false;
        let startX, startY, initialX = 0, initialY = 0;

        container.addEventListener('mousedown', (e) => {
            isDragging = true;
            container.classList.add('dragging');
            startX = e.clientX - initialX;
            startY = e.clientY - initialY;
        });

        document.addEventListener('mousemove', (e) => {
            if (!isDragging) return;

            e.preventDefault();
            initialX = e.clientX - startX;
            initialY = e.clientY - startY;

            container.style.transform = `translate(${initialX}px, ${initialY}px)`;
        });

        document.addEventListener('mouseup', () => {
            isDragging = false;
            container.classList.remove('dragging');
        });

        // Double click for surprise
        container.addEventListener('dblclick', () => {
            this.surpriseAction();
        });
    }

    surpriseAction() {
        const surprises = ['celebrate', 'spin', 'jump', 'fly'];
        const randomSurprise = surprises[Math.floor(Math.random() * surprises.length)];
        this.playAnimation(randomSurprise);
    }

    // ==========================================
    // GALLERY
    // ==========================================

    initGallery() {
        const galleryGrid = document.getElementById('galleryGrid');
        if (!galleryGrid) return;

        const skins = [
            { name: 'NanRobot', label: 'น่าน', gradient: 'linear-gradient(135deg, #40c4ff, #00e5ff)' },
            { name: 'GoldenNan', label: 'ทอง', gradient: 'linear-gradient(135deg, #ffd700, #ffb800)' },
            { name: 'NightNan', label: 'กลางคืน', gradient: 'linear-gradient(135deg, #8b9dc3, #6a7fdb)' },
            { name: 'SakuraNan', label: 'ซากุระ', gradient: 'linear-gradient(135deg, #ffb6c1, #ff91a4)' },
            { name: 'CyberNan', label: 'ไซเบอร์', gradient: 'linear-gradient(135deg, #00ffff, #ff00ff)' },
            { name: 'ForestNan', label: 'ป่า', gradient: 'linear-gradient(135deg, #4ade80, #22c55e)' },
            { name: 'OceanNan', label: 'ทะเล', gradient: 'linear-gradient(135deg, #38bdf8, #0ea5e9)' },
            { name: 'SunsetNan', label: 'ตะวันตก', gradient: 'linear-gradient(135deg, #fb923c, #f97316)' },
            { name: 'FestivalNan', label: 'เทศกาล', gradient: 'linear-gradient(135deg, #f43f5e, #8b5cf6)' }
        ];

        skins.forEach(skin => {
            const card = document.createElement('div');
            card.className = 'gallery-card';
            card.innerHTML = `
                <div class="gallery-preview" style="background: ${skin.gradient};"></div>
                <div class="gallery-info">
                    <h3>${skin.label}</h3>
                    <button class="gallery-btn" data-skin="${skin.name}">Preview</button>
                </div>
            `;

            card.querySelector('.gallery-btn').addEventListener('click', () => {
                this.showSection('playground');
                setTimeout(() => {
                    this.setSkin(skin.name);
                    document.querySelectorAll('.skin-btn').forEach(btn => {
                        btn.classList.toggle('active', btn.dataset.skin === skin.name);
                    });
                }, 300);
            });

            galleryGrid.appendChild(card);
        });
    }

    // ==========================================
    // ANIMATION LIBRARY
    // ==========================================

    initAnimationLibrary() {
        const library = document.getElementById('animationLibrary');
        if (!library) return;

        const animations = [
            { name: 'dance-hiphop', label: 'Hip Hop Dance', icon: '🎤', category: 'dance' },
            { name: 'dance-ballet', label: 'Ballet Dance', icon: '🩰', category: 'dance' },
            { name: 'dance-disco', label: 'Disco Dance', icon: '🕺', category: 'dance' },
            { name: 'dance-robot', label: 'Robot Dance', icon: '🤖', category: 'dance' },
            { name: 'dance-breakdance', label: 'Breakdance', icon: '🎪', category: 'dance' },
            { name: 'jump', label: 'Jump', icon: '⬆️', category: 'action' },
            { name: 'spin', label: 'Spin', icon: '🌀', category: 'action' },
            { name: 'wave', label: 'Wave', icon: '👋', category: 'action' },
            { name: 'bow', label: 'Bow', icon: '🙇', category: 'action' },
            { name: 'celebrate', label: 'Celebrate', icon: '🎉', category: 'action' },
            { name: 'sleep', label: 'Sleep', icon: '💤', category: 'emotion' },
            { name: 'exercise', label: 'Exercise', icon: '💪', category: 'action' },
            { name: 'fly', label: 'Fly', icon: '🚀', category: 'action' }
        ];

        animations.forEach(anim => {
            const card = document.createElement('div');
            card.className = 'animation-card';
            card.dataset.category = anim.category;
            card.innerHTML = `
                <div class="animation-card-icon">${anim.icon}</div>
                <h3>${anim.label}</h3>
                <button class="animation-card-btn" data-animation="${anim.name}">Play</button>
            `;

            card.querySelector('.animation-card-btn').addEventListener('click', () => {
                this.showSection('playground');
                setTimeout(() => {
                    this.playAnimation(anim.name);
                }, 300);
            });

            library.appendChild(card);
        });

        // Category filter
        document.querySelectorAll('.category-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const category = btn.dataset.category;

                document.querySelectorAll('.category-btn').forEach(b => b.classList.remove('active'));
                btn.classList.add('active');

                document.querySelectorAll('.animation-card').forEach(card => {
                    if (category === 'all' || card.dataset.category === category) {
                        card.style.display = 'block';
                    } else {
                        card.style.display = 'none';
                    }
                });
            });
        });
    }

    playSound(type) {
        if (window.SoundManager) {
            window.SoundManager.play(type);
        }
    }
}

// Initialize app when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    const app = new AvatarStudioApp();
    app.init();
    window.AvatarStudioApp = app;
});