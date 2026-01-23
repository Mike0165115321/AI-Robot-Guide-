/**
 * particles.js - Particle System
 * Creates dynamic particle effects around the avatar
 */

class ParticleSystem {
    constructor() {
        this.container = null;
        this.particles = [];
        this.isActive = true;
        this.particleCount = 30;
    }

    init() {
        this.container = document.getElementById('particleContainer');
        if (!this.container) {
            console.warn('Particle container not found');
            return;
        }

        this.createParticles();
        this.animate();
        console.log('✅ Particle System initialized');
    }

    createParticles() {
        for (let i = 0; i < this.particleCount; i++) {
            const particle = document.createElement('div');
            particle.className = 'particle';
            
            // Random position
            particle.style.left = `${Math.random() * 100}%`;
            particle.style.top = `${Math.random() * 100}%`;
            
            // Random animation delay
            particle.style.animationDelay = `${Math.random() * 3}s`;
            particle.style.animationDuration = `${3 + Math.random() * 2}s`;
            
            this.container.appendChild(particle);
            this.particles.push(particle);
        }
    }

    animate() {
        if (!this.isActive) return;

        this.particles.forEach((particle, index) => {
            const currentTop = parseFloat(particle.style.top);
            const currentLeft = parseFloat(particle.style.left);

            // Gentle floating motion
            const newTop = currentTop + (Math.sin(Date.now() * 0.001 + index) * 0.1);
            const newLeft = currentLeft + (Math.cos(Date.now() * 0.001 + index) * 0.1);

            // Wrap around edges
            if (newTop < 0) particle.style.top = '100%';
            else if (newTop > 100) particle.style.top = '0%';
            else particle.style.top = `${newTop}%`;

            if (newLeft < 0) particle.style.left = '100%';
            else if (newLeft > 100) particle.style.left = '0%';
            else particle.style.left = `${newLeft}%`;
        });

        requestAnimationFrame(() => this.animate());
    }

    createBurst(x, y, count = 20, color = null) {
        const stage = document.querySelector('.avatar-stage');
        if (!stage) return;

        for (let i = 0; i < count; i++) {
            const particle = document.createElement('div');
            particle.className = 'burst-particle';
            particle.style.position = 'absolute';
            particle.style.width = '6px';
            particle.style.height = '6px';
            particle.style.borderRadius = '50%';
            particle.style.background = color || `hsl(${Math.random() * 360}, 70%, 60%)`;
            particle.style.left = `${x}px`;
            particle.style.top = `${y}px`;
            particle.style.zIndex = '100';
            stage.appendChild(particle);

            const angle = (Math.PI * 2 * i) / count;
            const velocity = 50 + Math.random() * 100;

            gsap.to(particle, {
                x: Math.cos(angle) * velocity,
                y: Math.sin(angle) * velocity,
                opacity: 0,
                scale: 0,
                duration: 1 + Math.random() * 0.5,
                ease: 'power2.out',
                onComplete: () => particle.remove()
            });
        }
    }

    createTrail(element, color = null) {
        const rect = element.getBoundingClientRect();
        const stage = document.querySelector('.avatar-stage');
        if (!stage) return;

        const trail = document.createElement('div');
        trail.className = 'trail-particle';
        trail.style.position = 'absolute';
        trail.style.width = '20px';
        trail.style.height = '20px';
        trail.style.borderRadius = '50%';
        trail.style.background = color || 'rgba(255, 255, 255, 0.6)';
        trail.style.left = `${rect.left + rect.width / 2}px`;
        trail.style.top = `${rect.top + rect.height / 2}px`;
        trail.style.zIndex = '5';
        trail.style.pointerEvents = 'none';
        stage.appendChild(trail);

        gsap.to(trail, {
            scale: 2,
            opacity: 0,
            duration: 0.8,
            ease: 'power2.out',
            onComplete: () => trail.remove()
        });
    }

    stop() {
        this.isActive = false;
    }

    start() {
        this.isActive = true;
        this.animate();
    }

    clear() {
        this.particles.forEach(p => p.remove());
        this.particles = [];
    }

    setTheme(theme) {
        // Adjust particle colors based on theme
        const isDark = theme === 'dark';
        this.particles.forEach(particle => {
            particle.style.background = isDark ? 'rgba(255, 255, 255, 0.6)' : 'rgba(0, 0, 0, 0.1)';
        });
    }
}

// Create global instance
window.ParticleSystem = new ParticleSystem();