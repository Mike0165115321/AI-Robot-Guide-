/**
 * core/EyeTracking.js - Advanced Mouse & Object Tracking for Eyes
 */

export class EyeTracking {
    constructor(controller) {
        this.controller = controller;
        this.isEnabled = true;
        this.maxMove = 12;
        this.lastMove = { x: 0, y: 0 };
    }

    init() {
        document.addEventListener('mousemove', (e) => this.handleMouseMove(e));
        console.log('👁️ EyeTracking Initialized');
    }

    handleMouseMove(event) {
        if (!this.isEnabled || !this.controller.face) return;

        const rect = this.controller.face.getBoundingClientRect();
        const centerX = rect.left + rect.width / 2;
        const centerY = rect.top + rect.height / 2;

        let moveX = (event.clientX - centerX) / 50;
        let moveY = (event.clientY - centerY) / 50;

        // Clamp values
        moveX = Math.max(-this.maxMove, Math.min(this.maxMove, moveX));
        moveY = Math.max(-this.maxMove, Math.min(this.maxMove, moveY));

        this.lastMove = { x: moveX, y: moveY };
        this.applyEyeTransform(moveX, moveY);
    }

    applyEyeTransform(x, y) {
        if (!this.controller.eyes.length) return;

        if (window.gsap) {
            gsap.to(this.controller.eyes, {
                x: x,
                y: y,
                duration: 0.4,
                ease: 'power3.out'
            });
        } else {
            this.controller.eyes.forEach(eye => {
                eye.style.transform = `translate(${x}px, ${y}px)`;
            });
        }
    }

    setEnabled(status) {
        this.isEnabled = status;
        if (!status) {
            this.applyEyeTransform(0, 0); // Reset to center
        }
    }
}
