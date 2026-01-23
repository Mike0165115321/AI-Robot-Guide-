/**
 * core/BlinkController.js - Managed Blinking Behavior
 */

export class BlinkController {
    constructor(controller) {
        this.controller = controller;
        this.blinkInterval = null;
        this.isBlinking = false;
    }

    start() {
        this.stop();
        const scheduleNext = () => {
            const delay = 2000 + Math.random() * 5000;
            this.blinkInterval = setTimeout(() => {
                this.blink();
                scheduleNext();
            }, delay);
        };
        scheduleNext();
        console.log('✨ BlinkController Started');
    }

    stop() {
        if (this.blinkInterval) {
            clearTimeout(this.blinkInterval);
            this.blinkInterval = null;
        }
    }

    blink() {
        if (!this.controller.eyes.length || this.isBlinking) return;

        this.isBlinking = true;
        if (window.gsap) {
            gsap.to(this.controller.eyes, {
                scaleY: 0.1,
                duration: 0.1,
                yoyo: true,
                repeat: 1,
                onComplete: () => { this.isBlinking = false; }
            });
        } else {
            this.controller.eyes.forEach(eye => eye.classList.add('blinking'));
            setTimeout(() => {
                this.controller.eyes.forEach(eye => eye.classList.remove('blinking'));
                this.isBlinking = false;
            }, 200);
        }
    }
}
