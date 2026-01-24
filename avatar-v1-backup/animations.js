/**
 * animations.js - Advanced Animation System
 * Handles all dance moves, actions, and complex animations
 */

class AnimationController {
    constructor() {
        this.isAnimating = false;
        this.currentAnimation = null;
        this.container = null;
        this.body = null;
        this.head = null;
        this.leftArm = null;
        this.rightArm = null;
        this.eyes = null;
    }

    init() {
        this.container = document.getElementById('robot-master-container');
        this.body = this.container?.querySelector('.robot-body');
        this.head = this.container?.querySelector('.robot-head');
        this.leftArm = document.getElementById('left-arm');
        this.rightArm = document.getElementById('right-arm');
        this.eyes = this.container?.querySelectorAll('.robot-eye');

        console.log('✅ Animation Controller initialized');
    }

    // ==========================================
    // DANCE ANIMATIONS
    // ==========================================

    danceHipHop() {
        if (this.isAnimating) return;
        this.isAnimating = true;
        this.currentAnimation = 'Hip Hop Dance';

        const timeline = gsap.timeline({
            onComplete: () => {
                this.isAnimating = false;
                this.currentAnimation = null;
            }
        });

        // Hip hop groove
        timeline
            .to(this.container, {
                rotation: -5,
                y: -10,
                duration: 0.3,
                ease: 'power2.out'
            })
            .to(this.leftArm, {
                rotation: -90,
                y: -20,
                duration: 0.3
            }, '<')
            .to(this.rightArm, {
                rotation: 45,
                x: 20,
                duration: 0.3
            }, '<')
            .to(this.container, {
                rotation: 5,
                y: 0,
                duration: 0.3,
                ease: 'power2.inOut'
            })
            .to(this.leftArm, {
                rotation: 45,
                x: -20,
                y: 0,
                duration: 0.3
            }, '<')
            .to(this.rightArm, {
                rotation: -90,
                x: 0,
                y: -20,
                duration: 0.3
            }, '<')
            .to(this.container, {
                rotation: -5,
                y: -10,
                duration: 0.3
            })
            .to(this.container, {
                rotation: 5,
                y: 0,
                duration: 0.3
            })
            .to(this.container, {
                rotation: 0,
                y: 0,
                duration: 0.4,
                ease: 'power2.inOut'
            })
            .to([this.leftArm, this.rightArm], {
                rotation: 0,
                x: 0,
                y: 0,
                duration: 0.4
            }, '<');

        this.playSound('dance');
    }

    danceBallet() {
        if (this.isAnimating) return;
        this.isAnimating = true;
        this.currentAnimation = 'Ballet Dance';

        const timeline = gsap.timeline({
            onComplete: () => {
                this.isAnimating = false;
                this.currentAnimation = null;
            }
        });

        // Graceful ballet spin
        timeline
            .to(this.leftArm, {
                rotation: -120,
                y: -40,
                duration: 0.6,
                ease: 'power2.out'
            })
            .to(this.rightArm, {
                rotation: 120,
                y: -40,
                duration: 0.6,
                ease: 'power2.out'
            }, '<')
            .to(this.container, {
                rotation: 360,
                duration: 1.5,
                ease: 'power2.inOut'
            }, '<')
            .to(this.body, {
                y: -30,
                duration: 0.75,
                ease: 'power2.out'
            }, '<')
            .to(this.body, {
                y: 0,
                duration: 0.75,
                ease: 'power2.in'
            })
            .to([this.leftArm, this.rightArm], {
                rotation: 0,
                y: 0,
                duration: 0.5,
                ease: 'power2.inOut'
            }, '-=0.3');

        this.playSound('dance');
    }

    danceDisco() {
        if (this.isAnimating) return;
        this.isAnimating = true;
        this.currentAnimation = 'Disco Dance';

        const timeline = gsap.timeline({
            onComplete: () => {
                this.isAnimating = false;
                this.currentAnimation = null;
            }
        });

        // Disco fever
        for (let i = 0; i < 4; i++) {
            timeline
                .to(this.leftArm, {
                    rotation: i % 2 === 0 ? -45 : 45,
                    y: i % 2 === 0 ? -30 : 0,
                    duration: 0.2
                })
                .to(this.rightArm, {
                    rotation: i % 2 === 0 ? 45 : -45,
                    y: i % 2 === 0 ? 0 : -30,
                    duration: 0.2
                }, '<')
                .to(this.head, {
                    rotation: i % 2 === 0 ? 10 : -10,
                    duration: 0.2
                }, '<')
                .to(this.container, {
                    y: i % 2 === 0 ? -15 : 0,
                    duration: 0.2
                }, '<');
        }

        timeline
            .to([this.leftArm, this.rightArm, this.head], {
                rotation: 0,
                y: 0,
                duration: 0.3
            })
            .to(this.container, {
                y: 0,
                duration: 0.3
            }, '<');

        this.playSound('dance');
    }

    danceRobot() {
        if (this.isAnimating) return;
        this.isAnimating = true;
        this.currentAnimation = 'Robot Dance';

        const timeline = gsap.timeline({
            onComplete: () => {
                this.isAnimating = false;
                this.currentAnimation = null;
            }
        });

        // Mechanical robot moves
        timeline
            .to(this.leftArm, {
                rotation: -90,
                duration: 0.15,
                ease: 'steps(1)'
            })
            .to(this.head, {
                rotation: 20,
                duration: 0.15,
                ease: 'steps(1)'
            }, '<')
            .to(this.rightArm, {
                rotation: 90,
                duration: 0.15,
                ease: 'steps(1)'
            })
            .to(this.head, {
                rotation: -20,
                duration: 0.15,
                ease: 'steps(1)'
            }, '<')
            .to(this.leftArm, {
                rotation: 0,
                duration: 0.15,
                ease: 'steps(1)'
            })
            .to(this.rightArm, {
                rotation: 0,
                duration: 0.15,
                ease: 'steps(1)'
            }, '<')
            .to(this.head, {
                rotation: 0,
                duration: 0.15,
                ease: 'steps(1)'
            }, '<')
            .to(this.container, {
                rotation: 90,
                duration: 0.2,
                ease: 'steps(1)'
            })
            .to(this.container, {
                rotation: 180,
                duration: 0.2,
                ease: 'steps(1)'
            })
            .to(this.container, {
                rotation: 270,
                duration: 0.2,
                ease: 'steps(1)'
            })
            .to(this.container, {
                rotation: 360,
                duration: 0.2,
                ease: 'steps(1)'
            });

        this.playSound('robot');
    }

    danceBreakdance() {
        if (this.isAnimating) return;
        this.isAnimating = true;
        this.currentAnimation = 'Breakdance';

        const timeline = gsap.timeline({
            onComplete: () => {
                this.isAnimating = false;
                this.currentAnimation = null;
            }
        });

        // Power moves
        timeline
            .to(this.container, {
                rotation: 180,
                y: -50,
                duration: 0.6,
                ease: 'power2.out'
            })
            .to(this.leftArm, {
                rotation: -180,
                duration: 0.6
            }, '<')
            .to(this.rightArm, {
                rotation: 180,
                duration: 0.6
            }, '<')
            .to(this.container, {
                rotation: 360,
                duration: 0.6,
                ease: 'power2.inOut'
            })
            .to(this.container, {
                y: 0,
                duration: 0.4,
                ease: 'bounce.out'
            })
            .to([this.leftArm, this.rightArm], {
                rotation: 0,
                duration: 0.4
            }, '<');

        this.playSound('dance');
    }

    // ==========================================
    // ACTION ANIMATIONS
    // ==========================================

    jump() {
        if (this.isAnimating) return;
        this.isAnimating = true;
        this.currentAnimation = 'Jump';

        gsap.timeline({
            onComplete: () => {
                this.isAnimating = false;
                this.currentAnimation = null;
            }
        })
            .to(this.container, {
                y: -100,
                duration: 0.4,
                ease: 'power2.out'
            })
            .to(this.leftArm, {
                rotation: -30,
                y: -20,
                duration: 0.4
            }, '<')
            .to(this.rightArm, {
                rotation: 30,
                y: -20,
                duration: 0.4
            }, '<')
            .to(this.container, {
                y: 0,
                duration: 0.4,
                ease: 'bounce.out'
            })
            .to([this.leftArm, this.rightArm], {
                rotation: 0,
                y: 0,
                duration: 0.3
            }, '<');

        this.playSound('jump');
    }

    spin() {
        if (this.isAnimating) return;
        this.isAnimating = true;
        this.currentAnimation = 'Spin';

        gsap.timeline({
            onComplete: () => {
                this.isAnimating = false;
                this.currentAnimation = null;
            }
        })
            .to(this.container, {
                rotation: 720,
                duration: 1.5,
                ease: 'power2.inOut'
            })
            .to(this.leftArm, {
                rotation: -90,
                duration: 0.75
            }, '<')
            .to(this.rightArm, {
                rotation: 90,
                duration: 0.75
            }, '<')
            .to([this.leftArm, this.rightArm], {
                rotation: 0,
                duration: 0.5
            });

        this.playSound('whoosh');
    }

    wave() {
        if (this.isAnimating) return;
        this.isAnimating = true;
        this.currentAnimation = 'Wave';

        gsap.timeline({
            onComplete: () => {
                this.isAnimating = false;
                this.currentAnimation = null;
            }
        })
            .to(this.rightArm, {
                rotation: -45,
                y: -30,
                duration: 0.3,
                ease: 'power2.out'
            })
            .to(this.rightArm, {
                rotation: -30,
                duration: 0.15
            })
            .to(this.rightArm, {
                rotation: -45,
                duration: 0.15
            })
            .to(this.rightArm, {
                rotation: -30,
                duration: 0.15
            })
            .to(this.rightArm, {
                rotation: -45,
                duration: 0.15
            })
            .to(this.rightArm, {
                rotation: 5,
                y: 0,
                duration: 0.4,
                ease: 'power2.in'
            });

        this.playSound('wave');
    }

    bow() {
        if (this.isAnimating) return;
        this.isAnimating = true;
        this.currentAnimation = 'Bow';

        gsap.timeline({
            onComplete: () => {
                this.isAnimating = false;
                this.currentAnimation = null;
            }
        })
            .to(this.body, {
                rotation: 30,
                transformOrigin: 'center bottom',
                duration: 0.6,
                ease: 'power2.out'
            })
            .to(this.leftArm, {
                rotation: -20,
                duration: 0.6
            }, '<')
            .to(this.rightArm, {
                rotation: 20,
                duration: 0.6
            }, '<')
            .to({}, { duration: 0.5 })
            .to(this.body, {
                rotation: 0,
                duration: 0.5,
                ease: 'power2.in'
            })
            .to([this.leftArm, this.rightArm], {
                rotation: 0,
                duration: 0.5
            }, '<');

        this.playSound('bow');
    }

    celebrate() {
        if (this.isAnimating) return;
        this.isAnimating = true;
        this.currentAnimation = 'Celebrate';

        const timeline = gsap.timeline({
            onComplete: () => {
                this.isAnimating = false;
                this.currentAnimation = null;
            }
        });

        // Celebration jumps
        for (let i = 0; i < 3; i++) {
            timeline
                .to(this.container, {
                    y: -60,
                    duration: 0.3,
                    ease: 'power2.out'
                })
                .to(this.leftArm, {
                    rotation: -120,
                    y: -30,
                    duration: 0.3
                }, '<')
                .to(this.rightArm, {
                    rotation: 120,
                    y: -30,
                    duration: 0.3
                }, '<')
                .to(this.container, {
                    y: 0,
                    duration: 0.3,
                    ease: 'bounce.out'
                })
                .to([this.leftArm, this.rightArm], {
                    rotation: 0,
                    y: 0,
                    duration: 0.3
                }, '<');
        }

        this.playSound('celebrate');
        this.createConfetti();
    }

    sleep() {
        if (this.isAnimating) return;
        this.isAnimating = true;
        this.currentAnimation = 'Sleep';

        gsap.timeline({
            onComplete: () => {
                this.isAnimating = false;
                this.currentAnimation = null;
            }
        })
            .to(this.head, {
                rotation: 15,
                duration: 1,
                ease: 'power2.out'
            })
            .to(this.eyes, {
                scaleY: 0.1,
                duration: 0.5
            }, '<')
            .to({}, { duration: 2 })
            .to(this.head, {
                rotation: 0,
                duration: 0.8
            })
            .to(this.eyes, {
                scaleY: 1,
                duration: 0.3
            }, '<');

        this.playSound('sleep');
        this.createZzz();
    }

    exercise() {
        if (this.isAnimating) return;
        this.isAnimating = true;
        this.currentAnimation = 'Exercise';

        const timeline = gsap.timeline({
            onComplete: () => {
                this.isAnimating = false;
                this.currentAnimation = null;
            }
        });

        // Push-ups motion
        for (let i = 0; i < 5; i++) {
            timeline
                .to(this.leftArm, {
                    rotation: -90,
                    duration: 0.3
                })
                .to(this.rightArm, {
                    rotation: 90,
                    duration: 0.3
                }, '<')
                .to(this.body, {
                    y: 20,
                    duration: 0.3
                }, '<')
                .to(this.leftArm, {
                    rotation: -45,
                    duration: 0.3
                })
                .to(this.rightArm, {
                    rotation: 45,
                    duration: 0.3
                }, '<')
                .to(this.body, {
                    y: 0,
                    duration: 0.3
                }, '<');
        }

        timeline
            .to([this.leftArm, this.rightArm], {
                rotation: 0,
                duration: 0.4
            });

        this.playSound('exercise');
    }

    fly() {
        if (this.isAnimating) return;
        this.isAnimating = true;
        this.currentAnimation = 'Fly';

        gsap.timeline({
            onComplete: () => {
                this.isAnimating = false;
                this.currentAnimation = null;
            }
        })
            .to(this.leftArm, {
                rotation: -120,
                duration: 0.4
            })
            .to(this.rightArm, {
                rotation: 120,
                duration: 0.4
            }, '<')
            .to(this.container, {
                y: -150,
                duration: 1.5,
                ease: 'power2.out'
            })
            .to(this.leftArm, {
                rotation: -90,
                duration: 0.3,
                repeat: 3,
                yoyo: true
            }, '<')
            .to(this.rightArm, {
                rotation: 90,
                duration: 0.3,
                repeat: 3,
                yoyo: true
            }, '<')
            .to(this.container, {
                y: 0,
                duration: 1,
                ease: 'power2.in'
            })
            .to([this.leftArm, this.rightArm], {
                rotation: 0,
                duration: 0.4
            }, '<');

        this.playSound('fly');
    }

    // ==========================================
    // HELPER FUNCTIONS
    // ==========================================

    createConfetti() {
        const colors = ['#ff6b6b', '#4ecdc4', '#45b7d1', '#f7b731', '#5f27cd'];
        const stage = document.querySelector('.avatar-stage');

        for (let i = 0; i < 50; i++) {
            const confetti = document.createElement('div');
            confetti.style.position = 'absolute';
            confetti.style.width = '10px';
            confetti.style.height = '10px';
            confetti.style.background = colors[Math.floor(Math.random() * colors.length)];
            confetti.style.left = '50%';
            confetti.style.top = '30%';
            confetti.style.borderRadius = '50%';
            confetti.style.zIndex = '100';
            stage.appendChild(confetti);

            gsap.to(confetti, {
                x: (Math.random() - 0.5) * 400,
                y: Math.random() * 400 + 200,
                rotation: Math.random() * 720,
                opacity: 0,
                duration: 2 + Math.random(),
                ease: 'power2.out',
                onComplete: () => confetti.remove()
            });
        }
    }

    createZzz() {
        const stage = document.querySelector('.avatar-stage');
        
        for (let i = 0; i < 3; i++) {
            setTimeout(() => {
                const zzz = document.createElement('div');
                zzz.textContent = 'Z';
                zzz.style.position = 'absolute';
                zzz.style.left = '55%';
                zzz.style.top = '25%';
                zzz.style.fontSize = '2rem';
                zzz.style.color = 'white';
                zzz.style.fontWeight = 'bold';
                zzz.style.zIndex = '100';
                stage.appendChild(zzz);

                gsap.to(zzz, {
                    x: 30 + i * 20,
                    y: -50 - i * 30,
                    opacity: 0,
                    duration: 2,
                    ease: 'power1.out',
                    onComplete: () => zzz.remove()
                });
            }, i * 600);
        }
    }

    playSound(type) {
        if (window.SoundManager) {
            window.SoundManager.play(type);
        }
    }
}

// Create global instance
window.AnimationController = new AnimationController();