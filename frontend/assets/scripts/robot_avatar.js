// /assets/scripts/robot_avatar.js (V5.1 - FINAL FIX with Gallery + image_url)

class AvatarAnimator {
    constructor() {
        this.robotMasterContainer = document.getElementById('robot-master-container');
        this.presentationArea = document.getElementById('presentation-area');
        this.infoDisplay = document.getElementById('info-display');
        this.resultText = document.getElementById('result-text');
        this.eyeElements = document.querySelectorAll('.robot-eye');
        this.robotFace = document.getElementById('robot-face');
        this.leftArm = document.getElementById('left-arm');
        this.rightArm = document.getElementById('right-arm');

        if (!this.robotMasterContainer || !this.presentationArea || !this.infoDisplay || !this.resultText || !this.robotFace) {
            console.error("AvatarAnimator: Critical elements not found in the DOM!");
            return;
        }

        this.blinkInterval = null;
        this.isEyeTrackingEnabled = true;
        this.BASE_EYE_Y_OFFSET = -5;
        this.EYE_MOVE_SCALING_FACTOR = 80;
        this.MAX_EYE_MOVE = 10;
        this.lastMove = { x: 0, y: 0 };
        this.emotionOffsetY = 0;
        this.isSpeaking = false; // [LIP SYNC FIX] New state variable

        this.eyeQuickTos = [];
        if (typeof gsap !== 'undefined') {
            this.eyeQuickTos = Array.from(this.eyeElements).map(el => ({
                x: gsap.quickTo(el, "x", { duration: 0.4, ease: "power3" }),
                y: gsap.quickTo(el, "y", { duration: 0.4, ease: "power3" })
            }));
            this.quickSetEyeX = (value) => { this.eyeQuickTos.forEach(eye => eye.x(value)); };
            this.quickSetEyeY = (value) => { this.eyeQuickTos.forEach(eye => eye.y(value)); };
            this.bindEvents();
        } else {
            console.error("GSAP not loaded, animations disabled.");
            this.quickSetEyeX = () => { };
            this.quickSetEyeY = () => { };
        }
    }

    bindEvents() {
        window.addEventListener('mousemove', (e) => this.trackEyes(e));
    }

    trackEyes(event) {
        if (!this.isEyeTrackingEnabled || !this.robotFace) return;
        try {
            const faceRect = this.robotFace.getBoundingClientRect();
            const faceCenterX = faceRect.left + faceRect.width / 2;
            const faceCenterY = faceRect.top + faceRect.height / 2;
            const diffX = event.clientX - faceCenterX;
            const diffY = event.clientY - faceCenterY;
            let moveX = diffX / this.EYE_MOVE_SCALING_FACTOR;
            let moveY = diffY / this.EYE_MOVE_SCALING_FACTOR;
            if (typeof gsap !== 'undefined' && gsap.utils) {
                this.lastMove.x = gsap.utils.clamp(-this.MAX_EYE_MOVE, this.MAX_EYE_MOVE, moveX);
                this.lastMove.y = gsap.utils.clamp(-this.MAX_EYE_MOVE, this.MAX_EYE_MOVE, moveY);
            } else {
                this.lastMove.x = Math.max(-this.MAX_EYE_MOVE, Math.min(this.MAX_EYE_MOVE, moveX));
                this.lastMove.y = Math.max(-this.MAX_EYE_MOVE, Math.min(this.MAX_EYE_MOVE, moveY));
            }
            this.applyEyeTransform();
        } catch (e) { console.error("Error in trackEyes:", e); }
    }

    applyEyeTransform() {
        const finalY = this.BASE_EYE_Y_OFFSET + this.emotionOffsetY + this.lastMove.y;
        this.quickSetEyeX(this.lastMove.x);
        this.quickSetEyeY(finalY);
    }

    eyeBlink() {
        if (this.isSpeaking || !this.robotFace || this.eyeElements.length === 0 || typeof gsap === 'undefined') return;
        if (!this.robotFace.classList.contains('face-thinking')) {
            gsap.to(this.eyeElements, { scaleY: 0.1, duration: 0.07, transformOrigin: "center center", yoyo: true, repeat: 1, overwrite: true });
        }
    }

    startBlinkLoop() {
        this.stopBlinkLoop();
        if (typeof this.eyeBlink === 'function') {
            this.blinkInterval = setInterval(() => { if (this.isEyeTrackingEnabled) this.eyeBlink(); }, 4000 + Math.random() * 2000);
        }
    }

    stopBlinkLoop() {
        clearInterval(this.blinkInterval);
        this.blinkInterval = null;
    }

    startSpeaking() {
        this.isSpeaking = true;
        this.setEmotion('speaking');
    }

    stopSpeaking() {
        this.isSpeaking = false;
    }

    setEmotion(emotion) {
        if (!this.robotFace || !this.leftArm || !this.rightArm || this.eyeElements.length === 0 || typeof gsap === 'undefined') return;
        this.stopBlinkLoop();
        gsap.killTweensOf(this.eyeElements);
        gsap.set(this.eyeElements, { scaleY: 1 });
        const classesToRemove = ['face-speaking', 'face-thinking', 'face-listening', 'face-normal'];
        const armClassesToRemove = ['arm-speaking', 'arm-thinking', 'arm-listening', 'arm-normal'];
        this.robotFace.classList.remove(...classesToRemove);
        [this.leftArm, this.rightArm].forEach(arm => arm?.classList.remove(...armClassesToRemove));
        this.isEyeTrackingEnabled = (emotion === 'normal' || emotion === 'listening');
        this.emotionOffsetY = 0;
        switch (emotion) {
            case 'speaking':
                this.robotFace.classList.add('face-speaking');
                [this.leftArm, this.rightArm].forEach(arm => arm?.classList.add('arm-speaking'));
                break;
            case 'thinking':
                this.robotFace.classList.add('face-thinking');
                [this.leftArm, this.rightArm].forEach(arm => arm?.classList.add('arm-thinking'));
                break;
            case 'listening':
                this.robotFace.classList.add('face-listening');
                [this.leftArm, this.rightArm].forEach(arm => arm?.classList.add('arm-listening'));
                this.emotionOffsetY = 8;
                break;
            default:
                this.robotFace.classList.add('face-normal');
                [this.leftArm, this.rightArm].forEach(arm => arm?.classList.add('arm-normal'));
                break;
        }
        this.applyEyeTransform();
        if (this.isEyeTrackingEnabled) { this.startBlinkLoop(); }
        else { gsap.to(this.eyeElements, { x: 0, y: this.BASE_EYE_Y_OFFSET + this.emotionOffsetY, duration: 0.4, ease: "power3", overwrite: true }); }
    }

    enterPresentationMode(data) {
        if (!this.robotMasterContainer || !this.presentationArea || typeof gsap === 'undefined' || !data) return;

        this.updatePresentation(data);

        gsap.killTweensOf([this.robotMasterContainer, this.presentationArea]);
        gsap.timeline()
            .to(this.robotMasterContainer, { x: '32vw', scale: 0.85, duration: 1.0, ease: 'power3.inOut' })
            .to(this.presentationArea, { opacity: 1, visibility: 'visible', y: 0, duration: 0.8, ease: 'power3.out' }, "-=0.7");
    }

    exitPresentationMode() {
        if (!this.presentationArea || !this.robotMasterContainer || typeof gsap === 'undefined') return;

        gsap.killTweensOf([this.presentationArea, this.robotMasterContainer]);
        gsap.timeline()
            .to(this.presentationArea, {
                opacity: 0, y: 30, duration: 0.6, ease: 'power2.in',
                onComplete: () => {
                    if (this.presentationArea) this.presentationArea.style.visibility = 'hidden';
                    if (this.infoDisplay) this.infoDisplay.innerHTML = '';
                    if (this.resultText) this.resultText.innerHTML = '';
                }
            })
            .to(this.robotMasterContainer, { x: 0, scale: 1, duration: 1.0, ease: 'power3.inOut' }, "-=0.3");
    }

    updatePresentation(data) {
        if (!this.infoDisplay || !this.resultText || typeof gsap === 'undefined') return;

        if (data && data.html_is_pre_rendered) {
            console.log("AvatarAnimator: HTML was pre-rendered by avatar_logic. Skipping content update.");
            return;
        }
        this.infoDisplay.innerHTML = '';
        this.resultText.innerHTML = '';

        const answerText = data.answer || '';
        this.resultText.innerHTML = typeof marked !== 'undefined' ? marked.parse(answerText) : answerText;
        gsap.fromTo(this.resultText, { opacity: 0 }, { opacity: 1, duration: 0.6 });

        if (data.action === 'SHOW_MAP_EMBED' && data.action_payload) {
            const mapUrl = data.action_payload.embed_url;
            const destName = data.action_payload.destination_name || 'สถานที่';

            if (mapUrl) {
                const mapContainer = document.createElement('div');
                mapContainer.className = 'map-container';
                mapContainer.style.marginTop = '20px';
                mapContainer.style.borderRadius = '15px';
                mapContainer.style.overflow = 'hidden';
                mapContainer.style.border = '1px solid rgba(255, 255, 255, 0.2)';
                mapContainer.style.background = '#000';

                const iframe = document.createElement('iframe');
                iframe.width = '100%';
                iframe.height = '300';
                iframe.style.border = 'none';
                iframe.loading = 'lazy';
                iframe.allowFullscreen = true;
                iframe.src = mapUrl;

                mapContainer.appendChild(iframe);

                if (data.action_payload.external_link) {
                    const btnLink = document.createElement('a');
                    btnLink.href = data.action_payload.external_link;
                    btnLink.target = '_blank';
                    btnLink.innerHTML = `<i class="fa-solid fa-route" style="margin-right: 8px;"></i> เริ่มนำทางไป "${destName}"`;
                    btnLink.style.cssText = `
                        display: flex;
                        align-items: center;
                        justify-content: center;
                        gap: 8px;
                        margin-top: 15px;
                        padding: 14px 24px;
                        background: linear-gradient(135deg, #3b82f6, #2563eb);
                        color: white;
                        border-radius: 10px;
                        text-decoration: none;
                        font-weight: bold;
                        font-size: 1rem;
                        box-shadow: 0 4px 15px rgba(59, 130, 246, 0.4);
                        transition: transform 0.2s, box-shadow 0.2s;
                    `;
                    btnLink.onmouseover = () => {
                        btnLink.style.transform = 'scale(1.02)';
                        btnLink.style.boxShadow = '0 6px 20px rgba(59, 130, 246, 0.6)';
                    };
                    btnLink.onmouseout = () => {
                        btnLink.style.transform = 'scale(1)';
                        btnLink.style.boxShadow = '0 4px 15px rgba(59, 130, 246, 0.4)';
                    };

                    mapContainer.appendChild(btnLink);
                }

                this.infoDisplay.appendChild(mapContainer);
            }
        }

        // Fix: Define allImages from data
        let allImages = [];
        if (data.image_gallery && Array.isArray(data.image_gallery)) {
            allImages = [...data.image_gallery];
        }
        if (data.image_url && !allImages.includes(data.image_url)) {
            allImages.unshift(data.image_url);
        }

        if (allImages.length > 0) {
            const galleryContainer = document.createElement('div');
            galleryContainer.className = 'gallery-container';
            galleryContainer.style.cssText = `
                margin-top: 25px;
                padding: 20px;
                background: rgba(15, 23, 42, 0.6);
                border-radius: 16px;
                border: 1px solid rgba(255, 255, 255, 0.1);
                backdrop-filter: blur(10px);
            `;

            const title = document.createElement('h3');
            title.textContent = '📸 รูปภาพประกอบ';
            title.style.cssText = `
                margin: 0 0 15px 0;
                font-size: 1.1rem;
                color: #2dd4bf;
                font-weight: 600;
            `;
            galleryContainer.appendChild(title);

            const imagesWrapper = document.createElement('div');
            imagesWrapper.className = 'gallery-images-wrapper';
            imagesWrapper.style.cssText = `
                display: grid;
                grid-template-columns: repeat(auto-fill, minmax(140px, 1fr));
                gap: 12px;
            `;

            allImages.slice(0, 6).forEach((url, index) => {
                const imgContainer = document.createElement('div');
                imgContainer.style.cssText = `
                    position: relative;
                    border-radius: 12px;
                    overflow: hidden;
                    aspect-ratio: 4/3;
                    box-shadow: 0 4px 15px rgba(0, 0, 0, 0.3);
                    transition: transform 0.3s, box-shadow 0.3s;
                    cursor: pointer;
                `;
                imgContainer.onmouseover = () => {
                    imgContainer.style.transform = 'scale(1.05)';
                    imgContainer.style.boxShadow = '0 8px 25px rgba(0, 0, 0, 0.4)';
                };
                imgContainer.onmouseout = () => {
                    imgContainer.style.transform = 'scale(1)';
                    imgContainer.style.boxShadow = '0 4px 15px rgba(0, 0, 0, 0.3)';
                };
                imgContainer.onclick = () => window.open(url, '_blank');

                const img = document.createElement('img');
                img.src = url;
                img.alt = 'ภาพประกอบ';
                img.style.cssText = `
                    width: 100%;
                    height: 100%;
                    object-fit: cover;
                `;
                imgContainer.appendChild(img);
                imagesWrapper.appendChild(imgContainer);
            });

            galleryContainer.appendChild(imagesWrapper);
            this.infoDisplay.appendChild(galleryContainer);
        }

        // ❌ ลบส่วน "แหล่งข้อมูลเพิ่มเติม" ออกตามที่ user ต้องการ

        gsap.fromTo(this.infoDisplay.children, {
            opacity: 0, y: 20
        }, {
            opacity: 1, y: 0, duration: 0.5, stagger: 0.2, ease: 'power2.out', delay: 0.3
        });
    }
}

document.addEventListener('DOMContentLoaded', () => {
    if (typeof gsap === 'undefined') {
        console.error("GSAP library is not loaded!"); return;
    }
    if (typeof marked === 'undefined') {
        console.warn("Marked library not loaded.");
    }
    try {
        const animatorInstance = new AvatarAnimator();
        window.avatarAnimator = animatorInstance;
        animatorInstance.setEmotion('normal');
        console.log("AvatarAnimator initialized successfully.");
    } catch (e) {
        console.error("Failed to initialize AvatarAnimator:", e);
    }
});