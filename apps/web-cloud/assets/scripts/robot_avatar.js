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

        // 🔍 DEBUG: Log image data
        console.log("🖼️ [Avatar] Image Data:", {
            image_url: data.image_url,
            image_gallery: data.image_gallery
        });

        // 🆕 Strip inline markdown images (![alt](url)) from answer text
        // Because LLM may translate filenames, causing broken images
        // We display actual images in the gallery below instead
        let processedAnswer = answerText.replace(/!\[[^\]]*\]\([^)]+\)/g, '');

        // 🆕 Pre-process: Fix 0.0.0.0 in markdown BEFORE parsing
        if (typeof fixImageUrl === 'function' && processedAnswer.includes('0.0.0.0')) {
            // Replace all 0.0.0.0 with correct host in the raw text
            const correctHost = window.API_HOST || window.location.hostname;
            processedAnswer = processedAnswer.replace(/0\.0\.0\.0/g, correctHost);
        }

        let contentHtml = typeof marked !== 'undefined' ? marked.parse(processedAnswer) : processedAnswer;

        // 🆕 Restore Image Gallery (Chat Style)
        // User Request: "เอารุแบบการตอบเหมือนหน้าแชทเลยครับ อย่าไปแก้โค้ดหน้าแชทนะ ไปก็อปมาเลยก็ได้"
        let allImages = [];
        if (data.image_gallery && Array.isArray(data.image_gallery)) {
            allImages = [...data.image_gallery];
        }
        if (data.image_url && !allImages.includes(data.image_url)) {
            allImages.unshift(data.image_url);
        }

        if (allImages.length > 0) {
            contentHtml += `<div class="image-gallery-grid">`;
            allImages.slice(0, 4).forEach(url => {
                const fixedUrl = typeof fixImageUrl === 'function' ? fixImageUrl(url) : url;
                contentHtml += `<img src="${fixedUrl}" class="responsive-image" onclick="window.open('${fixedUrl}', '_blank')">`;
            });
            contentHtml += `</div>`;
        } else if (data.image_url) {
            const fixedUrl = typeof fixImageUrl === 'function' ? fixImageUrl(data.image_url) : data.image_url;
            contentHtml += `<div class="single-image-container"><img src="${fixedUrl}" class="responsive-image" onclick="window.open('${fixedUrl}', '_blank')"></div>`;
        }

        this.resultText.innerHTML = ""; // Clear initial content

        // 1. Create Main Content Wrapper
        const contentWrapper = document.createElement('div');
        contentWrapper.innerHTML = contentHtml;
        this.resultText.appendChild(contentWrapper);

        // 🆕 2. Append Action Buttons (Like, Dislike, Copy, Print)
        // User Request: "เพิ่มปุ่ม ปริ้นหรือ ปุ่ม กดดไลค์ เหมือน หน้า แชทมาด้วยนะครับ"

        const actionsContainer = document.createElement('div');
        actionsContainer.style.display = 'flex';
        actionsContainer.style.gap = '10px';
        actionsContainer.style.marginTop = '15px';
        actionsContainer.style.paddingTop = '10px';
        actionsContainer.style.borderTop = '1px solid rgba(255, 255, 255, 0.1)';
        actionsContainer.style.justifyContent = 'space-between';
        actionsContainer.style.alignItems = 'center';

        // Left Side: Feedback
        const feedbackGroup = document.createElement('div');
        feedbackGroup.style.display = 'flex';
        feedbackGroup.style.gap = '8px';

        const likeBtn = document.createElement('button');
        likeBtn.className = 'avatar-action-btn';
        likeBtn.innerHTML = '<i class="fa-regular fa-thumbs-up"></i>';
        likeBtn.title = "ถูกใจ";

        const dislikeBtn = document.createElement('button');
        dislikeBtn.className = 'avatar-action-btn';
        dislikeBtn.innerHTML = '<i class="fa-regular fa-thumbs-down"></i>';
        dislikeBtn.title = "ไม่ถูกใจ";

        // Feedback Logic
        const submitAvatarFeedback = async (type) => {
            const sessionId = sessionStorage.getItem('session_id') || 'anonymous';
            if (type === 'like') {
                likeBtn.innerHTML = '<i class="fa-solid fa-thumbs-up"></i>';
                likeBtn.style.color = '#4ade80'; // Green
                dislikeBtn.innerHTML = '<i class="fa-regular fa-thumbs-down"></i>';
                dislikeBtn.style.color = '';
            } else {
                dislikeBtn.innerHTML = '<i class="fa-solid fa-thumbs-down"></i>';
                dislikeBtn.style.color = '#f87171'; // Red
                likeBtn.innerHTML = '<i class="fa-regular fa-thumbs-up"></i>';
                likeBtn.style.color = '';
            }
            likeBtn.disabled = true;
            dislikeBtn.disabled = true;

            try {
                await fetch('/api/analytics/submit_feedback', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        session_id: sessionId,
                        query: document.getElementById('user-input')?.value || "speech_input",
                        response: answerText.substring(0, 500),
                        feedback_type: type
                    })
                });
            } catch (e) {
                console.error("Feedback failed:", e);
            }
        };

        likeBtn.onclick = () => submitAvatarFeedback('like');
        dislikeBtn.onclick = () => submitAvatarFeedback('dislike');

        feedbackGroup.appendChild(likeBtn);
        feedbackGroup.appendChild(dislikeBtn);

        // Right Side: Utility
        const utilityGroup = document.createElement('div');
        utilityGroup.style.display = 'flex';
        utilityGroup.style.gap = '8px';

        const copyBtn = document.createElement('button');
        copyBtn.className = 'avatar-action-btn';
        copyBtn.innerHTML = '<i class="fa-regular fa-copy"></i>';
        copyBtn.title = "คัดลอก";
        copyBtn.onclick = () => {
            navigator.clipboard.writeText(answerText).then(() => {
                copyBtn.innerHTML = '<i class="fa-solid fa-check"></i>';
                setTimeout(() => copyBtn.innerHTML = '<i class="fa-regular fa-copy"></i>', 2000);
            });
        };

        const printBtn = document.createElement('button');
        printBtn.className = 'avatar-action-btn';
        printBtn.innerHTML = '<i class="fa-solid fa-print"></i>';
        printBtn.title = "พิมพ์";
        printBtn.onclick = () => {
            const printWindow = window.open('', '_blank');
            let printImagesHtml = '';

            // Use same images for print
            if (allImages && allImages.length > 0) {
                printImagesHtml += '<div class="gallery">';
                allImages.forEach(img => {
                    const fixedImg = typeof fixImageUrl === 'function' ? fixImageUrl(img) : img;
                    printImagesHtml += `<img src="${fixedImg}" alt="Gallery Image">`;
                });
                printImagesHtml += '</div>';
            }

            const htmlContent = typeof marked !== 'undefined' ? marked.parse(answerText) : answerText;

            printWindow.document.write(`
            <!DOCTYPE html>
            <html lang="th">
            <head>
                <meta charset="UTF-8">
                <title>พิมพ์เอกสาร - AI Guide Nan</title>
                <link href="https://fonts.googleapis.com/css2?family=Sarabun:wght@300;400;600&display=swap" rel="stylesheet">
                <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
                <style>
                    body {
                        font-family: 'Sarabun', sans-serif;
                        line-height: 1.6;
                        color: #333;
                        max-width: 210mm; /* A4 width */
                        margin: 0 auto;
                        padding: 20px;
                        background: white;
                    }
                    @page {
                        size: A4;
                        margin: 20mm;
                    }
                    header {
                        border-bottom: 2px solid #3b82f6;
                        padding-bottom: 10px;
                        margin-bottom: 20px;
                        display: flex;
                        justify-content: space-between;
                        align-items: center;
                    }
                    .brand {
                        font-size: 1.5rem;
                        font-weight: bold;
                        color: #1e40af;
                    }
                    .date {
                        font-size: 0.9rem;
                        color: #666;
                    }
                    .content {
                        font-size: 14px;
                    }
                    h1, h2, h3 { color: #1e3a8a; margin-top: 15px; }
                    ul { margin-left: 20px; }
                    .main-image {
                        width: auto;
                        max-width: 100%;
                        max-height: 200px; /* Reduced height for print */
                        object-fit: cover;
                        border-radius: 8px;
                        margin: 10px auto;
                        display: block;
                    }
                    .content img {
                        max-width: 80%; /* Don't let inline images take full width */
                        max-height: 200px; /* Limit height */
                        width: auto;
                        display: block;
                        margin: 10px auto; /* Center */
                        border-radius: 4px;
                    }
                    .gallery {
                        display: grid;
                        grid-template-columns: repeat(3, 1fr);
                        gap: 10px;
                        margin-top: 15px;
                    }
                    .gallery img {
                        width: 100%;
                        height: 100px;
                        object-fit: cover;
                        border-radius: 4px;
                    }
                    .footer {
                        margin-top: 30px;
                        padding-top: 10px;
                        border-top: 1px solid #ddd;
                        text-align: center;
                        font-size: 0.8rem;
                        color: #888;
                    }
                    @media print {
                        body { -webkit-print-color-adjust: exact; }
                        .no-print { display: none; }
                    }
                </style>
            </head>
            <body>
                <header>
                    <div class="brand"><i class="fa-solid fa-robot"></i> AI Guide Nan</div>
                    <div class="date">พิมพ์เมื่อ: ${new Date().toLocaleString('th-TH')}</div>
                </header>
                
                <div class="content">
                    ${htmlContent}
                    ${printImagesHtml}
                </div>

                <div class="footer">
                    เอกสารนี้สร้างโดยระบบ AI Robot Guide จังหวัดน่าน | ข้อมูลเพื่อการท่องเที่ยวเท่านั้น
                </div>

                <script>
                    window.onload = () => { setTimeout(() => window.print(), 500); };
                </script>
            </body>
            </html>
        `);
            printWindow.document.close();
        }

        utilityGroup.appendChild(copyBtn);
        utilityGroup.appendChild(printBtn);

        actionsContainer.appendChild(feedbackGroup);
        actionsContainer.appendChild(utilityGroup);

        this.resultText.appendChild(actionsContainer);

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

        // ❌ Image Gallery Removed as per request


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

        // 🚀 Initialize FAB Manager
        if (window.FabManager) {
            const fabManager = new FabManager({
                buttons: {
                    music: 'avatar-music-btn',
                    faq: 'avatar-faq-btn',
                    calc: 'avatar-calc-btn',
                    nav: 'avatar-nav-btn'
                },
                callbacks: {
                    // When a widget requests to send a message (e.g. FAQ click)
                    sendMessage: (text, intent) => {
                        const inputField = document.getElementById('text-query-input');
                        if (inputField) {
                            inputField.value = text;
                            // Trigger submit manually
                            const form = document.getElementById('text-query-form');
                            if (form) form.dispatchEvent(new Event('submit'));
                        }
                    },

                    // Widget Action Handlers -> Show in Presentation Area
                    onMusicAction: () => {
                        const widget = fabManager.createMusicWidget();
                        displayWidgetInPresentation(widget);
                    },
                    onFaqAction: () => {
                        const widget = fabManager.createFAQWidget();
                        displayWidgetInPresentation(widget);
                    },
                    onNavAction: () => {
                        const widget = fabManager.createNavigationWidget();
                        displayWidgetInPresentation(widget);
                    },
                    onCalcAction: () => {
                        const widget = fabManager.createCalculatorWidget();
                        displayWidgetInPresentation(widget);
                    }
                }
            });
            console.log("🚀 FabManager initialized for Robot Avatar");

            // FAB Toggle Open/Close
            const fabToggle = document.getElementById('fab-toggle');
            const fabActions = document.getElementById('fab-actions');
            if (fabToggle && fabActions) {
                fabToggle.addEventListener('click', () => {
                    fabToggle.classList.toggle('active');
                    fabActions.classList.toggle('open');
                });
                // Close FAB when clicking any action button
                fabActions.querySelectorAll('.fab-btn').forEach(btn => {
                    btn.addEventListener('click', () => {
                        fabToggle.classList.remove('active');
                        fabActions.classList.remove('open');
                    });
                });
            }

            // AI Mode Toggle
            const aiModeToggle = document.getElementById('ai-mode-toggle');
            if (aiModeToggle && window.NanApp) {
                const modeManager = window.NanApp.getAIModeManager();
                const updateAIModeUI = (mode) => {
                    const icon = aiModeToggle.querySelector('i');
                    const span = aiModeToggle.querySelector('span');
                    if (mode === 'fast') {
                        icon.className = 'fas fa-bolt';
                        span.textContent = 'คิดเร็ว';
                    } else {
                        icon.className = 'fas fa-brain';
                        span.textContent = 'คิดละเอียด';
                    }
                };
                updateAIModeUI(modeManager.getMode());
                aiModeToggle.addEventListener('click', () => {
                    const newMode = modeManager.toggle();
                    updateAIModeUI(newMode);
                });
            }
        }

        // Helper to show FAB widgets in the presentation area
        function displayWidgetInPresentation(widgetNode) {
            const presentationArea = document.getElementById('presentation-area');
            const resultText = document.getElementById('result-text');
            const infoDisplay = document.getElementById('info-display');
            const robotContainer = document.getElementById('robot-master-container');

            if (!presentationArea || !resultText) return;

            // Clear previous content
            resultText.innerHTML = '';
            infoDisplay.innerHTML = '';

            // Append widget
            resultText.appendChild(widgetNode);

            // Animate View (similar to enterPresentationMode)
            if (typeof gsap !== 'undefined') {
                gsap.to(robotContainer, { x: '32vw', scale: 0.85, duration: 1.0, ease: 'power3.inOut' });
                gsap.to(presentationArea, { opacity: 1, visibility: 'visible', y: 0, duration: 0.8, ease: 'power3.out', delay: 0.3 });
            }
        }

    } catch (e) {
        console.error("Failed to initialize AvatarAnimator:", e);
    }
});