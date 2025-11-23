// /frontend/assets/scripts/map_handler.js

class MapHandler {
    constructor() {
        this.userLat = null;
        this.userLon = null;

        // ผูกกับ DOM Elements
        this.elements = {
            overlay: document.getElementById('map-overlay'),
            canvas: document.getElementById('map-canvas'),
            headerTitle: document.getElementById('map-header-title'),
            closeBtn: document.getElementById('close-map-btn')
        };

        this.init();
    }

    init() {
        // 1. ขอพิกัดทันทีที่โหลด
        this.requestUserLocation();

        // 2. ผูกปุ่มปิดแผนที่
        if (this.elements.closeBtn) {
            this.elements.closeBtn.addEventListener('click', () => this.closeMap());
        }
    }

    requestUserLocation() {
        if (navigator.geolocation) {
            navigator.geolocation.getCurrentPosition(
                (position) => {
                    this.userLat = position.coords.latitude;
                    this.userLon = position.coords.longitude;
                    console.log(`📍 Got Location: ${this.userLat}, ${this.userLon}`);
                },
                (error) => {
                    console.warn("⚠️ Location access denied or error:", error);
                    // อาจจะแจ้งเตือนผู้ใช้เบาๆ หรือปล่อยผ่านไป
                }
            );
        } else {
            console.error("Geolocation is not supported by this browser.");
        }
    }

    /**
     * คืนค่าพิกัดปัจจุบัน (เพื่อให้ Chat.js เรียกใช้)
     */
    getLocation() {
        return { lat: this.userLat, lon: this.userLon };
    }

    /**
     * @param {Object} payload - { embed_url, destination_name, external_link }
     */
    openMap(payload) {
        if (!this.elements.overlay || !this.elements.canvas) return;

        const { embed_url, destination_name, external_link } = payload;

        let mapHtml = `
            <iframe 
                src="${embed_url}" 
                width="100%" 
                height="100%" 
                style="border:0;" 
                allowfullscreen="" 
                loading="lazy" 
                referrerpolicy="no-referrer-when-downgrade">
            </iframe>
            
            <div style="position:absolute; bottom:30px; left:0; right:0; text-align:center; pointer-events:none;">
                <a href="${external_link}" target="_blank" 
                style="pointer-events:auto; background:#28a745; color:white; padding:12px 24px; border-radius:50px; text-decoration:none; font-weight:bold; box-shadow: 0 4px 10px rgba(0,0,0,0.3); font-family: 'Kanit', sans-serif;">
                🚗 เปิดแอป Google Maps
                </a>
            </div>
        `;

        this.elements.canvas.innerHTML = mapHtml;

        if (this.elements.headerTitle) {
            this.elements.headerTitle.textContent = `📍 ปลายทาง: ${destination_name}`;
        }
        this.elements.overlay.style.display = 'flex';
    }

    closeMap() {
        if (this.elements.overlay) {
            this.elements.overlay.style.display = 'none';
        }
        // ล้าง iframe เพื่อหยุดโหลดและประหยัดแรม
        if (this.elements.canvas) {
            this.elements.canvas.innerHTML = '';
        }
    }
}

window.mapHandler = new MapHandler();