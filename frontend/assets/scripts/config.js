// /frontend/assets/scripts/config.js
// Dynamic API Configuration - ปรับตัวตาม environment อัตโนมัติ

// ตรวจสอบว่ารันบน production (same origin) หรือ development
const isLocalDev = window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1';

// สำหรับ Production: ใช้ same origin (relative URL)
// สำหรับ Development: ใช้ localhost:9090
const API_HOST = isLocalDev ? '127.0.0.1' : window.location.hostname;
const API_PORT = isLocalDev ? 9090 : window.location.port || (window.location.protocol === 'https:' ? 443 : 80);

// ถ้า production บน same origin ให้ใช้ '' แทน full URL
const API_BASE_URL = isLocalDev
    ? `http://${API_HOST}:${API_PORT}`
    : `${window.location.protocol}//${window.location.host}`;

// สำหรับ WebSocket
const WS_PROTOCOL = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
const WS_BASE_URL = isLocalDev
    ? `ws://${API_HOST}:${API_PORT}`
    : `${WS_PROTOCOL}//${window.location.host}`;

// Export to window for other scripts
window.API_HOST = API_HOST;
window.API_PORT = API_PORT;
window.API_BASE_URL = API_BASE_URL;
window.WS_BASE_URL = WS_BASE_URL;