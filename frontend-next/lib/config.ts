// /lib/config.ts
// Dynamic API Configuration - ปรับตัวตาม environment อัตโนมัติ

// ตรวจสอบว่า client-side หรือไม่
const isBrowser = typeof window !== 'undefined';

// ตรวจสอบว่ารันบน production (same origin) หรือ development
const isLocalDev = isBrowser
    ? (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1')
    : true; // Default to dev in SSR

// สำหรับ Production: ใช้ same origin (relative URL)
// สำหรับ Development: ใช้ localhost:9090
const API_HOST = isLocalDev ? '127.0.0.1' : (isBrowser ? window.location.hostname : 'localhost');
const API_PORT = isLocalDev ? 9090 : (isBrowser ? (window.location.port || (window.location.protocol === 'https:' ? 443 : 80)) : 9090);

// ถ้า production บน same origin ให้ใช้ '' แทน full URL
export const API_BASE_URL = isLocalDev
    ? `http://${API_HOST}:${API_PORT}`
    : (isBrowser ? `${window.location.protocol}//${window.location.host}` : 'http://localhost:9090');

// สำหรับ WebSocket
const WS_PROTOCOL = isBrowser
    ? (window.location.protocol === 'https:' ? 'wss:' : 'ws:')
    : 'ws:';

export const WS_BASE_URL = isLocalDev
    ? `ws://${API_HOST}:${API_PORT}`
    : (isBrowser ? `${WS_PROTOCOL}//${window.location.host}` : 'ws://localhost:9090');

// Export สำหรับใช้ในที่อื่น
export const config = {
    API_BASE_URL,
    WS_BASE_URL,
    isLocalDev,
    isBrowser,
};

export default config;
