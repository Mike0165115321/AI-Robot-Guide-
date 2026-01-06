/**
 * # Config - ค่าคงที่และ Configuration
 */

export const CONFIG = {
    // API Endpoints
    API_BASE_URL: 'http://localhost:8014/api',

    // WebSocket Base URL
    WS_BASE_URL: 'ws://localhost:8014',

    // Static files URL (for images)
    STATIC_URL: 'http://localhost:8014/static/images',

    // Avatar
    AVATAR_PATH: '../avatar/',

    // Settings
    DEBUG: true,

    // API Endpoints
    ENDPOINTS: {
        chat: '/chat/',
        chatWs: '/api/chat/ws',
        tts: '/tts',
        stt: '/stt',
        locations: '/locations',
        alerts: '/alerts'
    }
};

/**
 * แปลง image path ให้เป็น full URL
 * @param {string} imagePath - path รูปภาพ (อาจเป็น relative หรือ full URL)
 * @returns {string} Full URL
 */
export function getFullImageUrl(imagePath) {
    if (!imagePath) return '';
    if (imagePath.startsWith('http://') || imagePath.startsWith('https://')) {
        return imagePath;
    }
    if (imagePath.startsWith('/')) {
        return `http://localhost:8014${imagePath}`;
    }
    // Relative path - add static URL
    return `${CONFIG.STATIC_URL}/${imagePath}`;
}

export default CONFIG;
