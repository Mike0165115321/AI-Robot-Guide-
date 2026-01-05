/**
 * # Config - ค่าคงที่และ Configuration
 */

export const CONFIG = {
    // API Endpoints
    API_BASE_URL: 'http://localhost:8014/api',

    // WebSocket (Base URL)
    WS_URL: 'ws://localhost:8014/api',

    // Avatar
    AVATAR_PATH: '../avatar/',

    // Settings
    DEBUG: true,

    // API Endpoints
    ENDPOINTS: {
        chat: '/chat',
        tts: '/tts',
        stt: '/stt',
        locations: '/locations'
    }
};

export default CONFIG;
