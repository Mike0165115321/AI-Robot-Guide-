// /frontend/assets/scripts/debug.js
// Utility สำหรับ conditional logging - ปิดได้ง่ายตอน production

const DEBUG_MODE = false; // 🔴 เปลี่ยนเป็น false ก่อน production

const logger = {
    log: (...args) => {
        if (DEBUG_MODE) console.log(...args);
    },
    info: (...args) => {
        if (DEBUG_MODE) console.info(...args);
    },
    warn: (...args) => {
        if (DEBUG_MODE) console.warn(...args);
    },
    error: (...args) => {
        // Error ควรแสดงเสมอแม้ใน production
        console.error(...args);
    },
    debug: (...args) => {
        if (DEBUG_MODE) console.debug(...args);
    }
};

// Global error handler - จับ error ที่ไม่ได้ handle
window.onerror = function (msg, url, lineNo, columnNo, error) {
    logger.error('Global Error:', msg, 'at', url, ':', lineNo);
    return false;
};

// Promise rejection handler
window.addEventListener('unhandledrejection', function (event) {
    logger.error('Unhandled Promise Rejection:', event.reason);
});

// Export for use in other scripts
if (typeof module !== 'undefined' && module.exports) {
    module.exports = { logger, DEBUG_MODE };
}
