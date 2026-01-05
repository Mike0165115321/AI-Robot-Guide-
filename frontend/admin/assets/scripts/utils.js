/**
 * Shared Utils for AI Guide Nan
 * v2.0.0 - Simplified with API-driven image URLs
 */

// Global constant for placeholder image
const PLACEHOLDER_IMG = 'data:image/svg+xml;charset=UTF-8,%3Csvg%20width%3D%22100%22%20height%3D%2275%22%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%20100%2075%22%20preserveAspectRatio%3D%22none%22%3E%3Cdefs%3E%3Cstyle%20type%3D%22text%2Fcss%22%3E%23holder_1%20text%20%7B%20fill%3A%23AAAAAA%3Bfont-weight%3Abold%3Bfont-family%3AArial%2C%20Helvetica%2C%20Open%20Sans%2C%20sans-serif%2C%20monospace%3Bfont-size%3A10pt%20%7D%20%3C%2Fstyle%3E%3C%2Fdefs%3E%3Cg%20id%3D%22holder_1%22%3E%3Crect%20width%3D%22100%22%20height%3D%2275%22%20fill%3D%22%23EEEEEE%22%3E%3C%2Frect%3E%3Cg%3E%3Ctext%20x%3D%2227.5%22%20y%3D%2242%22%3ENo Image%3C%2Ftext%3E%3C%2Fg%3E%3C%2Fg%3E%3C%2Fsvg%3E';

const DYNAMIC_PLACEHOLDER_BASE = 'https://placehold.co/600x400/1e293b/94a3b8?text=';

/**
 * ดึง URL รูปภาพสำหรับ location item
 * v2.0 - ใช้ image_urls จาก API เป็นหลัก (Exact Match จาก Backend)
 * 
 * @param {Object} item - Location object จาก API
 * @param {string} apiBaseUrl - Base URL ของ API
 * @returns {Object} { primaryUrl, placeholder, imgOnError }
 */
function getLocationImages(item, apiBaseUrl = '') {
    // ใช้ global API_BASE_URL ถ้าไม่ได้ส่งมา
    if (!apiBaseUrl && typeof API_BASE_URL !== 'undefined') {
        apiBaseUrl = API_BASE_URL;
    }

    // สร้าง placeholder พร้อมชื่อสถานที่
    let placeholder = PLACEHOLDER_IMG;
    if (item.title) {
        placeholder = DYNAMIC_PLACEHOLDER_BASE + encodeURIComponent(item.title);
    }

    let primaryUrl = placeholder;

    // 1. ลำดับแรก: preview_image_url (จาก Admin API)
    if (item.preview_image_url) {
        primaryUrl = item.preview_image_url.startsWith('/')
            ? `${apiBaseUrl}${item.preview_image_url}`
            : item.preview_image_url;
    }
    // 2. ลำดับสอง: image_urls[] จาก API (Exact Match จาก Backend)
    else if (item.image_urls && item.image_urls.length > 0) {
        primaryUrl = item.image_urls[0];
        if (primaryUrl.startsWith('/')) {
            primaryUrl = `${apiBaseUrl}${primaryUrl}`;
        }
    }
    // 3. ไม่มีรูป → แสดง placeholder (ไม่เดา URL อีกต่อไป)

    // onError handler: ถ้าโหลดไม่ได้ ให้แสดง placeholder
    const imgOnError = `this.onerror=null; this.src='${placeholder}';`;

    return {
        primaryUrl,
        placeholder,
        imgOnError
    };
}

/**
 * แก้ไข URL รูปภาพให้ถูกต้อง
 * - แปลง 0.0.0.0 เป็น hostname ปัจจุบัน
 * - แปลง relative URL ให้มี full path
 */
function fixImageUrl(url) {
    if (!url) return null;

    // ถ้าเป็น Data URI หรือ External URL ที่ถูกต้องอยู่แล้ว (และไม่ใช่ 0.0.0.0)
    if (url.startsWith('data:') || (url.startsWith('http') && !url.includes('0.0.0.0'))) {
        return url;
    }

    // แก้ 0.0.0.0
    if (url.includes('0.0.0.0')) {
        const correctHost = window.API_HOST || window.location.hostname;
        return url.replace('0.0.0.0', correctHost);
    }

    // แก้ Relative URL
    if (url.startsWith('/')) {
        const baseUrl = window.API_BASE_URL || '';
        return `${baseUrl}${url}`;
    }

    return url;
}
