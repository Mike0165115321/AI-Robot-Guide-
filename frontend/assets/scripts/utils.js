/**
 * Shared Utils for AI Guide Nan
 * v1.0.0
 */

// Global constant for placeholder image
const PLACEHOLDER_IMG = 'data:image/svg+xml;charset=UTF-8,%3Csvg%20width%3D%22100%22%20height%3D%2275%22%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%20100%2075%22%20preserveAspectRatio%3D%22none%22%3E%3Cdefs%3E%3Cstyle%20type%3D%22text%2Fcss%22%3E%23holder_1%20text%20%7B%20fill%3A%23AAAAAA%3Bfont-weight%3Abold%3Bfont-family%3AArial%2C%20Helvetica%2C%20Open%20Sans%2C%20sans-serif%2C%20monospace%3Bfont-size%3A10pt%20%7D%20%3C%2Fstyle%3E%3C%2Fdefs%3E%3Cg%20id%3D%22holder_1%22%3E%3Crect%20width%3D%22100%22%20height%3D%2275%22%20fill%3D%22%23EEEEEE%22%3E%3C%2Frect%3E%3Cg%3E%3Ctext%20x%3D%2227.5%22%20y%3D%2242%22%3ENo Image%3C%2Ftext%3E%3C%2Fg%3E%3C%2Fg%3E%3C%2Fsvg%3E';

const DYNAMIC_PLACEHOLDER_BASE = 'https://placehold.co/600x400/1e293b/94a3b8?text=';

/**
 * Robustly determines the primary and secondary image URLs for a location item.
 * Mirrors the logic used in Admin Panel.
 * 
 * @param {Object} item - The location object (from API).
 * @param {string} apiBaseUrl - Base URL of the API (optional, defaults to empty or global API_BASE_URL if available).
 * @returns {Object} { primaryUrl, secondaryUrl, imgOnError }
 */
function getLocationImages(item, apiBaseUrl = '') {
    // If apiBaseUrl is not passed, try to use global variable if it exists
    if (!apiBaseUrl && typeof API_BASE_URL !== 'undefined') {
        apiBaseUrl = API_BASE_URL;
    }

    // Default Placeholder
    let placeholder = PLACEHOLDER_IMG;
    if (item.title) {
        placeholder = DYNAMIC_PLACEHOLDER_BASE + encodeURIComponent(item.title);
    }

    let primaryUrl = placeholder;
    let secondaryUrl = null;

    // 0. Priority: Preview Image URL (if exists)
    if (item.preview_image_url) {
        primaryUrl = `${apiBaseUrl}${item.preview_image_url}`;
    }
    // 0. Priority: Backend Provided Image URLs (Array)
    else if (item.image_urls && item.image_urls.length > 0) {
        primaryUrl = item.image_urls[0];
        // Ensure relative paths from backend are prefixed
        if (primaryUrl.startsWith('/')) {
            primaryUrl = `${apiBaseUrl}${primaryUrl}`;
        }
    }
    else {
        // 1. Try Image Prefix Metadata
        if (item.metadata && item.metadata.image_prefix) {
            let prefixName = item.metadata.image_prefix.trim().replace(/\s+/g, '-').replace(/-+$/, '');
            primaryUrl = `${apiBaseUrl}/static/images/${prefixName}-01.jpg`;
        }

        // 2. Prepare Slug Fallback
        if (item.slug) {
            let slugName = item.slug.trim().replace(/\s+/g, '-').replace(/-+$/, '');
            let slugUrl = `${apiBaseUrl}/static/images/${slugName}-01.jpg`;

            if (primaryUrl === placeholder) {
                primaryUrl = slugUrl; // No prefix found, so slug is primary
            } else if (primaryUrl !== slugUrl) {
                secondaryUrl = slugUrl; // Prefix exists, so slug is secondary fallback
            }
        }
    }

    let imgOnError = `this.onerror=null; this.src='${placeholder}';`;

    if (secondaryUrl) {
        imgOnError = `if (!this.dataset.triedSlug) { this.dataset.triedSlug=true; this.src='${secondaryUrl}'; } else { this.src='${placeholder}'; }`;
    }

    return {
        primaryUrl,
        secondaryUrl, // Useful if consumer wants to know
        placeholder,
        imgOnError // Ready-to-use string for onError attribute
    };
}
