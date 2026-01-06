/**
 * # Markdown Service
 * 
 * บริการแปลง Markdown เป็น HTML
 * ใช้ marked.js เป็น parser หลัก
 * 
 * @example
 * import { renderMarkdown, renderInline } from './services/markdownService.js';
 * 
 * const html = renderMarkdown('**สวัสดี** น้องน่าน');
 * const inline = renderInline('ข้อความ *สำคัญ*');
 */

// marked.js จะถูกโหลดผ่าน script tag ใน HTML
// เราจะเช็คว่ามีหรือไม่ และ fallback ถ้าไม่มี

class MarkdownService {
    constructor() {
        this.isReady = false;
        this.init();
    }

    /**
     * Initialize marked.js configuration
     */
    init() {
        if (typeof marked !== 'undefined') {
            // Configure marked.js
            marked.setOptions({
                breaks: true,          // แปลง \n เป็น <br>
                gfm: true,             // GitHub Flavored Markdown
                headerIds: false,      // ไม่ใส่ id ใน headers
                mangle: false,         // ไม่เข้ารหัส email
                sanitize: false,       // ไม่ sanitize (เราจะจัดการเอง)
            });
            this.isReady = true;
            console.log('✅ Markdown Service: Ready');
        } else {
            console.warn('⚠️ Markdown Service: marked.js not loaded, using fallback');
        }
    }

    /**
     * แปลง Markdown เป็น HTML (Block Level)
     * @param {string} text - ข้อความ Markdown
     * @returns {string} HTML
     */
    render(text) {
        if (!text) return '';

        if (this.isReady) {
            try {
                return marked.parse(text);
            } catch (e) {
                console.error('Markdown parse error:', e);
                return this.fallbackRender(text);
            }
        }
        return this.fallbackRender(text);
    }

    /**
     * แปลง Markdown เป็น HTML (Inline Only - ไม่มี <p> wrapper)
     * @param {string} text - ข้อความ Markdown
     * @returns {string} HTML
     */
    renderInline(text) {
        if (!text) return '';

        if (this.isReady && typeof marked.parseInline === 'function') {
            try {
                return marked.parseInline(text);
            } catch (e) {
                console.error('Markdown inline parse error:', e);
                return this.fallbackRender(text);
            }
        }
        return this.fallbackRender(text);
    }

    /**
     * Fallback renderer เมื่อ marked.js ไม่พร้อม
     * @param {string} text 
     * @returns {string}
     */
    fallbackRender(text) {
        // Simple fallback: แปลงเฉพาะ basic formatting
        return text
            .replace(/\*\*(.+?)\*\*/g, '<strong>$1</strong>')  // **bold**
            .replace(/\*(.+?)\*/g, '<em>$1</em>')              // *italic*
            .replace(/`(.+?)`/g, '<code>$1</code>')            // `code`
            .replace(/\n/g, '<br>');                           // newlines
    }

    /**
     * Sanitize HTML เพื่อป้องกัน XSS
     * @param {string} html 
     * @returns {string}
     */
    sanitize(html) {
        // Basic sanitization - remove script tags
        return html
            .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '')
            .replace(/on\w+="[^"]*"/gi, '')
            .replace(/on\w+='[^']*'/gi, '');
    }

    /**
     * Render และ Sanitize พร้อมกัน
     * @param {string} text 
     * @returns {string}
     */
    renderSafe(text) {
        return this.sanitize(this.render(text));
    }
}

// Singleton instance
const markdownService = new MarkdownService();

// Export functions
export function renderMarkdown(text) {
    return markdownService.render(text);
}

export function renderInline(text) {
    return markdownService.renderInline(text);
}

export function renderMarkdownSafe(text) {
    return markdownService.renderSafe(text);
}

export default markdownService;
