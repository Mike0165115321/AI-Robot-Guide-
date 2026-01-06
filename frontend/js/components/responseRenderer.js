
/**
 * # ResponseRenderer - Generate HTML for Rich Cards & Content
 * 
 * รองรับ:
 * - Location cards
 * - Music/Video embeds
 * - Weather info
 * - Navigation with map
 * - Image gallery
 * - Alerts
 * - Suggested questions
 */

import { renderMarkdown } from '../services/markdownService.js';

export const responseRenderer = {
    /**
     * Render payload based on type
     */
    render(payload) {
        if (!payload || !payload.type) return '';

        switch (payload.type) {
            case 'location':
                return this.renderLocation(payload.data);
            case 'music':
                return this.renderMusic(payload.data);
            case 'weather':
                return this.renderWeather(payload.data);
            case 'navigation':
            case 'map':
                return this.renderNavigation(payload.data);
            case 'gallery':
                return this.renderGallery(payload.data);
            case 'text':
                return this.renderText(payload.data);
            default:
                return `<div class="response-card"><p>${JSON.stringify(payload)}</p></div>`;
        }
    },

    /**
     * Render AI response with optional enrichments
     */
    renderAIResponse(data) {
        let html = '';

        // Main answer with Markdown
        if (data.answer) {
            html += `<div class="ai-answer">${renderMarkdown(data.answer)}</div>`;
        }

        // Image gallery
        if (data.image_gallery && data.image_gallery.length > 0) {
            html += this.renderGallery(data.image_gallery);
        } else if (data.image_url) {
            html += `<div class="single-image"><img src="${data.image_url}" alt="รูปภาพประกอบ" loading="lazy"></div>`;
        }

        // Map embed
        if (data.action === 'SHOW_MAP_EMBED' && data.action_payload) {
            html += this.renderMapEmbed(data.action_payload);
        }

        // Suggested questions
        if (data.suggested_questions && data.suggested_questions.length > 0) {
            html += this.renderSuggestedQuestions(data.suggested_questions);
        }

        // Processing time
        if (data.processing_time) {
            html += `<div class="processing-time">⏱️ AI Time: ${data.processing_time}s</div>`;
        }

        return html;
    },

    /**
     * Render location card
     */
    renderLocation(data) {
        return `
            <div class="response-card card-location">
                <h3>📍 ${data.name}</h3>
                <p>${data.description || ''}</p>
                ${data.image ? `<img src="${data.image}" alt="${data.name}" loading="lazy">` : ''}
                ${data.distance ? `<div class="location-meta"><span>📏 ${data.distance} km</span></div>` : ''}
                ${data.rating ? `<div class="location-meta"><span>⭐ ${data.rating}</span></div>` : ''}
            </div>
        `;
    },

    /**
     * Render music player
     */
    renderMusic(data) {
        return `
            <div class="response-card card-music">
                <h3>🎵 กำลังเล่นเพลง</h3>
                <p>${data.title}</p>
                <div style="margin-top:10px;border-radius:10px;overflow:hidden;">
                    <iframe width="100%" height="160" 
                        src="https://www.youtube.com/embed/${data.video_id}?autoplay=1" 
                        frameborder="0" 
                        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" 
                        allowfullscreen></iframe>
                </div>
            </div>
        `;
    },

    /**
     * Render weather info
     */
    renderWeather(data) {
        return `
            <div class="response-card card-weather">
                <div class="temp">${data.temp}°C</div>
                <div>
                    <h3>${data.location}</h3>
                    <p>${data.condition}</p>
                    <small>💧 ความชื้น ${data.humidity}%</small>
                </div>
            </div>
        `;
    },

    /**
     * Render navigation/map
     */
    renderNavigation(data) {
        const mapHtml = data.embed_url ? this.renderMapEmbed(data) : '';
        return `
            <div class="response-card card-navigation">
                <h3>🗺️ นำทาง: ${data.destination || data.destination_name || ''}</h3>
                ${mapHtml}
            </div>
        `;
    },

    /**
     * Render map embed
     */
    renderMapEmbed(data) {
        const navButton = data.external_link ? `
            <a href="${data.external_link}" target="_blank" rel="noopener" 
               class="nav-btn" style="
                display: inline-flex;
                align-items: center;
                gap: 8px;
                padding: 10px 20px;
                background: linear-gradient(135deg, #3b82f6, #2563eb);
                border-radius: 8px;
                color: white;
                text-decoration: none;
                font-weight: bold;
                margin-top: 10px;
            ">
                🚗 เริ่มนำทาง
            </a>
        ` : '';

        return `
            <div class="map-embed-container" style="margin-top:15px;border-radius:10px;overflow:hidden;">
                <div style="background:rgba(0,0,0,0.5);padding:8px 12px;display:flex;justify-content:space-between;align-items:center;">
                    <span style="font-size:0.85rem;color:var(--color-accent);">
                        📍 ${data.destination_name || 'แผนที่'}
                    </span>
                    <a href="${data.embed_url}" target="_blank" style="font-size:0.8rem;color:var(--color-primary);">
                        ↗ ขยาย
                    </a>
                </div>
                <iframe src="${data.embed_url}" width="100%" height="250" 
                    style="border:0;" allowfullscreen loading="lazy"></iframe>
                <div style="padding:10px;text-align:center;">
                    ${navButton}
                </div>
            </div>
        `;
    },

    /**
     * Render image gallery
     */
    renderGallery(images) {
        if (!images || images.length === 0) return '';

        const imagesHtml = images.map(img => {
            const url = typeof img === 'string' ? img : img.url;
            const alt = typeof img === 'string' ? 'รูปภาพ' : (img.alt || 'รูปภาพ');
            return `<img src="${url}" alt="${alt}" loading="lazy" onclick="window.open('${url}', '_blank')">`;
        }).join('');

        return `
            <div class="image-gallery" style="
                display: grid;
                grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
                gap: 8px;
                margin-top: 15px;
            ">
                ${imagesHtml}
            </div>
        `;
    },

    /**
     * Render suggested questions
     */
    renderSuggestedQuestions(questions, onClick) {
        if (!questions || questions.length === 0) return '';

        const btns = questions.map(q =>
            `<button class="suggestion-chip" data-question="${q}">${q}</button>`
        ).join('');

        return `
            <div class="suggested-questions" style="
                display: flex;
                flex-wrap: wrap;
                gap: 8px;
                margin-top: 15px;
            ">
                ${btns}
            </div>
        `;
    },

    /**
     * Render text with Markdown
     */
    renderText(data) {
        const text = typeof data === 'string' ? data : data.text;
        return `<div class="response-card">${renderMarkdown(text)}</div>`;
    },

    /**
     * Render alert details
     */
    renderAlert(data) {
        let recommendation = data.action_recommendation || 'โปรดระมัดระวังและติดตามข่าวสารอย่างใกล้ชิด';
        if (recommendation === 'info_only') recommendation = 'ติดตามข่าวสารอย่างใกล้ชิด';

        // Determine Header Style based on Severity
        const severity = data.severity_score || 1;
        let headerTitle = '📰 ข่าวสาร';
        let headerColor = '#10b981'; // Green
        let headerIcon = '📰';

        if (severity >= 4) {
            headerTitle = '⚠️ แจ้งเตือนภัย';
            headerColor = '#ef4444'; // Red
            headerIcon = '⚠️';
        } else if (severity === 3) {
            headerTitle = '📢 น่าสนใจ';
            headerColor = '#f59e0b'; // Yellow/Orange
            headerIcon = '📢';
        }

        // Date Handling
        const dateSource = data.timestamp || data.created_at || Date.now();
        const timeStr = new Date(dateSource).toLocaleString('th-TH');

        return `
            <div class="response-card card-alert-detail">
                <div style="border-bottom:1px solid rgba(255,255,255,0.1);padding-bottom:10px;margin-bottom:10px;display:flex;justify-content:space-between;align-items:center;">
                    <h3 style="color:${headerColor};margin:0;">${headerIcon} ${headerTitle}</h3>
                    <span style="font-size:0.8rem;opacity:0.7;">${timeStr}</span>
                </div>
                
                <h2 style="font-size:1.2rem;margin-bottom:10px;">${data.summary}</h2>
                
                ${data.location_name ? `
                <div style="background:rgba(255,255,255,0.05);padding:10px;border-radius:8px;margin-bottom:15px;">
                    <strong>📍 พื้นที่:</strong> ${data.location_name}
                </div>` : ''}

                ${data.original_body ? `
                <div style="margin-bottom:15px;font-size:0.95rem;line-height:1.5;opacity:0.9;background:rgba(0,0,0,0.2);padding:10px;border-radius:6px;">
                    ${data.original_body}
                </div>` : ''}

                <div style="margin-bottom:15px;">
                    <strong>💡 คำแนะนำ:</strong>
                    <p style="color:#fbbf24;">${recommendation}</p>
                </div>

                <div style="font-size:0.8rem;opacity:0.6;text-align:right;">
                   Source: ${data.original_source || 'AI Monitor'}
                </div>
            </div>
        `;
    }
};

export default responseRenderer;
