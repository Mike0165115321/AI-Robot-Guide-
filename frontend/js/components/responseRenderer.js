
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
            html += `<div class="ai-answer" id="ai-answer-content">${renderMarkdown(data.answer)}</div>`;
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

        // Footer Actions: Processing time & Print Button
        html += `<div class="response-footer" style="display: flex; justify-content: space-between; align-items: center; margin-top: 15px; padding-top: 10px; border-top: 1px solid rgba(255,255,255,0.1);">`;

        if (data.processing_time) {
            html += `<div class="processing-time" style="font-size: 0.8rem; opacity: 0.6;">⏱️ AI Time: ${data.processing_time}s</div>`;
        } else {
            html += `<div></div>`;
        }

        html += `
            <button class="btn-print" onclick="window.printCurrentResponse()" title="พิมพ์หน้านี้" 
                style="background: none; border: none; cursor: pointer; opacity: 0.7; transition: opacity 0.2s; color: var(--color-text);">
                <i class="fa-solid fa-print"></i> 🖨️ พิมพ์
            </button>
        `;

        html += `</div>`;

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
    },

    /**
     * Print functionality
     */
    printResponse(content, imageUrl, imageGallery) {
        const printWindow = window.open('', '_blank');
        if (!printWindow) {
            alert('Please allow popups for printing');
            return;
        }

        let imagesHtml = '';
        if (imageUrl) {
            imagesHtml += `<img src="${imageUrl}" class="main-image" alt="Main Image">`;
        }
        if (imageGallery && imageGallery.length > 0) {
            imagesHtml += '<div class="gallery">';
            imageGallery.forEach(img => {
                const url = typeof img === 'string' ? img : img.url;
                imagesHtml += `<img src="${url}" alt="Gallery Image">`;
            });
            imagesHtml += '</div>';
        }

        printWindow.document.write(`
            <!DOCTYPE html>
            <html lang="th">
            <head>
                <meta charset="UTF-8">
                <title>พิมพ์เอกสาร - AI Guide Nan</title>
                <link href="https://fonts.googleapis.com/css2?family=Sarabun:wght@300;400;600&display=swap" rel="stylesheet">
                <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
                <style>
                    body {
                        font-family: 'Sarabun', sans-serif;
                        line-height: 1.6;
                        color: #333;
                        max-width: 210mm;
                        margin: 0 auto;
                        padding: 20px;
                        background: white;
                    }
                    @page {
                        size: A4;
                        margin: 20mm;
                    }
                    header {
                        border-bottom: 2px solid #3b82f6;
                        padding-bottom: 10px;
                        margin-bottom: 20px;
                        display: flex;
                        justify-content: space-between;
                        align-items: center;
                    }
                    .brand {
                        font-size: 1.5rem;
                        font-weight: bold;
                        color: #1e40af;
                    }
                    .date {
                        font-size: 0.9rem;
                        color: #666;
                    }
                    .content {
                        font-size: 14px;
                    }
                    h1, h2, h3 { color: #1e3a8a; margin-top: 15px; }
                    ul { margin-left: 20px; }
                    .main-image {
                        width: auto;
                        max-width: 100%;
                        max-height: 200px;
                        object-fit: cover;
                        border-radius: 8px;
                        margin: 10px auto;
                        display: block;
                    }
                    .content img {
                        max-width: 80%;
                        max-height: 200px;
                        width: auto;
                        display: block;
                        margin: 10px auto;
                        border-radius: 4px;
                    }
                    .gallery {
                        display: grid;
                        grid-template-columns: repeat(3, 1fr);
                        gap: 10px;
                        margin-top: 15px;
                    }
                    .gallery img {
                        width: 100%;
                        height: 100px;
                        object-fit: cover;
                        border-radius: 4px;
                    }
                    .footer {
                        margin-top: 30px;
                        padding-top: 10px;
                        border-top: 1px solid #ddd;
                        text-align: center;
                        font-size: 0.8rem;
                        color: #888;
                    }
                    @media print {
                        body { -webkit-print-color-adjust: exact; }
                        .no-print { display: none; }
                    }
                </style>
            </head>
            <body>
                <header>
                    <div class="brand"><i class="fa-solid fa-robot"></i> AI Guide Nan</div>
                    <div class="date">พิมพ์เมื่อ: ${new Date().toLocaleString('th-TH')}</div>
                </header>
                
                <div class="content">
                    ${content}
                    ${imagesHtml}
                </div>

                <div class="footer">
                    เอกสารนี้สร้างโดยระบบ AI Robot Guide จังหวัดน่าน | ข้อมูลเพื่อการท่องเที่ยวเท่านั้น
                </div>

                <script>
                    window.onload = () => { setTimeout(() => window.print(), 500); };
                </script>
            </body>
            </html>
        `);
        printWindow.document.close();
    }
};

// 🌍 Global Printer for onclick events
window.printCurrentResponse = () => {
    // Helper to find the AI answer content in the DOM
    // App.js uses .response-text, renderAIResponse uses .ai-answer
    const answerEl = document.querySelector('#presentation-content .ai-answer') ||
        document.querySelector('#presentation-content .response-text');

    if (!answerEl) {
        console.warn('No content found to print');
        return;
    }

    // Attempt to find images in the panel
    const mainImage = document.querySelector('#presentation-content .single-image img');
    const galleryImages = document.querySelectorAll('#presentation-content .image-gallery img');

    // Use current src (absolute path from browser)
    const imageUrl = mainImage ? mainImage.src : null;
    let gallery = [];
    if (galleryImages.length > 0) {
        galleryImages.forEach(img => gallery.push(img.src));
    }

    responseRenderer.printResponse(answerEl.innerHTML, imageUrl, gallery);
};

export default responseRenderer;
