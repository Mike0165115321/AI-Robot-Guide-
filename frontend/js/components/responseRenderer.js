
/**
 * # ResponseRenderer - Generate HTML for Rich Cards
 */

export const responseRenderer = {
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
                return this.renderNavigation(payload.data);
            default:
                return `<div class="response-card"><p>${JSON.stringify(payload)}</p></div>`;
        }
    },

    renderLocation(data) {
        // data: { name, description, image, distance?, rating? }
        return `
            <div class="response-card card-location">
                <h3>📍 ${data.name}</h3>
                <p>${data.description}</p>
                ${data.image ? `<img src="${data.image}" alt="${data.name}" loading="lazy">` : ''}
                ${data.distance ? `<div class="location-meta"><span>📏 ${data.distance} km</span></div>` : ''}
            </div>
        `;
    },

    renderMusic(data) {
        // data: { title, video_id, url }
        return `
            <div class="response-card card-music">
                <h3>🎵 กำลังเล่นเพลง</h3>
                <p>${data.title}</p>
                <div style="margin-top:10px;border-radius:10px;overflow:hidden;">
                    <iframe width="100%" height="160" src="https://www.youtube.com/embed/${data.video_id}?autoplay=1" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
                </div>
            </div>
        `;
    },

    renderWeather(data) {
        // data: { location, temp, condition, humidity }
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

    renderNavigation(data) {
        // data: { origin, destination, map_url }
        return `
            <div class="response-card card-navigation">
                <h3>🗺️ นำทาง: ${data.destination}</h3>
                <p>กำลังเปิดแผนที่...</p>
                <!-- Optional map embed or image -->
            </div>
        `;
    },

    renderAlert(data) {
        // Handle "info_only" or missing recommendation
        let recommendation = data.action_recommendation || 'โปรดระมัดระวังและติดตามข่าวสารอย่างใกล้ชิด';
        if (recommendation === 'info_only') recommendation = 'ติดตามข่าวสารอย่างใกล้ชิด';

        return `
            <div class="response-card card-alert-detail">
                <div style="border-bottom:1px solid rgba(255,255,255,0.1);padding-bottom:10px;margin-bottom:10px;display:flex;justify-content:space-between;align-items:center;">
                    <h3 style="color:#ef4444;margin:0;">⚠️ แจ้งเตือนภัย</h3>
                    <span style="font-size:0.8rem;opacity:0.7;">${new Date(data.broadcasted_at || Date.now()).toLocaleString('th-TH')}</span>
                </div>
                
                <h2 style="font-size:1.2rem;margin-bottom:10px;">${data.summary}</h2>
                
                ${data.location_name ? `
                <div style="background:rgba(255,255,255,0.05);padding:10px;border-radius:8px;margin-bottom:15px;">
                    <strong>📍 พื้นที่:</strong> ${data.location_name}
                </div>` : ''}

                <!-- Full Content / Description -->
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
