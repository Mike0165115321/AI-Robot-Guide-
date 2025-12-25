// /assets/scripts/alert_handler.js
/**
 * Alert Handler - รับ alerts จาก WebSocket และแสดง UI
 * Supports: Toast notifications, Map markers, Alert dashboard
 */

class AlertHandler {
    constructor() {
        this.ws = null;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 10;
        this.reconnectDelay = 5000;
        this.toastContainer = null;
        this.connected = false;
        this.alerts = [];

        this.init();
    }

    init() {
        this.createToastContainer();
        this.connectWebSocket();

        // Ping every 30 seconds to keep connection alive
        setInterval(() => {
            if (this.ws && this.ws.readyState === WebSocket.OPEN) {
                this.ws.send('ping');
            }
        }, 30000);
    }

    createToastContainer() {
        // Check if container already exists
        if (document.querySelector('.alert-toast-container')) {
            this.toastContainer = document.querySelector('.alert-toast-container');
            return;
        }

        this.toastContainer = document.createElement('div');
        this.toastContainer.className = 'alert-toast-container';
        document.body.appendChild(this.toastContainer);
    }

    connectWebSocket() {
        const wsUrl = `${window.WS_BASE_URL}/api/alerts/ws`;
        console.log('🔗 [Alert] Connecting to:', wsUrl);

        try {
            this.ws = new WebSocket(wsUrl);

            this.ws.onopen = () => {
                console.log('✅ [Alert] WebSocket connected');
                this.connected = true;
                this.reconnectAttempts = 0;
                this.updateConnectionStatus(true);
            };

            this.ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    this.handleMessage(data);
                } catch (e) {
                    console.error('❌ [Alert] Parse error:', e);
                }
            };

            this.ws.onclose = () => {
                console.log('🔌 [Alert] WebSocket disconnected');
                this.connected = false;
                this.updateConnectionStatus(false);
                this.scheduleReconnect();
            };

            this.ws.onerror = (error) => {
                console.error('❌ [Alert] WebSocket error:', error);
            };

        } catch (e) {
            console.error('❌ [Alert] Connection error:', e);
            this.scheduleReconnect();
        }
    }

    scheduleReconnect() {
        if (this.reconnectAttempts < this.maxReconnectAttempts) {
            this.reconnectAttempts++;
            console.log(`🔄 [Alert] Reconnecting in ${this.reconnectDelay / 1000}s... (attempt ${this.reconnectAttempts})`);
            setTimeout(() => this.connectWebSocket(), this.reconnectDelay);
        } else {
            console.error('❌ [Alert] Max reconnect attempts reached');
        }
    }

    handleMessage(data) {
        const type = data.type;

        switch (type) {
            case 'connection_established':
                console.log('📥 [Alert] Connection established, recent alerts:', data.recent_alerts?.length || 0);
                if (data.recent_alerts) {
                    this.alerts = data.recent_alerts;
                }
                break;

            case 'alert':
                console.log('🚨 [Alert] New alert:', data.summary);
                this.alerts.unshift(data);
                this.showToast(data);
                this.addMapMarker(data);
                this.updateDashboard();
                break;

            case 'history':
                console.log('📜 [Alert] History received:', data.alerts?.length || 0);
                this.alerts = data.alerts || [];
                this.updateDashboard();
                break;

            case 'pong':
                // Connection alive
                break;

            default:
                console.log('📨 [Alert] Unknown message type:', type);
        }
    }

    showToast(alert) {
        const severity = alert.severity_score || 1;

        // Only show toast for severity >= 4 (as per user requirement)
        if (severity < 4) {
            return;
        }

        const toast = document.createElement('div');
        toast.className = `alert-toast severity-${severity}`;

        const icon = this.getSeverityIcon(severity);
        const categoryLabel = this.getCategoryLabel(alert.category);

        toast.innerHTML = `
            <button class="alert-toast-close" onclick="this.parentElement.remove()">×</button>
            <div class="alert-toast-header">
                <span class="alert-toast-icon">${icon}</span>
                <span class="alert-toast-category">${categoryLabel}</span>
            </div>
            <div class="alert-toast-summary">${alert.summary || 'ไม่มีรายละเอียด'}</div>
            <div class="alert-toast-meta">
                ${alert.location_name ? `
                    <span class="alert-toast-location">
                        📍 ${alert.location_name}
                    </span>
                ` : '<span></span>'}
                <span>${this.formatTime(alert.broadcasted_at)}</span>
            </div>
        `;

        // Click to view on map (if has coordinates)
        if (alert.lat && alert.lon) {
            toast.onclick = () => {
                this.focusOnLocation(alert.lat, alert.lon, alert.summary);
            };
            toast.style.cursor = 'pointer';
        }

        this.toastContainer.appendChild(toast);

        // Play sound for critical alerts
        if (severity >= 5) {
            this.playAlertSound();
        }

        // Auto remove after 10 seconds
        setTimeout(() => {
            toast.classList.add('hiding');
            setTimeout(() => toast.remove(), 300);
        }, 10000);
    }

    getSeverityIcon(severity) {
        const icons = {
            5: '🚨',
            4: '⚠️',
            3: '⚡',
            2: 'ℹ️',
            1: '📢'
        };
        return icons[severity] || '📢';
    }

    getCategoryLabel(category) {
        const labels = {
            disaster: 'ภัยพิบัติ',
            traffic: 'การจราจร',
            weather: 'สภาพอากาศ',
            health: 'สุขภาพ',
            event: 'กิจกรรม',
            general: 'ทั่วไป'
        };
        return labels[category] || category || 'แจ้งเตือน';
    }

    formatTime(isoString) {
        if (!isoString) return '';
        const date = new Date(isoString);
        return date.toLocaleTimeString('th-TH', { hour: '2-digit', minute: '2-digit' });
    }

    addMapMarker(alert) {
        // Check if MapHandler exists and has coordinates
        if (!alert.lat || !alert.lon) return;

        // TODO: Integrate with Leaflet map when available
        console.log('📍 [Alert] Map marker:', alert.lat, alert.lon, alert.summary);
    }

    focusOnLocation(lat, lon, title) {
        // TODO: Pan map to location
        console.log('🗺️ [Alert] Focus on:', lat, lon, title);
    }

    playAlertSound() {
        try {
            const audio = new Audio('data:audio/wav;base64,UklGRnoGAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YQoGAACBhYqFbF1cW2BwdH');
            audio.volume = 0.3;
            audio.play().catch(() => { });
        } catch (e) {
            // Audio not supported
        }
    }

    updateConnectionStatus(connected) {
        const statusEl = document.querySelector('.connection-status');
        if (statusEl) {
            statusEl.className = `connection-status ${connected ? 'connected' : 'disconnected'}`;
            statusEl.innerHTML = `
                <span class="connection-status-dot"></span>
                ${connected ? 'เชื่อมต่อแล้ว' : 'ไม่ได้เชื่อมต่อ'}
            `;
        }
    }

    updateDashboard() {
        // Update dashboard if on alerts page
        const alertsList = document.querySelector('.alerts-list');
        if (!alertsList) return;

        if (this.alerts.length === 0) {
            alertsList.innerHTML = `
                <div class="alerts-empty">
                    <div class="alerts-empty-icon">🔔</div>
                    <p>ยังไม่มีการแจ้งเตือน</p>
                </div>
            `;
            return;
        }

        alertsList.innerHTML = this.alerts.map(alert => this.renderAlertCard(alert)).join('');

        // Update stats
        const totalEl = document.querySelector('.stat-total');
        const criticalEl = document.querySelector('.stat-critical');

        if (totalEl) totalEl.textContent = this.alerts.length;
        if (criticalEl) {
            criticalEl.textContent = this.alerts.filter(a => a.severity_score >= 4).length;
        }
    }

    renderAlertCard(alert, index) {
        const severity = alert.severity_score || alert.sev || 1;
        const severityClass = severity >= 5 ? 'critical' : severity >= 4 ? 'warning' : '';
        const severityLabel = severity >= 5 ? 'วิกฤต' : severity >= 4 ? 'เร่งด่วน' : severity >= 3 ? 'สำคัญ' : 'ทั่วไป';

        // ใช้วันที่ภาษาไทยจาก MongoDB หรือ format เอง
        const dateDisplay = alert.created_at_th || this.formatThaiDate(alert.created_at || alert.analyzed_at);
        const summary = alert.summary || alert.sum || 'ไม่มีรายละเอียด';
        const location = alert.location_name || alert.loc || '';
        const alertId = alert._id || index;

        return `
            <div class="alert-card severity-${severity}" onclick="alertHandler.showAlertDetail('${alertId}')" style="cursor: pointer;">
                <div class="alert-card-header">
                    <span class="alert-card-category">
                        ${this.getSeverityIcon(severity)}
                        ${this.getCategoryLabel(alert.category || alert.cat)}
                    </span>
                    <span class="alert-card-severity ${severityClass}">${severityLabel}</span>
                </div>
                <div class="alert-card-summary">${summary}</div>
                <div class="alert-card-footer">
                    <span>${location ? `📍 ${location}` : ''}</span>
                    <span>📅 ${dateDisplay}</span>
                </div>
            </div>
        `;
    }

    formatThaiDate(isoString) {
        if (!isoString) return 'ไม่ระบุวันที่';
        const date = new Date(isoString);

        const thaiMonths = ['ม.ค.', 'ก.พ.', 'มี.ค.', 'เม.ย.', 'พ.ค.', 'มิ.ย.',
            'ก.ค.', 'ส.ค.', 'ก.ย.', 'ต.ค.', 'พ.ย.', 'ธ.ค.'];

        const day = date.getDate();
        const month = thaiMonths[date.getMonth()];
        const year = date.getFullYear() + 543; // แปลงเป็น พ.ศ.
        const time = date.toLocaleTimeString('th-TH', { hour: '2-digit', minute: '2-digit' });

        return `${day} ${month} ${year} ${time} น.`;
    }

    showAlertDetail(alertId) {
        const alert = this.alerts.find(a => a._id === alertId || this.alerts.indexOf(a) === parseInt(alertId));
        if (!alert) return;

        const severity = alert.severity_score || alert.sev || 1;
        const dateDisplay = alert.created_at_th || this.formatThaiDate(alert.created_at);
        const summary = alert.summary || alert.sum || 'ไม่มีรายละเอียด';
        const location = alert.location_name || alert.loc || 'ไม่ระบุ';
        const category = this.getCategoryLabel(alert.category || alert.cat);
        const source = alert.original_source || alert.source || 'ไม่ระบุ';
        const title = alert.original_title || alert.title || '';

        // สร้าง modal
        const modal = document.createElement('div');
        modal.className = 'alert-modal-overlay';
        modal.innerHTML = `
            <div class="alert-modal">
                <button class="alert-modal-close" onclick="this.parentElement.parentElement.remove()">×</button>
                <div class="alert-modal-header">
                    <span class="alert-modal-icon">${this.getSeverityIcon(severity)}</span>
                    <span class="alert-modal-category">${category}</span>
                </div>
                <div class="alert-modal-body">
                    ${title ? `<h3 class="alert-modal-title">${title}</h3>` : ''}
                    <p class="alert-modal-summary">${summary}</p>
                    <div class="alert-modal-info">
                        <div class="alert-modal-info-row">
                            <span class="label">📍 สถานที่:</span>
                            <span class="value">${location}</span>
                        </div>
                        <div class="alert-modal-info-row">
                            <span class="label">📅 วันที่:</span>
                            <span class="value">${dateDisplay}</span>
                        </div>
                        <div class="alert-modal-info-row">
                            <span class="label">📰 แหล่งที่มา:</span>
                            <span class="value">${source}</span>
                        </div>
                        <div class="alert-modal-info-row">
                            <span class="label">⚠️ ระดับความสำคัญ:</span>
                            <span class="value">${severity}/5</span>
                        </div>
                    </div>
                    ${alert.original_url ? `
                        <a href="${alert.original_url}" target="_blank" class="alert-modal-link">
                            🔗 อ่านข่าวต้นฉบับ
                        </a>
                    ` : ''}
                </div>
            </div>
        `;

        // คลิกที่ overlay เพื่อปิด
        modal.onclick = (e) => {
            if (e.target === modal) modal.remove();
        };

        document.body.appendChild(modal);
    }

    // Public API
    requestHistory() {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send('get_history');
        }
    }

    getAlerts() {
        return this.alerts;
    }

    isConnected() {
        return this.connected;
    }

    // Load alerts from MongoDB (persisted data)
    async loadFromDatabase(limit = 50) {
        try {
            const response = await fetch(`${window.API_BASE_URL}/api/alerts/db/recent?limit=${limit}`);
            const data = await response.json();

            if (data.success && data.alerts) {
                this.alerts = data.alerts;
                this.updateDashboard();
                console.log(`📥 [Alert] Loaded ${data.count} alerts from MongoDB`);
            }
        } catch (e) {
            console.error('❌ [Alert] Failed to load from DB:', e);
        }
    }

    // Load stats from MongoDB
    async loadStats() {
        try {
            const response = await fetch(`${window.API_BASE_URL}/api/alerts/db/stats`);
            const data = await response.json();

            if (data.success && data.stats) {
                const stats = data.stats;

                const totalEl = document.querySelector('.stat-total');
                const criticalEl = document.querySelector('.stat-critical');
                const todayEl = document.querySelector('.stat-today');

                if (totalEl) totalEl.textContent = stats.total_alerts || 0;
                if (criticalEl) criticalEl.textContent = stats.critical_alerts || 0;
                if (todayEl) todayEl.textContent = stats.today_alerts || 0;

                console.log('📊 [Alert] Stats loaded:', stats);
            }
        } catch (e) {
            console.error('❌ [Alert] Failed to load stats:', e);
        }
    }
}

// Initialize AlertHandler globally
window.alertHandler = new AlertHandler();

// Auto-load from database when on alerts page
document.addEventListener('DOMContentLoaded', () => {
    if (window.location.pathname.includes('alerts')) {
        // Load alerts from MongoDB
        setTimeout(() => {
            window.alertHandler.loadFromDatabase();
            window.alertHandler.loadStats();
        }, 1000);
    }
});
