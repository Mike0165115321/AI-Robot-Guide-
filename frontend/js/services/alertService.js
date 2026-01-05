
import { CONFIG } from '../config.js';

class AlertService {
    constructor() {
        this.ws = null;
        this.isConnected = false;
        this.listeners = [];
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
    }

    connect() {
        if (this.ws && (this.ws.readyState === WebSocket.OPEN || this.ws.readyState === WebSocket.CONNECTING)) {
            return;
        }

        // URL Should be: ws://localhost:8014/api/alerts/ws
        const wsUrl = `${CONFIG.WS_URL}/alerts/ws`;
        console.log(`🔔 Connecting to Alert WS: ${wsUrl}`);

        this.ws = new WebSocket(wsUrl);

        this.ws.onopen = () => {
            console.log('✅ Alert WebSocket Connected');
            this.isConnected = true;
            this.reconnectAttempts = 0;
        };

        this.ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                if (data.type === 'alert' || data.type === 'connection_established') {
                    this._notify(data);
                }
            } catch (e) {
                console.error('Alert WS Parse Error:', e);
            }
        };

        this.ws.onclose = () => {
            console.log('❌ Alert WebSocket Closed');
            this.isConnected = false;

            if (this.reconnectAttempts < this.maxReconnectAttempts) {
                setTimeout(() => {
                    this.reconnectAttempts++;
                    console.log(`🔄 Reconnecting Alert WS (${this.reconnectAttempts})...`);
                    this.connect();
                }, 5000 * this.reconnectAttempts);
            }
        };

        this.ws.onerror = (err) => {
            // Suppress error logs to avoid spamming if backend is offline
        };
    }

    onAlert(callback) {
        this.listeners.push(callback);
    }

    _notify(data) {
        this.listeners.forEach(cb => cb(data));
    }
}

export const alertService = new AlertService();
export default alertService;
