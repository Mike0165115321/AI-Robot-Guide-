
import { CONFIG } from '../config.js';

class AvatarService {
    constructor() {
        this.ws = null;
        this.isConnected = false;
        this.listeners = [];
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
    }

    connect(sessionId) {
        if (this.ws && (this.ws.readyState === WebSocket.OPEN || this.ws.readyState === WebSocket.CONNECTING)) {
            return;
        }

        const wsUrl = `${CONFIG.WS_BASE_URL}/api/avatar/ws?client_id=${sessionId}`;
        console.log(`🔌 Connecting to Avatar WS: ${wsUrl}`);

        this.ws = new WebSocket(wsUrl);

        this.ws.onopen = () => {
            console.log('✅ Avatar WebSocket Connected');
            this.isConnected = true;
            this.reconnectAttempts = 0;
            this._notify({ type: 'connection_status', status: 'connected' });
        };

        this.ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                this._notify(data);
            } catch (e) {
                console.error('WS Parse Error:', e);
            }
        };

        this.ws.onclose = () => {
            console.log('❌ Avatar WebSocket Closed');
            this.isConnected = false;
            this._notify({ type: 'connection_status', status: 'disconnected' });

            if (this.reconnectAttempts < this.maxReconnectAttempts) {
                setTimeout(() => {
                    this.reconnectAttempts++;
                    console.log(`🔄 Reconnecting (${this.reconnectAttempts}/${this.maxReconnectAttempts})...`);
                    this.connect(sessionId);
                }, 2000 * this.reconnectAttempts);
            }
        };

        this.ws.onerror = (err) => {
            console.error('Avatar WebSocket Error:', err);
        };
    }

    disconnect() {
        if (this.ws) {
            this.ws.close();
        }
    }

    send(data) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(data));
        } else {
            console.warn('⚠️ Avatar WS not connected, cannot send message');
        }
    }

    onMessage(callback) {
        this.listeners.push(callback);
    }

    _notify(data) {
        this.listeners.forEach(cb => cb(data));
    }
}

export const avatarService = new AvatarService();
export default avatarService;
