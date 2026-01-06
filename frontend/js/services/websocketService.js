/**
 * # WebSocket Service
 * 
 * จัดการการเชื่อมต่อ WebSocket สำหรับ real-time chat
 * รองรับ auto-reconnect และ exponential backoff
 * 
 * @example
 * const ws = new WebSocketService('/api/chat/ws');
 * ws.onMessage((data) => console.log(data));
 * ws.connect();
 */

import { CONFIG } from '../config.js';

class WebSocketService {
    constructor(endpoint = '/api/chat/ws') {
        this.endpoint = endpoint;
        this.socket = null;
        this.reconnectAttempts = 0;
        this.maxReconnectDelay = 30000; // 30 seconds max
        this.isConnected = false;

        // Callbacks
        this.callbacks = {
            onOpen: () => { },
            onMessage: () => { },
            onClose: () => { },
            onError: () => { },
            onAudio: () => { }
        };
    }

    /**
     * เชื่อมต่อ WebSocket
     */
    connect() {
        if (this.socket && this.socket.readyState === WebSocket.OPEN) {
            console.warn('⚡ WebSocket: Already connected');
            return;
        }

        const wsUrl = this._buildWsUrl();
        console.log(`⚡ WebSocket: Connecting to ${wsUrl}`);

        this.socket = new WebSocket(wsUrl);
        this.socket.binaryType = 'arraybuffer';

        this.socket.onopen = () => {
            console.log('⚡ WebSocket: Connected');
            this.isConnected = true;
            this.reconnectAttempts = 0;
            this.callbacks.onOpen();
        };

        this.socket.onmessage = (event) => {
            if (event.data instanceof ArrayBuffer) {
                // Binary data (audio)
                this.callbacks.onAudio(event.data);
            } else {
                // Text data (JSON)
                try {
                    const data = JSON.parse(event.data);
                    this.callbacks.onMessage(data);
                } catch (e) {
                    console.error('⚡ WebSocket: Invalid JSON', e);
                }
            }
        };

        this.socket.onclose = (event) => {
            console.log('⚡ WebSocket: Closed', event.code, event.reason);
            this.isConnected = false;
            this.callbacks.onClose(event);

            // Auto-reconnect if not closed cleanly
            if (!event.wasClean) {
                this._scheduleReconnect();
            }
        };

        this.socket.onerror = (error) => {
            console.error('⚡ WebSocket: Error', error);
            this.callbacks.onError(error);
        };
    }

    /**
     * ส่งข้อความ
     */
    send(data) {
        if (!this.socket || this.socket.readyState !== WebSocket.OPEN) {
            console.error('⚡ WebSocket: Not connected');
            return false;
        }

        const message = typeof data === 'string' ? data : JSON.stringify(data);
        this.socket.send(message);
        return true;
    }

    /**
     * ปิดการเชื่อมต่อ
     */
    disconnect() {
        if (this.socket) {
            this.socket.close(1000, 'User disconnected');
            this.socket = null;
        }
    }

    /**
     * ตั้ง callback
     */
    onOpen(callback) { this.callbacks.onOpen = callback; }
    onMessage(callback) { this.callbacks.onMessage = callback; }
    onClose(callback) { this.callbacks.onClose = callback; }
    onError(callback) { this.callbacks.onError = callback; }
    onAudio(callback) { this.callbacks.onAudio = callback; }

    /**
     * สร้าง WebSocket URL
     * @private
     */
    _buildWsUrl() {
        // ใช้ WS_BASE_URL ถ้ามี
        if (CONFIG.WS_BASE_URL) {
            return `${CONFIG.WS_BASE_URL}${this.endpoint}`;
        }

        // Fallback: แปลง HTTP URL เป็น WS
        const httpUrl = CONFIG.API_BASE_URL || window.location.origin;
        const wsProtocol = httpUrl.startsWith('https') ? 'wss' : 'ws';
        const host = httpUrl.replace(/^https?:\/\//, '');
        return `${wsProtocol}://${host}${this.endpoint}`;
    }

    /**
     * ตั้งเวลา reconnect
     * @private
     */
    _scheduleReconnect() {
        const delay = Math.min(
            1000 * Math.pow(2, this.reconnectAttempts),
            this.maxReconnectDelay
        );
        this.reconnectAttempts++;

        console.log(`⚡ WebSocket: Reconnecting in ${delay / 1000}s (attempt ${this.reconnectAttempts})`);
        setTimeout(() => this.connect(), delay);
    }

    /**
     * Get connection status
     */
    getStatus() {
        return {
            isConnected: this.isConnected,
            reconnectAttempts: this.reconnectAttempts,
            readyState: this.socket?.readyState
        };
    }
}

// Singleton for chat
export const chatWebSocket = new WebSocketService('/api/chat/ws');
export default WebSocketService;
