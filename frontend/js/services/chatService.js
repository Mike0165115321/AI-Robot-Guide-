import api from '../api/client.js';
import { CONFIG } from '../config.js';
import aiModeManager from './aiModeManager.js';

class ChatService {
    /**
     * Send text query to backend
     * @param {string} text - User query
     * @param {string} sessionId - Session ID
     * @returns {Promise<Object>}
     */
    async sendText(text, sessionId) {
        try {
            return await api.post(CONFIG.ENDPOINTS.chat, {
                query: text,
                session_id: sessionId,
                ai_mode: aiModeManager.getMode(),
                frontend_intent: 'GENERAL'
            });
        } catch (error) {
            console.error('ChatService Error:', error);
            return { success: false, error: error.message };
        }
    }

    /**
     * Send audio blob to backend
     * @param {Blob} audioBlob - Audio recording
     * @param {string} sessionId - Session ID
     * @returns {Promise<Object>}
     */
    async sendAudio(audioBlob, sessionId) {
        try {
            const formData = new FormData();
            formData.append('file', audioBlob, 'voice.webm');

            // ใช้ endpoint /chat/transcribe ที่รองรับ Whisper STT
            const response = await fetch(`${CONFIG.API_BASE_URL}/chat/transcribe?session_id=${sessionId}`, {
                method: 'POST',
                body: formData
            });

            if (!response.ok) throw new Error(`HTTP Error: ${response.status}`);
            return await response.json();

        } catch (error) {
            console.error('Voice Error:', error);
            return { success: false, error: error.message };
        }
    }

    /**
     * Send audio for STT only
     * @param {Blob} audioBlob 
     * @returns {Promise<string>} Transcribed text
     */
    async transcribeAudio(audioBlob) {
        try {
            const formData = new FormData();
            formData.append('file', audioBlob, 'voice.webm');

            const response = await fetch(`${CONFIG.API_BASE_URL}/chat/stt`, {
                method: 'POST',
                body: formData
            });

            if (!response.ok) throw new Error(`STT failed: ${response.status}`);
            const data = await response.json();
            return data.text || '';
        } catch (error) {
            console.error('STT Error:', error);
            return '';
        }
    }
}

export const chatService = new ChatService();
export default chatService;
