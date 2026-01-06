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
            formData.append('file', audioBlob, 'voice.wav');

            // Note: client.js might not support FormData directly if it sets JSON headers automatically
            // We might need to use raw fetch here or update client.js
            // Let's use raw fetch for now for file upload
            const response = await fetch(`${CONFIG.API_BASE_URL}${CONFIG.ENDPOINTS.chat}/audio?session_id=${sessionId}`, {
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
}

export const chatService = new ChatService();
export default chatService;
