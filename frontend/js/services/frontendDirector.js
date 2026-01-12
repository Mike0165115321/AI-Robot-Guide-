/**
 * # FrontendDirector.js
 * 
 * The "Brain" of the frontend. 
 * Orchestrates the decision flow: Local Logic -> Google Assistant -> RAG.
 * Designed to be easily expandable for new features.
 */




const FrontendDirector = {

    /**
     * Main Entry Point
     * @param {string} text - User input
     * @param {string} lang - Language code
     * @returns {Promise<Object>} - The decision result { source, data, handled_locally }
     */
    async decide(text, lang = 'th') {
        const t = text.trim();

        console.log(`🎬 [Director] Analyzing: "${t}" (${lang})`);

        // 1. Check Local Logic (Instant)
        const localAction = this.checkLocalKeywords(t);
        if (localAction) {
            return { type: 'LOCAL', action: localAction };
        }

        // 1.5 Check Direct Pipe (Bypass Google for Travel Queries)
        if (this.shouldBypassGoogle(t)) {
            console.log('🚀 [Director] Travel Query detected -> Direct Pipe to RAG (Skipping Google)');
            return { type: 'RAG_FALLBACK' };
        }

        // 2. Ask Google Assistant (The Secretary)
        const googleResult = await this.callGoogleAssistant(t, lang);

        if (this.isGoogleUseful(googleResult)) {
            return {
                type: 'GOOGLE',
                data: {
                    answer: googleResult.reply,
                    intent: googleResult.intent,
                    // Map intent to actions
                    action: (googleResult.intent === 'CMD_MUSIC') ? 'SHOW_SONG_CHOICES' : null,
                    avatar_mood: 'happy',
                    show_slide: false // ❌ Google Assistant: No slide by default
                }
            };
        }

        // 3. Fallback to RAG (The Expert)
        // Director doesn't call RAG directly (to avoid circular dep with chatService if possible, or just return instruction)
        return { type: 'RAG_FALLBACK' };
    },

    /**
     * Check for local keywords
     * @returns {string|null} Action ID or null
     */
    checkLocalKeywords(text) {
        const t = text.toLowerCase();

        if (t.includes('เต้น') || t.includes('dance')) return 'dance';
        if (t.includes('หัวเราะ') || t.includes('laugh')) return 'laugh';
        if (t.includes('หยุด') || t.includes('stop')) return 'stop';

        return null;
    },

    /**
     * Check if we should bypass Google and go straight to RAG (Direct Pipe)
     * For travel queries where we want our specific DB answer, not Google's generic one.
     */
    shouldBypassGoogle(text) {
        const t = text.toLowerCase();
        // Keywords from FAQ and Travel-specific intents
        const ragKeywords = [
            'แนะนำ', 'ที่เที่ยว', 'วัด', 'ร้านอาหาร', 'โรงแรม', 'ของฝาก',
            'guide', 'hotel', 'food', 'restaurant', 'attraction', 'temple',
            'น่าน', 'nan'
        ];
        return ragKeywords.some(kw => t.includes(kw));
    },

    /**
     * Call the Backend Proxy for Google Assistant
     */
    async callGoogleAssistant(text, lang) {
        try {
            const res = await fetch('/api/assistant/query', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ text: text, language: lang })
            });
            const json = await res.json();
            return json.success ? json.data : null;
        } catch (e) {
            console.warn("⚠️ [Director] Assistant Proxy Error:", e);
            return null;
        }
    },

    /**
     * Decide if Google's answer is good enough
     */
    isGoogleUseful(result) {
        if (!result) return false;
        if (result.intent === 'RAG_QUERY') return false;
        if (!result.reply || result.reply.trim() === '') return false;

        // Filter out "I don't know" responses if Google returns them as text
        const badPhrases = ["ไม่เข้าใจ", "ขอโทษ", "sorry", "i don't understand"];
        // Simple check (can be improved)
        // if (badPhrases.some(p => result.reply.toLowerCase().includes(p))) return false;

        return true;
    }
};

export default FrontendDirector;
