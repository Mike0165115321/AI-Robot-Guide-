/**
 * # LanguageUtils.js
 * Utilities for language detection and processing.
 */

/**
 * Detect language from text
 * @param {string} text 
 * @returns {string} 'th' | 'en' | 'ja' | 'zh'
 */
export function detect(text) {
    if (!text) return 'th';

    // Simple regex for Thai characters
    const thaiRegex = /[\u0E00-\u0E7F]/;

    // Japanese (Hiragana, Katakana, Kanji)
    const japRegex = /[\u3040-\u309F\u30A0-\u30FF\u4E00-\u9FAF]/;

    // Chinese (Simplified/Traditional) - overlaps with Kanji but distinguishing simple case
    const zhRegex = /[\u4E00-\u9FFF]/;

    if (thaiRegex.test(text)) return 'th';
    if (japRegex.test(text)) return 'ja'; // Prioritize logic as needed

    // Default to English if ASCII or unknown
    return 'en';
}

/**
 * Check if text contains typical "Question words"
 */
export function isQuestion(text, lang = 'th') {
    if (lang === 'th') {
        return /(ไหม|อะไร|ที่ไหน|อย่างไร|ทำไม|เท่าไหร่)/.test(text) || text.includes('?');
    }
    return text.includes('?');
}
