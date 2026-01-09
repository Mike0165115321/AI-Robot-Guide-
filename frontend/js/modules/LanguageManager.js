import { translations } from '../translations/index.js';

class LanguageManager {
    constructor() {
        if (LanguageManager.instance) {
            return LanguageManager.instance;
        }

        // 1. Get stored language or default to 'th'
        this.currentLang = localStorage.getItem('app_language') || 'th';
        this.defaultLang = 'th';
        this.listeners = [];

        LanguageManager.instance = this;
    }

    /**
     * Get translated text with Fallback logic
     * 1. Try current language
     * 2. Try default language (th)
     * 3. Return key itself
     */
    getText(key) {
        const dict = translations[this.currentLang];
        const defaultDict = translations[this.defaultLang];

        // 1. Current Language
        if (dict && dict[key]) {
            // console.log(`[I18N] Found '${key}' in '${this.currentLang}': ${dict[key]}`);
            return dict[key];
        } else {
            // console.warn(`[I18N] '${key}' NOT found in '${this.currentLang}'`);
        }

        // 2. Fallback to Default Language
        if (this.currentLang !== this.defaultLang && defaultDict && defaultDict[key]) {
            console.warn(`[I18N] Missing translation for '${key}' in '${this.currentLang}'. Falling back to default.`);
            return defaultDict[key];
        }

        // 3. Last resort: Return key
        console.warn(`[I18N] Missing translation for '${key}' everywhere.`);
        return key;
    }

    /**
     * Change language and notify listeners
     */
    setLanguage(langCode) {
        if (!translations[langCode]) {
            console.error(`[I18N] Language '${langCode}' not supported yet.`);
            return;
        }

        this.currentLang = langCode;
        localStorage.setItem('app_language', langCode);

        // Notify all subscribers
        this.notifyListeners();

        // Dispatch global event for non-subscribers
        window.dispatchEvent(new CustomEvent('languageChanged', { detail: { language: langCode } }));

        console.log(`[I18N] Language changed to: ${langCode}`);
    }

    getCurrentLanguage() {
        return this.currentLang;
    }

    /**
     * Subscribe to language changes
     * callback: function(langCode)
     */
    subscribe(callback) {
        this.listeners.push(callback);
        // Execute immediately
        callback(this.currentLang);
    }

    notifyListeners() {
        this.listeners.forEach(callback => callback(this.currentLang));
    }
}

export const languageManager = new LanguageManager();
