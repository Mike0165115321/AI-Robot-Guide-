import { languageManager } from '../js/modules/LanguageManager.js';

console.log('⚙️ Settings Page Loaded');

class SettingsPage {
    constructor() {
        console.log('Settings Page Constructor');
        this.langSelect = document.getElementById('language-select');
        this.init();
    }

    init() {
        // 1. Initial UI Update
        this.updateUI();

        // 2. Set initial value in dropdown
        if (this.langSelect) {
            this.langSelect.value = languageManager.currentLang;

            // Bind Change Event
            this.langSelect.addEventListener('change', (e) => {
                const newLang = e.target.value;
                console.log('User changed language to:', newLang);

                // alert(`Changing language to: ${newLang}`); 

                languageManager.setLanguage(newLang);
            });
        }

        // 3. Subscribe to Language Changes
        languageManager.subscribe((lang) => {
            this.updateUI();
        });
    }

    updateUI() {
        // Update all elements with [data-i18n]
        document.querySelectorAll('[data-i18n]').forEach(el => {
            const key = el.getAttribute('data-i18n');
            const text = languageManager.getText(key);

            if (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA') {
                el.placeholder = text;
            } else {
                el.innerText = text;
            }
        });

        // Optional: Update title attributes if data-i18n-title exists
        document.querySelectorAll('[data-i18n-title]').forEach(el => {
            const key = el.getAttribute('data-i18n-title');
            el.title = languageManager.getText(key);
        });
    }
}

// Start
new SettingsPage();
