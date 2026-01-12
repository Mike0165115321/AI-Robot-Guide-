
import th from '../../../js/translations/th.js';
import en from '../../../js/translations/en.js';
import zh from '../../../js/translations/zh.js';
import ja from '../../../js/translations/ja.js';
import ru from '../../../js/translations/ru.js';
import hi from '../../../js/translations/hi.js';
import ms from '../../../js/translations/ms.js';

const translations = { th, en, zh, ja, ru, hi, ms };

function updateAdminLanguage() {
    const lang = localStorage.getItem('app_language') || 'th';
    const t = translations[lang] || translations['th'];

    console.log(`[AdminI18N] Updating language to: ${lang}`);

    document.querySelectorAll('[data-i18n]').forEach(el => {
        const key = el.getAttribute('data-i18n');
        if (t[key]) {
            if (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA') {
                el.placeholder = t[key];
            } else {
                // If the translation contains HTML (e.g. <strong>), use innerHTML
                if (t[key].includes('<')) {
                    el.innerHTML = t[key];
                } else {
                    el.innerText = t[key];
                }
            }
        } else {
            console.warn(`[AdminI18N] Missing key: ${key} for lang: ${lang}`);
        }
    });

    // Special Title Update
    if (t['admin_panel_title']) {
        document.title = `${t['admin_panel_title']} - AI Guide Nan`;
    }
}

// Run on load
document.addEventListener('DOMContentLoaded', updateAdminLanguage);

// Expose update function globally just in case
window.updateAdminLanguage = updateAdminLanguage;
