import { languageManager } from '../modules/LanguageManager.js';

export function renderNavbar(containerId = 'navbar-container', activePage = '') {
    const container = document.getElementById(containerId);
    if (!container) {
        console.error(`Navbar container #${containerId} not found.`);
        return;
    }

    const updateRender = () => {
        if (!activePage) {
            const path = window.location.pathname;
            if (path.endsWith('index.html') || path === '/' || path.endsWith('/')) activePage = 'home';
            else if (path.includes('places.html')) activePage = 'places';
            else if (path.includes('chat.html')) activePage = 'chat';
            else if (path.includes('alerts.html')) activePage = 'alerts';
            else if (path.includes('about.html')) activePage = 'about';
            else if (path.includes('admin')) activePage = 'admin';
        }

        const menuItems = [
            { name: languageManager.getText('nav_home'), link: 'index.html', icon: '🏠', id: 'home' },
            { name: languageManager.getText('nav_places'), link: 'places.html', icon: '📍', id: 'places' },
            { name: languageManager.getText('nav_news'), link: 'alerts.html', icon: '📢', id: 'alerts' },
            // Linked to dedicated page
            { name: languageManager.getText('nav_language') || 'Language', link: 'language.html', icon: '🌐', id: 'language' }
        ];

        const adminDropdownItems = [
            // MOVED: About -> Dropdown
            { name: 'ℹ️ ' + languageManager.getText('nav_about'), link: 'about.html' },
            { name: '🔒 ' + languageManager.getText('nav_admin'), link: 'admin/login.html' },
            { name: '⚙️ ' + languageManager.getText('nav_settings'), link: 'admin/settings.html' }
        ];

        // Detect if we are in a subdirectory (like /admin/)
        const isInAdmin = window.location.pathname.includes('/admin/');
        const prefix = isInAdmin ? '../' : '';

        const adjustLink = (link) => {
            if (link === '#') return '#';
            if (isInAdmin) {
                if (link.startsWith('admin/')) return link.replace('admin/', '');
                return prefix + link;
            }
            return link;
        };

        const navHtml = `
        <nav class="navbar">
            <a href="${adjustLink('index.html')}" class="navbar-brand">🤖 น้องน่าน</a>
            
            <div class="navbar-menu" id="navbar-menu">
                ${menuItems.map(item => `
                    <a href="${adjustLink(item.link)}" class="navbar-link ${activePage === item.id ? 'active' : ''}" ${item.id === 'language' ? 'id="nav-btn-language"' : ''}>
                        <span class="icon">${item.icon}</span>
                        <span>${item.name}</span>
                    </a>
                `).join('')}
            </div>

            <button class="navbar-toggle" id="navbar-toggle" aria-label="Toggle menu">☰</button>
            
            <div class="dropdown-wrapper">
                <div class="dropdown-menu" id="dropdown-menu">
                    ${adminDropdownItems.map(item => `
                        <a href="${adjustLink(item.link)}" class="dropdown-item">${item.name}</a>
                    `).join('')}
                </div>
        `;

        container.innerHTML = navHtml;
        initNavbarLogic();
    };

    // Initial Render
    updateRender();

    // Subscribe to Language Changes
    // Only subscribe ONCE per page load to avoid duplicates if called multiple times?
    // Actually renderNavbar usually called once.
    languageManager.subscribe(() => {
        updateRender();
    });
}

function initNavbarLogic() {
    const toggle = document.getElementById('navbar-toggle');
    const navMenu = document.getElementById('navbar-menu');
    const dropMenu = document.getElementById('dropdown-menu');

    // Language Popup Logic
    const btnLang = document.getElementById('nav-btn-language');
    const popupOverlay = document.getElementById('language-popup-overlay');
    const closeBtn = document.getElementById('close-lang-popup');
    const langOpts = document.querySelectorAll('.lang-opt');

    if (btnLang && popupOverlay) {
        btnLang.addEventListener('click', (e) => {
            e.preventDefault();
            e.stopPropagation();
            popupOverlay.style.display = 'flex'; // Show
        });

        // Close logic
        const closePopup = () => {
            popupOverlay.style.display = 'none';
        };

        if (closeBtn) closeBtn.addEventListener('click', closePopup);
        popupOverlay.addEventListener('click', (e) => {
            if (e.target === popupOverlay) closePopup();
        });

        // Language Selection
        langOpts.forEach(btn => {
            btn.addEventListener('click', () => {
                const lang = btn.dataset.lang;
                languageManager.setLanguage(lang);

                // Visual Feedback (Active State)
                langOpts.forEach(b => b.style.background = 'rgba(255,255,255,0.05)');
                btn.style.background = 'rgba(64, 196, 255, 0.3)';

                // Close after a brief delay
                setTimeout(closePopup, 300);
            });
        });
    }

    if (!toggle) return;

    // Use replaceWith to clear old listeners if re-initializing? 
    // Or just let garbage collection handle it since innerHTML wiped the old elements.
    // Since innerHTML replaced the elements, the old listeners are attached to dead elements.
    // So we just attach new listeners to the new elements.

    toggle.addEventListener('click', (e) => {
        e.stopPropagation();
        const isMobile = window.innerWidth <= 768;
        if (isMobile) {
            if (navMenu) navMenu.classList.toggle('open');
        } else {
            if (dropMenu) dropMenu.classList.toggle('open');
        }
    });

    document.addEventListener('click', (e) => {
        if (navMenu && !navMenu.contains(e.target) && e.target !== toggle && e.target !== btnLang) {
            navMenu.classList.remove('open');
        }
        if (dropMenu && !dropMenu.contains(e.target) && e.target !== toggle) {
            dropMenu.classList.remove('open');
        }
    });
}


