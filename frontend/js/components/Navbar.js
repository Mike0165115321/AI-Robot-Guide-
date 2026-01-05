export function renderNavbar(containerId = 'navbar-container', activePage = '') {
    const container = document.getElementById(containerId);
    if (!container) {
        console.error(`Navbar container #${containerId} not found.`);
        return;
    }

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
        { name: 'หน้าหลัก', link: 'index.html', icon: '🏠', id: 'home' },
        { name: 'สถานที่', link: 'places.html', icon: '📍', id: 'places' },
        { name: 'พูดคุย', link: 'chat.html', icon: '💬', id: 'chat' },
        { name: 'ข่าวสาร', link: 'alerts.html', icon: '📢', id: 'alerts' },
        { name: 'เกี่ยวกับ', link: 'about.html', icon: 'ℹ️', id: 'about' }
    ];

    const adminDropdownItems = [
        { name: '🔒 ระบบ Admin', link: 'admin/login.html' },
        { name: '⚙️ ตั้งค่าระบบ', link: 'admin/settings.html' }
    ];

    // Detect if we are in a subdirectory (like /admin/)
    // Simple heuristic: count depth or check specific folder
    // Since we are in frontend/js/components/ imported by file, relative paths depend on the HOSTING page.
    // If hosting page is /admin/index.html, then 'index.html' means /admin/index.html, which is WRONG for Home.
    // We need to go up: '../index.html'.

    // Check if we are in /admin/
    const isInAdmin = window.location.pathname.includes('/admin/');
    const prefix = isInAdmin ? '../' : '';

    const adjustLink = (link) => {
        // If we are in Admin, and link is 'admin/index.html', we should just go to './index.html' or 'index.html'
        // If we are in Admin, and link is 'index.html', we should go to '../index.html'

        if (isInAdmin) {
            if (link.startsWith('admin/')) {
                return link.replace('admin/', '');
            }
            return prefix + link;
        }
        return link;
    };

    const navHtml = `
    <nav class="navbar">
        <a href="${adjustLink('index.html')}" class="navbar-brand">🤖 น้องน่าน</a>
        
        <div class="navbar-menu" id="navbar-menu">
            ${menuItems.map(item => `
                <a href="${adjustLink(item.link)}" class="navbar-link ${activePage === item.id ? 'active' : ''}">
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
        </div>
    </nav>
    `;

    container.innerHTML = navHtml;
    initNavbarLogic();
}

function initNavbarLogic() {
    const toggle = document.getElementById('navbar-toggle');
    const navMenu = document.getElementById('navbar-menu');
    const dropMenu = document.getElementById('dropdown-menu');

    if (!toggle) return;

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
        if (navMenu && !navMenu.contains(e.target) && e.target !== toggle) {
            navMenu.classList.remove('open');
        }
        if (dropMenu && !dropMenu.contains(e.target) && e.target !== toggle) {
            dropMenu.classList.remove('open');
        }
    });
}
