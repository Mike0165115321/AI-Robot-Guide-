
import { $, $$, on } from './utils/dom.js';
import { CONFIG } from './config.js';
import responseRenderer from './components/responseRenderer.js';
import { renderNavbar } from './components/Navbar.js';

const ITEMS_PER_PAGE = 12;
let currentPage = 1;
let totalAlerts = 0;

document.addEventListener('DOMContentLoaded', () => {
    renderNavbar('navbar-container', 'alerts');
    loadAlerts(currentPage);
    initModal();
});

async function loadAlerts(page = 1) {
    currentPage = page;
    const grid = $('#alerts-grid');

    // Calculate skip
    const skip = (page - 1) * ITEMS_PER_PAGE;

    try {
        // Fetch with pagination
        const response = await fetch(`${CONFIG.API_BASE_URL}/alerts/db/recent?limit=${ITEMS_PER_PAGE}&skip=${skip}`);
        const data = await response.json();

        if (data.success) {
            totalAlerts = data.total || 0;
            const alerts = data.alerts || [];

            if (alerts.length > 0) {
                renderAlerts(alerts);
                renderPagination();
            } else {
                grid.innerHTML = '<p style="grid-column:1/-1;text-align:center;opacity:0.6;padding:40px;">ไม่พบข่าวสารในหน้านี้</p>';
                $('.pagination-container')?.remove();
            }
        }
    } catch (err) {
        console.error('Fetch Alerts Error:', err);
        grid.innerHTML = '<p style="grid-column:1/-1;text-align:center;color:red;">เกิดข้อผิดพลาดในการโหลดข้อมูล</p>';
    }
}

function renderAlerts(alerts) {
    const grid = $('#alerts-grid');
    grid.innerHTML = '';

    alerts.forEach(alert => {
        const card = createAlertCard(alert);
        grid.appendChild(card);
    });
}

function renderPagination() {
    const totalPages = Math.ceil(totalAlerts / ITEMS_PER_PAGE);

    // Check if pagination container exists, if not create it
    let pagContainer = $('#pagination-container');
    if (!pagContainer) {
        pagContainer = document.createElement('div');
        pagContainer.id = 'pagination-container';
        pagContainer.className = 'pagination-container';
        // Insert after grid
        const grid = $('#alerts-grid');
        grid.parentNode.insertBefore(pagContainer, grid.nextSibling);
    }

    pagContainer.innerHTML = '';

    if (totalPages <= 1) {
        pagContainer.style.display = 'none';
        return;
    }

    pagContainer.style.display = 'flex';

    // Prev Button
    const prevBtn = document.createElement('button');
    prevBtn.className = 'pagination-btn';
    prevBtn.innerHTML = '❮ ย้อนกลับ';
    prevBtn.disabled = currentPage === 1;
    prevBtn.onclick = () => loadAlerts(currentPage - 1);

    // Page Info
    const pageInfo = document.createElement('span');
    pageInfo.className = 'pagination-info';
    pageInfo.innerText = `หน้า ${currentPage} / ${totalPages}`;

    // Next Button
    const nextBtn = document.createElement('button');
    nextBtn.className = 'pagination-btn';
    nextBtn.innerHTML = 'ถัดไป ❯';
    nextBtn.disabled = currentPage === totalPages;
    nextBtn.onclick = () => loadAlerts(currentPage + 1);

    pagContainer.appendChild(prevBtn);
    pagContainer.appendChild(pageInfo);
    pagContainer.appendChild(nextBtn);
}

function createAlertCard(alert) {
    const article = document.createElement('article');
    article.className = 'alert-card';

    // Severity Label
    const levels = ['General', 'Info', 'Interest', 'Caution', 'Warning', 'Critical'];
    const severityIdx = Math.min(Math.max((alert.severity_score || 1), 1), 5);
    const severityClass = `severity-${severityIdx}`;

    // Date Formatting
    const timeStr = new Date(alert.broadcasted_at || Date.now()).toLocaleString('th-TH', {
        dateStyle: 'short',
        timeStyle: 'short'
    });

    article.innerHTML = `
        <div class="alert-header">
            <span class="alert-badge ${severityClass}">Level ${severityIdx}</span>
            <span class="alert-time">${timeStr}</span>
        </div>
        <h3 class="alert-title">${alert.summary}</h3>
        ${alert.location_name ? `<div class="alert-location">📍 ${alert.location_name}</div>` : ''}
        <div style="font-size:0.8rem;opacity:0.6;margin-top:10px;">Source: ${alert.original_source || 'AI Monitor'}</div>
    `;

    // Click to open modal
    on(article, 'click', () => openAlertModal(alert));

    return article;
}

// Modal Logic
const modal = $('#alert-modal');
const modalBody = $('#modal-body');
const closeBtn = $('.modal-close');

function initModal() {
    if (closeBtn) {
        on(closeBtn, 'click', closeAlertModal);
    }
    if (modal) {
        on(modal, 'click', (e) => {
            if (e.target === modal) closeAlertModal();
        });
    }
}

function openAlertModal(alert) {
    if (!modal || !modalBody) return;

    // Use responseRenderer to generate full detail HTML
    // Conveniently reuse existing rendering logic
    modalBody.innerHTML = responseRenderer.renderAlert(alert);

    modal.classList.add('visible');
    document.body.style.overflow = 'hidden'; // Prevent background scrolling
}

function closeAlertModal() {
    if (modal) modal.classList.remove('visible');
    document.body.style.overflow = '';
}
