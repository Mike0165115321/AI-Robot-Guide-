
import { $, $$, on } from './utils/dom.js';
import { CONFIG } from './config.js';
import responseRenderer from './components/responseRenderer.js';
import { renderNavbar } from './components/Navbar.js';

const ITEMS_PER_PAGE = 12;
let currentPage = 1;
let totalAlerts = 0;

let currentSearch = '';
let currentCategory = 'all';
let searchTimeout;

document.addEventListener('DOMContentLoaded', () => {
    renderNavbar('navbar-container', 'alerts');
    loadAlerts(currentPage);
    initModal();
    initFilters();
});

function initFilters() {
    // Search Input
    const searchInput = $('#alert-search');
    if (searchInput) {
        searchInput.addEventListener('input', (e) => {
            clearTimeout(searchTimeout);
            searchTimeout = setTimeout(() => {
                currentSearch = e.target.value.trim();
                currentPage = 1;
                loadAlerts(1);
            }, 500); // Debounce 500ms
        });
    }

    // Filter Chips
    const chips = $$('.filter-chip');
    chips.forEach(chip => {
        chip.addEventListener('click', () => {
            // Remove active class from all
            chips.forEach(c => c.classList.remove('active'));
            // Add active to clicked
            chip.classList.add('active');

            // Update state
            currentCategory = chip.dataset.category;
            currentPage = 1;
            loadAlerts(1);
        });
    });
}

async function loadAlerts(page = 1) {
    currentPage = page;
    const grid = $('#alerts-grid');

    // Show loading state
    grid.innerHTML = '<div style="grid-column:1/-1;text-align:center;padding:50px;"><div class="spinner"></div><p>กำลังค้นหาข้อมูล... ⏳</p></div>';

    // Calculate skip
    const skip = (page - 1) * ITEMS_PER_PAGE;

    try {
        // Build URL parameters
        let url = `${CONFIG.API_BASE_URL}/alerts/db/recent?limit=${ITEMS_PER_PAGE}&skip=${skip}`;
        if (currentSearch) url += `&search=${encodeURIComponent(currentSearch)}`;
        if (currentCategory && currentCategory !== 'all') url += `&category=${encodeURIComponent(currentCategory)}`;

        // Fetch with pagination parameters
        const response = await fetch(url);
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
    // Date Formatting - Prioritize original news timestamp -> created_at -> current time
    const dateSource = alert.timestamp || alert.created_at || Date.now();
    const timeStr = new Date(dateSource).toLocaleString('th-TH', {
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
