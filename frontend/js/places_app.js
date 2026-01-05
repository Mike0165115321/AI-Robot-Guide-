
import { $, $$, on, delegate } from './utils/dom.js';
import { CONFIG } from './config.js';
import { renderNavbar } from './components/Navbar.js';

// State
let allPlaces = [];
let currentCategory = 'all';
let searchQuery = '';

document.addEventListener('DOMContentLoaded', () => {
    renderNavbar('navbar-container', 'places');
    loadPlaces();
    bindEvents();
});

async function loadPlaces() {
    const grid = $('#places-grid');

    try {
        // Use the new port from CONFIG or default relative path
        const response = await fetch(`${CONFIG.API_BASE_URL}/admin/locations/?limit=100`);
        const data = await response.json();

        if (data && data.items) {
            allPlaces = data.items;
            renderPlaces();
        } else {
            grid.innerHTML = '<p style="grid-column:1/-1;text-align:center;">ไม่พบข้อมูลสถานที่</p>';
        }
    } catch (err) {
        console.error('Fetch Places Error:', err);
        grid.innerHTML = '<p style="grid-column:1/-1;text-align:center;color:red;">เกิดข้อผิดพลาดในการโหลดข้อมูล</p>';
    }
}

function renderPlaces() {
    const grid = $('#places-grid');
    grid.innerHTML = '';

    // Filter
    // Category Mapping (English/Key -> Database Values)
    const categoryMap = {
        'all': [],
        'temple': ['temple', 'วัด', 'religion', 'ศาสนา', 'พระธาตุ', 'buddha', 'พระ', 'monastery', 'shrine', 'viharn', 'วิหาร', 'chedi', 'เจดีย์'],
        'nature': ['nature', 'natural', 'ธรรมชาติ', 'park', 'mountain', 'ดอย', 'อุทยาน', 'cave', 'ถ้ำ', 'waterfall', 'น้ำตก', 'scenic', 'ทิวทัศน์', 'adventure', 'ผจญภัย', 'viewpoint', 'จุดชมวิว'],
        'food': ['food', 'restaurant', 'market', 'อาหาร', 'ร้านอาหาร', 'ตลาด', 'cafe', 'คาเฟ่', 'drink', 'เครื่องดื่ม', 'eating', 'กิน'],
        'hotel': ['hotel', 'accommodation', 'resort', 'homestay', 'ที่พัก', 'โรงแรม', 'รีสอร์ท', 'โฮมสเตย์', 'guesthouse', 'เกสต์เฮาส์'],
        'shopping': ['shopping', 'shop', 'souvenir', 'ช้อปปิ้ง', 'ของฝาก', 'สินค้า', 'mall', 'ห้าง', 'store', 'ร้านค้า']
    };

    // Filter
    const filtered = allPlaces.filter(place => {
        // Category Match
        let categoryMatch = false;
        if (currentCategory === 'all') {
            categoryMatch = true;
        } else {
            const placeCat = (place.category || '').toLowerCase();
            const validKeywords = categoryMap[currentCategory] || [currentCategory];
            categoryMatch = validKeywords.some(kw => placeCat.includes(kw));
        }

        // Search Match
        const searchLower = searchQuery.toLowerCase();
        const searchMatch = !searchQuery ||
            place.title.toLowerCase().includes(searchLower) ||
            (place.summary && place.summary.toLowerCase().includes(searchLower)) ||
            (place.location_name && place.location_name.toLowerCase().includes(searchLower));

        return categoryMatch && searchMatch;
    });

    if (filtered.length === 0) {
        grid.innerHTML = '<p style="grid-column:1/-1;text-align:center;opacity:0.6;">ไม่พบสถานที่ที่ตรงกับเงื่อนไข</p>';
        return;
    }

    filtered.forEach(place => {
        const card = createPlaceCard(place);
        grid.appendChild(card);
    });
}

function createPlaceCard(place) {
    const article = document.createElement('article');
    article.className = 'place-card';

    // Image or Icon fallback
    let imageHtml = '<div class="place-card-image">🏞️</div>';
    if (place.preview_image_url) {
        imageHtml = `<div class="place-card-image" style="background-image:url('${place.preview_image_url}');background-size:cover;"></div>`;
    }

    // Category Icon Mapping
    const catIcons = {
        'temple': '🏛️',
        'nature': '🌲',
        'food': '🍜',
        'hotel': '🏨',
        'shopping': '🛍️'
    };
    const icon = catIcons[place.category] || '📍';

    article.innerHTML = `
        ${imageHtml}
        <div class="place-card-content">
            <span class="place-card-category">${icon} ${place.category || 'ทั่วไป'}</span>
            <h3 class="place-card-title">${place.title}</h3>
            <p class="place-card-desc">${place.summary || 'ไม่มีรายละเอียด'}</p>
        </div>
        <div class="place-card-footer">
            <span class="place-card-location">📍 ${(place.related_info && place.related_info.district) ? place.related_info.district : 'น่าน'}</span>
            <span class="place-card-rating">⭐ ${place.rating || 'N/A'}</span>
        </div>
    `;

    // Click to Open Modal
    on(article, 'click', () => openModal(place));

    return article;
}

function openModal(place) {
    const modal = $('#place-modal');
    if (!modal) return;

    // Populate Data
    const imgContainer = $('#modal-image');
    if (place.preview_image_url) {
        imgContainer.style.backgroundImage = `url('${place.preview_image_url}')`;
        imgContainer.innerHTML = '';
    } else {
        imgContainer.style.backgroundImage = 'none';
        imgContainer.innerHTML = '<div style="width:100%;height:100%;display:flex;align-items:center;justify-content:center;font-size:5rem;background:#333;">🏞️</div>';
    }

    $('#modal-category').textContent = place.category || 'ทั่วไป';
    $('#modal-title').textContent = place.title || '';

    // Choose the longer content between summary and details
    let content = place.summary || 'ไม่มีรายละเอียดเพิ่มเติม';
    if (place.details && place.details.length > (place.summary || '').length) {
        content = place.details;
    }
    $('#modal-desc').innerHTML = content.replace(/\n/g, '<br>'); // Support line breaks

    $('#modal-location').textContent = `📍 ${(place.related_info && place.related_info.district) ? place.related_info.district : 'น่าน'}`;

    // Nav Button Logic
    const lat = (place.location_data && place.location_data.latitude) ? place.location_data.latitude : '';
    const lon = (place.location_data && place.location_data.longitude) ? place.location_data.longitude : '';
    const navBtn = $('#modal-nav-btn');

    if (lat && lon) {
        navBtn.classList.remove('hidden');
        navBtn.dataset.lat = lat;
        navBtn.dataset.lon = lon;
    } else {
        navBtn.classList.add('hidden');
    }

    // Previous / Next Buttons Logic
    const currentIndex = allPlaces.indexOf(place);
    const prevBtn = $('#modal-prev-btn');
    const nextBtn = $('#modal-next-btn');

    // Remove old listeners (cloning) to prevent duplicates
    if (prevBtn) {
        const newPrev = prevBtn.cloneNode(true);
        prevBtn.parentNode.replaceChild(newPrev, prevBtn);
        newPrev.onclick = () => {
            const newIndex = (currentIndex - 1 + allPlaces.length) % allPlaces.length;
            openModal(allPlaces[newIndex]);
        };
    }

    if (nextBtn) {
        const newNext = nextBtn.cloneNode(true);
        nextBtn.parentNode.replaceChild(newNext, nextBtn);
        newNext.onclick = () => {
            const newIndex = (currentIndex + 1) % allPlaces.length;
            openModal(allPlaces[newIndex]);
        };
    }

    // Show Modal
    modal.classList.remove('hidden');
}

function bindEvents() {
    // Navigate Button (in Modal)
    const modalNavBtn = $('#modal-nav-btn');
    if (modalNavBtn) {
        on(modalNavBtn, 'click', () => {
            const lat = modalNavBtn.dataset.lat;
            const lon = modalNavBtn.dataset.lon;
            if (lat && lon) {
                window.open(`https://www.google.com/maps/dir/?api=1&destination=${lat},${lon}`, '_blank');
            }
        });
    }

    // Modal Close
    const modal = $('#place-modal');
    const closeBtns = $$('.modal-close, .modal-close-btn');

    closeBtns.forEach(btn => {
        on(btn, 'click', () => {
            modal.classList.add('hidden');
        });
    });

    if (modal) {
        on(modal, 'click', (e) => {
            if (e.target === modal) {
                modal.classList.add('hidden');
            }
        });
    }

    // Filter Buttons
    const buttons = $$('.category-btn');
    buttons.forEach(btn => {
        on(btn, 'click', (e) => {
            console.log('Category Clicked:', btn.dataset.category);
            buttons.forEach(b => b.classList.remove('active'));
            btn.classList.add('active');

            currentCategory = btn.dataset.category;
            renderPlaces();
        });
    });

    // Search Input
    const searchInput = $('#search-input');
    if (searchInput) {
        on(searchInput, 'input', (e) => {
            searchQuery = e.target.value;
            renderPlaces();
        });
    }
}
