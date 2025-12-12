// /frontend/assets/scripts/travel_mode.js
// Standalone Travel Mode Logic

document.addEventListener('DOMContentLoaded', () => {

    let userLatitude = null;
    let userLongitude = null;
    let navigationList = [];
    let currentStepIndex = 0;

    const itineraryArea = document.getElementById('itinerary-area');
    const itineraryStatus = document.getElementById('itinerary-status');
    const startNavigationBtn = document.getElementById('start-navigation-btn');
    const navButtonText = document.getElementById('nav-button-text');
    const searchInput = document.getElementById('search-input');

    const mapOverlay = document.getElementById('map-overlay');
    const mapCanvas = document.getElementById('map-canvas');
    const mapHeaderTitle = document.getElementById('map-header-title');
    const closeMapBtn = document.getElementById('close-map-btn');

    // --- Helper Functions ---
    function alertMessage(message, isError = false) {
        const existingToast = document.getElementById('mock-toast');
        if (existingToast) existingToast.remove();

        const toast = document.createElement('div');
        toast.id = 'mock-toast';
        const bgColor = isError ? 'var(--danger-color, #ef4444)' : 'var(--success-color, #22c55e)';
        const textColor = '#ffffff';

        toast.className = 'fixed top-4 right-4 px-4 py-2 rounded-lg shadow-xl z-50 transition-opacity duration-300 font-kanit font-semibold';
        toast.style.backgroundColor = bgColor;
        toast.style.color = textColor;
        toast.textContent = message;

        document.body.appendChild(toast);

        setTimeout(() => {
            toast.style.opacity = '0';
            setTimeout(() => toast.remove(), 300);
        }, 4000);
    }

    // --- Core Logic ---
    function getUserLocation() {
        if (!navigator.geolocation) {
            alertMessage("เบราว์เซอร์ของคุณไม่รองรับการระบุตำแหน่ง", true);
            itineraryStatus.textContent = "ไม่รองรับการระบุตำแหน่ง";
            return;
        }

        itineraryStatus.textContent = "กำลังขออนุญาตเข้าถึงตำแหน่ง...";
        navigator.geolocation.getCurrentPosition(
            (position) => {
                userLatitude = position.coords.latitude;
                userLongitude = position.coords.longitude;
                console.log(`User Location: ${userLatitude}, ${userLongitude}`);
                itineraryStatus.textContent = "ได้ตำแหน่งแล้ว! กำลังโหลดสถานที่...";
                startNavigationBtn.disabled = false;
                fetchNavigationList();
            },
            (error) => {
                console.error("Error getting location:", error);
                itineraryStatus.textContent = "ไม่พบตำแหน่ง (แสดงรายการทั้งหมด)";
                fetchNavigationList();
            }
        );
    }

    async function fetchNavigationList() {
        try {
            let url = `${API_BASE_URL}/api/navigation_list`;

            if (userLatitude && userLongitude) {
                url += `?lat=${userLatitude}&lon=${userLongitude}`;
                console.log("📍 Fetching locations sorted by distance...");
            } else {
                console.log("📍 Fetching locations (Default sort)...");
            }

            const response = await fetch(url);
            if (!response.ok) throw new Error('Failed to fetch list');

            navigationList = await response.json();
            console.log("Fetched navigation list:", navigationList);

            renderNavigationList();

        } catch (error) {
            console.error('Error fetching navigation list:', error);
            alertMessage("ไม่สามารถโหลดรายการสถานที่ได้", true);
        }
    }

    function renderNavigationList(filterText = '') {
        itineraryArea.innerHTML = '';

        // Filter list
        const filteredList = navigationList.filter(item => {
            if (!filterText) return true;
            const term = filterText.toLowerCase();
            return (item.title && item.title.toLowerCase().includes(term)) ||
                (item.topic && item.topic.toLowerCase().includes(term));
        });

        if (filteredList.length === 0) {
            itineraryArea.innerHTML = '<p class="text-gray-400 text-center mt-10">ไม่พบสถานที่สำหรับนำทาง...</p>';
            return;
        }

        filteredList.forEach((item, index) => {
            // Find original index in full list for click handler
            const originalIndex = navigationList.findIndex(x => x.slug === item.slug);

            const card = document.createElement('div');
            card.className = 'travel-card p-0 sm:flex sm:items-stretch overflow-hidden bg-slate-800/50 rounded-xl border border-slate-700 hover:border-teal-500/50 transition cursor-pointer mb-4';
            card.setAttribute('data-slug', item.slug);
            card.setAttribute('data-name', item.title);
            card.setAttribute('data-index', originalIndex);

            if (originalIndex === currentStepIndex) {
                card.classList.add('current-step');
                card.style.borderColor = 'var(--primary-color, #3b82f6)';
                card.style.boxShadow = '0 0 15px rgba(59, 130, 246, 0.2)';
            }

            // [Refactored Phase 4] Use shared utils
            const { primaryUrl: imageUrl, imgOnError } = getLocationImages(item, API_BASE_URL);

            let distanceBadge = '';
            if (item.distance_km !== undefined && item.distance_km !== null) {
                distanceBadge = `<span class="ml-2 text-xs font-medium px-2 py-0.5 rounded bg-blue-900/50 text-blue-200 border border-blue-700">
                        📏 ${item.distance_km} กม.</span>`;
            }

            card.innerHTML = `
                <div class="w-full sm:w-48 h-40 flex-shrink-0 overflow-hidden">
                    <img src="${imageUrl}" onerror="${imgOnError}"
                        alt="${item.title}" class="w-full h-full object-cover">
                </div>
                <div class="flex-1 p-4 space-y-2">
                    <div class="flex items-center flex-wrap gap-2">
                        <span class="inline-block px-3 py-1 text-xs rounded-full bg-teal-900/50 text-teal-200 border border-teal-700">
                            ${item.topic || 'ปลายทาง'}
                        </span>
                        ${distanceBadge}
                    </div>
                    <h3 class="text-lg font-bold text-white">${item.title}</h3>
                    <p class="text-sm text-gray-400 line-clamp-3">${item.summary || item.description || 'ไม่มีรายละเอียด'}</p>
                </div>
            `;

            card.addEventListener('click', () => {
                currentStepIndex = originalIndex;
                updateNavigationState();
            });

            itineraryArea.appendChild(card);
        });

        updateNavigationState();
    }

    function updateNavigationState() {
        if (navigationList.length === 0) return;

        const currentItem = navigationList[currentStepIndex];
        if (!currentItem) return;

        navButtonText.textContent = `เริ่มนำทางไปยัง ${currentItem.title} `;

        const cards = document.querySelectorAll('.travel-card');
        cards.forEach(card => {
            // Reset styles
            card.style.borderColor = 'rgba(51, 65, 85, 1)'; // border-slate-700
            card.style.boxShadow = 'none';

            if (card.getAttribute('data-slug') === currentItem.slug) {
                // Active styles
                card.style.borderColor = '#2dd4bf'; // teal-400
                card.style.boxShadow = '0 0 15px rgba(45, 212, 191, 0.2)';
                card.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
            }
        });
    }

    async function startNavigation() {
        if (!userLatitude || !userLongitude) {
            alertMessage("กำลังรอตำแหน่งปัจจุบัน... โปรดรอสักครู่ หรืออนุญาตตำแหน่ง", true);
            return;
        }

        const currentItem = navigationList[currentStepIndex];
        if (!currentItem) {
            alertMessage("โปรดเลือกสถานที่ก่อน", true);
            return;
        }

        const slug = currentItem.slug;
        const destName = currentItem.title;

        alertMessage(`กำลังคำนวณเส้นทางไปยัง ${destName}...`);
        startNavigationBtn.disabled = true;
        navButtonText.textContent = "กำลังโหลด...";

        try {
            const commandPayload = {
                action: "GET_DIRECTIONS",
                entity_slug: slug,
                user_lat: userLatitude,
                user_lon: userLongitude
            };

            // Use the chat API to get directions (reusing backend logic)
            const response = await fetch(`${API_BASE_URL}/api/chat/`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ query: commandPayload })
            });

            if (!response.ok) throw new Error('Failed to get directions');
            const result = await response.json();

            if (result.action === "SHOW_MAP_EMBED") {
                mapCanvas.innerHTML = '';

                const iframe = document.createElement('iframe');
                iframe.src = result.action_payload.embed_url;
                iframe.width = "100%";
                iframe.height = "100%";
                iframe.style.border = "0";
                iframe.allow = "accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share; geolocation";
                mapCanvas.appendChild(iframe);

                if (mapHeaderTitle) mapHeaderTitle.textContent = `เส้นทางไป: ${destName}`;
                showMap();
            } else {
                throw new Error(result.answer || 'Backend did not return a map.');
            }

        } catch (error) {
            console.error('Error starting navigation:', error);
            alertMessage(error.message || "ไม่สามารถคำนวณเส้นทางได้", true);
        } finally {
            startNavigationBtn.disabled = false;
            updateNavigationState();
        }
    }

    function showMap() {
        mapOverlay.style.display = 'flex';
    }

    function closeMap() {
        mapOverlay.style.display = 'none';
        mapCanvas.innerHTML = '<p class="text-gray-400">กำลังโหลดแผนที่...</p>';
    }

    // Event Listeners
    startNavigationBtn.addEventListener('click', startNavigation);
    closeMapBtn.addEventListener('click', closeMap);

    if (searchInput) {
        searchInput.addEventListener('input', (e) => {
            renderNavigationList(e.target.value);
        });
    }

    // Initialize
    getUserLocation();

});