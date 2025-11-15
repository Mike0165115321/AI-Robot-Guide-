// /frontend/assets/scripts/travel_mode.js 

document.addEventListener('DOMContentLoaded', () => {

    let userLatitude = null;
    let userLongitude = null;
    let navigationList = []; 
    let currentStepIndex = 0; 

    const itineraryArea = document.getElementById('itinerary-area');
    const itineraryStatus = document.getElementById('itinerary-status');
    const startNavigationBtn = document.getElementById('start-navigation-btn');
    const navButtonText = document.getElementById('nav-button-text');
    
    const mapOverlay = document.getElementById('map-overlay');
    const mapCanvas = document.getElementById('map-canvas');
    const mapHeaderTitle = document.getElementById('map-header-title');
    const closeMapBtn = document.getElementById('close-map-btn');
    
    const backToChatBtn = document.getElementById('back-to-chat-btn');
    const editTripBtn = document.getElementById('edit-trip-btn');
    
    // --- Helper Functions ---
    function alertMessage(message, isError = false) {
        const existingToast = document.getElementById('mock-toast');
        if (existingToast) existingToast.remove();

        const toast = document.createElement('div');
        toast.id = 'mock-toast';
        const bgColor = isError ? 'var(--color-highlight)' : 'var(--color-accent-teal)';
        const textColor = isError ? 'white' : 'var(--color-bg-primary)';
        
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
                alertMessage("โปรดอนุญาตการเข้าถึงตำแหน่งเพื่อนำทาง", true);
                itineraryStatus.textContent = "โปรดอนุญาตการเข้าถึงตำแหน่ง";
            }
        );
    }

    async function fetchNavigationList() {
        try {
            const response = await fetch(`${API_BASE_URL}/api/navigation_list`);
            if (!response.ok) throw new Error('Failed to fetch list');
            
            navigationList = await response.json();
            console.log("Fetched navigation list:", navigationList);
            
            renderNavigationList();
            
        } catch (error) {
            console.error('Error fetching navigation list:', error);
            alertMessage("ไม่สามารถโหลดรายการสถานที่ได้", true);
        }
    }

    function renderNavigationList() {
        itineraryArea.innerHTML = ''; 
        
        if (navigationList.length === 0) {
            itineraryArea.innerHTML = '<p class="text-color-text-secondary">ไม่พบสถานที่สำหรับนำทาง...</p>';
            return;
        }

        navigationList.forEach((item, index) => {
            const card = document.createElement('div');
            card.className = 'travel-card p-4 sm:flex sm:space-x-4';
            card.setAttribute('data-slug', item.slug);
            card.setAttribute('data-name', item.title);
            card.setAttribute('data-index', index);
            
            if (index === currentStepIndex) {
                card.classList.add('current-step');
            }

            const imageUrl = item.image_urls && item.image_urls.length > 0 
                ? item.image_urls[0] 
                : `https://placehold.co/600x400/112240/CCD6F6?text=${encodeURIComponent(item.title)}`;

            card.innerHTML = `
                <div class="sm:w-1/3 mb-3 sm:mb-0">
                    <img src="${imageUrl}" 
                         alt="${item.title}" class="w-full h-36 object-cover rounded-lg">
                </div>
                <div class="sm:w-2/3 space-y-2">
                    <span class="step-badge inline-block px-3 py-1 text-xs rounded-full">
                        ${item.topic || 'ปลายทาง'}
                    </span>
                    <h3 class="text-lg font-bold" style="color: var(--color-text-primary);">${item.title}</h3>
                </div>
            `;
            
            card.addEventListener('click', () => {
                currentStepIndex = index;
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

        navButtonText.textContent = `เริ่มนำทางไปยัง ${currentItem.title}`;

        const cards = document.querySelectorAll('.travel-card');
        cards.forEach(card => {
            card.classList.remove('current-step');
            if (card.getAttribute('data-slug') === currentItem.slug) {
                card.classList.add('current-step');
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
        mapCanvas.innerHTML = '<p class="text-color-text-secondary">กำลังโหลดแผนที่...</p>';
    }
    
    
    backToChatBtn.addEventListener('click', () => {
        window.location.href = 'chat';
    });

    startNavigationBtn.addEventListener('click', startNavigation);
    closeMapBtn.addEventListener('click', closeMap);
    
    if (editTripBtn) {
        editTripBtn.addEventListener('click', () => {
             alertMessage('ฟีเจอร์ "จัดการทริป" ยังไม่เปิดให้บริการค่ะ');
        });
    }

    getUserLocation();

});