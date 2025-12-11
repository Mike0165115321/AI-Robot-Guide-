console.log("%c ADMIN.JS LOADED - V.ULTIMATE FINAL -> V5.4.0 (Image Previews)", "color: lime; font-size: 16px; font-weight: bold;");

// --- Global Variables ---
let locationsTableBody, addLocationForm, editModal, editLocationForm, closeModalButton, fileInput, analyzeBtn, loadingSpinner, paginationContainer;
let currentPage = 1;
let itemsPerPage = 10;
let totalItems = 0;

async function fetchAndDisplayLocations() {
    if (!locationsTableBody) {
        console.error("locationsTableBody not found during fetch");
        return;
    }
    // [V5.4] Adjusted colspan from 6 to 7 to account for the new "Preview" column
    locationsTableBody.innerHTML = `<tr><td colspan="${visibleFields.length || 7}">Loading data...</td></tr>`;

    try {
        const skip = (currentPage - 1) * itemsPerPage;
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/?skip=${skip}&limit=${itemsPerPage}`);
        if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
        const data = await response.json();

        // Handle new response format (paginated) or old format (list)
        let locations = [];
        if (Array.isArray(data)) {
            locations = data; // Fallback for old API
        } else if (data.items) {
            locations = data.items;
            totalItems = data.total_count || 0;
        }

        locationsTableBody.innerHTML = '';

        if (!Array.isArray(locations) || locations.length === 0) {
            // [V5.4] Adjusted colspan - now dynamic
            locationsTableBody.innerHTML = `<tr><td colspan="${visibleFields.length || 7}">No locations found. Add one below!</td></tr>`;
            renderPaginationControls(0, currentPage, itemsPerPage);
            return;
        }

        locations.forEach(location => {
            // Check for incomplete data
            const isIncomplete = !location.summary || !location.keywords || location.keywords.length === 0;
            const warningBadge = isIncomplete
                ? '<span title="ข้อมูลไม่ครบ" style="color:#fbbf24;margin-left:4px;">⚠️</span>'
                : '';

            // Check if synced from Google Sheets
            const isFromGoogleSheets = location.metadata && location.metadata.synced_from === 'google_sheets';
            // Larger Google Sheets icon
            const sheetsBadge = isFromGoogleSheets
                ? '<span title="ข้อมูลจาก Google Sheets" style="display:inline-block;margin-left:8px;padding:2px 6px;background:rgba(34,197,94,0.15);border:1px solid rgba(34,197,94,0.4);border-radius:4px;font-size:0.75em;color:#4ade80;">📗 Sheets</span>'
                : '';

            // [Refactored Phase 3] Use shared utils
            const { primaryUrl, imgOnError } = getLocationImages(location, API_BASE_URL);

            const imagePreviewHtml = `<img src="${primaryUrl}" alt="Preview for ${location.slug}" style="width: 100px; height: 75px; object-fit: cover; border-radius: 4px; background-color: #f0f0f0;" onerror="${imgOnError}">`;

            // Truncate summary and format keywords
            const shortSummary = location.summary
                ? (location.summary.length > 60 ? location.summary.substring(0, 60) + '...' : location.summary)
                : '<span style="color:#888;">-</span>';

            const keywordsList = (location.keywords && Array.isArray(location.keywords) && location.keywords.length > 0)
                ? location.keywords.slice(0, 3).join(', ') + (location.keywords.length > 3 ? '...' : '')
                : '<span style="color:#888;">-</span>';

            const row = document.createElement('tr');

            // Apply subtle styling based on source
            if (isFromGoogleSheets) {
                // Subtle green indicator - soft border only
                row.style.cssText = `
                    border-left: 3px solid rgba(34, 197, 94, 0.6);
                    background: rgba(34, 197, 94, 0.03);
                `;
            }
            if (isIncomplete) {
                row.style.background = 'rgba(251,191,36,0.1)';
            }

            // Dynamic column rendering based on visibleFields
            let rowHtml = '';
            visibleFields.forEach(field => {
                let cellContent = '';
                let cellStyle = '';

                switch (field) {
                    case 'preview':
                        cellContent = imagePreviewHtml;
                        break;
                    case 'title':
                        cellContent = `${location.title || 'N/A'}${sheetsBadge}${warningBadge}`;
                        break;
                    case 'category':
                        cellContent = location.category || 'N/A';
                        break;
                    case 'topic':
                        cellContent = location.topic || 'N/A';
                        break;
                    case 'summary':
                        cellStyle = 'max-width:200px;overflow:hidden;text-overflow:ellipsis;';
                        cellContent = shortSummary;
                        break;
                    case 'keywords':
                        cellStyle = 'max-width:150px;overflow:hidden;text-overflow:ellipsis;';
                        cellContent = keywordsList;
                        break;
                    case 'actions':
                        cellContent = `
                            <button class="btn btn-edit" data-slug="${location.slug}">แก้ไข</button>
                            <button class="btn btn-delete" data-slug="${location.slug}">ลบ</button>
                        `;
                        break;
                    case 'slug':
                        cellContent = `<code style="font-size:0.8em;color:#888;">${location.slug || '-'}</code>`;
                        break;
                    case 'doc_type':
                        cellContent = location.doc_type || '-';
                        break;
                    case 'id':
                        cellContent = location.id || '-';
                        break;
                    default:
                        // For any other field, try to display it
                        const value = location[field];
                        if (typeof value === 'object' && value !== null) {
                            cellContent = '<span style="color:#888;">[Object]</span>';
                        } else if (Array.isArray(value)) {
                            cellContent = `<span style="color:#888;">[${value.length} items]</span>`;
                        } else {
                            cellContent = value || '-';
                        }
                }

                rowHtml += `<td style="${cellStyle}">${cellContent}</td>`;
            });

            row.innerHTML = rowHtml;
            locationsTableBody.appendChild(row);
        });

        renderPaginationControls(totalItems, currentPage, itemsPerPage);

    } catch (error) {
        console.error('Fetch error:', error);
        // [V5.4] Adjusted colspan
        locationsTableBody.innerHTML = `<tr><td colspan="${visibleFields.length || 7}">Failed to load data. Please check connection.</td></tr>`;
    }
}

function renderPaginationControls(total, current, limit) {
    if (!paginationContainer) return;
    paginationContainer.innerHTML = '';

    if (total === 0) return;

    const totalPages = Math.ceil(total / limit);

    // First Button
    const firstBtn = document.createElement('button');
    firstBtn.innerText = 'First';
    firstBtn.className = 'pagination-btn';
    firstBtn.disabled = current === 1;
    firstBtn.onclick = () => { currentPage = 1; fetchAndDisplayLocations(); };
    paginationContainer.appendChild(firstBtn);

    // Prev Button
    const prevBtn = document.createElement('button');
    prevBtn.innerText = 'Prev';
    prevBtn.className = 'pagination-btn';
    prevBtn.disabled = current === 1;
    prevBtn.onclick = () => { if (current > 1) { currentPage--; fetchAndDisplayLocations(); } };
    paginationContainer.appendChild(prevBtn);

    // Page Info
    const infoSpan = document.createElement('span');
    infoSpan.className = 'pagination-info';
    infoSpan.innerText = ` Page ${current} of ${totalPages} (Total: ${total}) `;
    paginationContainer.appendChild(infoSpan);

    // Next Button
    const nextBtn = document.createElement('button');
    nextBtn.innerText = 'Next';
    nextBtn.className = 'pagination-btn';
    nextBtn.disabled = current === totalPages;
    nextBtn.onclick = () => { if (current < totalPages) { currentPage++; fetchAndDisplayLocations(); } };
    paginationContainer.appendChild(nextBtn);

    // Last Button
    const lastBtn = document.createElement('button');
    lastBtn.innerText = 'Last';
    lastBtn.className = 'pagination-btn';
    lastBtn.disabled = current === totalPages;
    lastBtn.onclick = () => { currentPage = totalPages; fetchAndDisplayLocations(); };
    paginationContainer.appendChild(lastBtn);
}

async function deleteLocation(slug) {
    if (!slug) {
        alert('Error: Invalid slug provided for deletion.');
        return;
    }
    if (!confirm(`คุณแน่ใจหรือไม่ว่าจะลบรายการ "${slug}"?`)) return;

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/${slug}`, { method: 'DELETE' });
        if (response.status === 204) {
            alert(`ลบข้อมูล "${slug}" เรียบร้อยแล้ว!`);
            fetchAndDisplayLocations(); // Refresh table
        } else {
            let errorDetail = response.statusText;
            try {
                const errorData = await response.json();
                errorDetail = errorData.detail || errorDetail;
            } catch (e) { /* Ignore parsing error if no JSON body */ }
            alert(`ลบข้อมูลไม่สำเร็จ: ${errorDetail}`);
        }
    } catch (error) {
        console.error('Delete error:', error);
        alert('เกิดข้อผิดพลาดขณะทำการลบ (ดู Console)');
    }
}

async function openEditModal(slug) {
    if (!slug) {
        alert('Error: Invalid slug provided for editing.');
        return;
    }
    console.log(`Opening edit modal for slug: ${slug}`);

    if (!editModal || !editLocationForm) {
        console.error("Edit modal or form not found.");
        alert("ข้อผิดพลาด: ไม่พบองค์ประกอบฟอร์มแก้ไข");
        return;
    }

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/${slug}`);
        if (!response.ok) throw new Error(`Failed to fetch location details for "${slug}". Status: ${response.status}`);
        const location = await response.json();

        // --- [Refactored Phase 3] Use shared utils for Modal Preview ---
        const imagePreviewEl = document.getElementById('edit-form-image-preview');
        const noImageTextEl = document.getElementById('edit-form-no-image');

        // Reset state
        if (imagePreviewEl) {
            imagePreviewEl.style.display = 'none';
            imagePreviewEl.onerror = null;
            delete imagePreviewEl.dataset.triedSlug;
        }
        if (noImageTextEl) noImageTextEl.style.display = 'none';

        const { primaryUrl, secondaryUrl, placeholder } = getLocationImages(location, API_BASE_URL);

        // Logic: if primaryUrl IS the placeholder and no secondary, it means no image found at all.
        const isPlaceholder = (primaryUrl === placeholder || primaryUrl.startsWith('data:image/svg+xml'));

        if (isPlaceholder) {
            if (noImageTextEl) noImageTextEl.style.display = 'block';
        } else {
            if (imagePreviewEl) {
                imagePreviewEl.src = primaryUrl;
                imagePreviewEl.style.display = 'block';

                imagePreviewEl.onerror = function () {
                    if (secondaryUrl && !this.dataset.triedSlug) {
                        this.dataset.triedSlug = "true";
                        this.src = secondaryUrl;
                    } else {
                        // Failed both
                        this.style.display = 'none';
                        if (noImageTextEl) noImageTextEl.style.display = 'block';
                    }
                };
            }
        }
        // --- [END V5.5] ---

        // Populate hidden fields first
        document.getElementById('edit-form-mongo-id').value = location.mongo_id || '';
        document.getElementById('edit-form-slug').value = location.slug;

        // Populate visible fields
        document.getElementById('display-slug').value = location.slug;
        document.getElementById('edit-form-title').value = location.title || '';
        document.getElementById('edit-form-summary').value = location.summary || '';
        document.getElementById('edit-form-category').value = location.category || '';
        document.getElementById('edit-form-topic').value = location.topic || '';
        document.getElementById('edit-form-keywords').value = (location.keywords && Array.isArray(location.keywords))
            ? location.keywords.join(', ')
            : '';
        document.getElementById('edit-form-details').value = (location.details && Array.isArray(location.details))
            ? JSON.stringify(location.details, null, 2)
            : '[]';

        const imagePrefix = (location.metadata && location.metadata.image_prefix) ? location.metadata.image_prefix : '';
        document.getElementById('edit-form-image-prefix').value = imagePrefix;
        document.getElementById('edit-form-image-file').value = '';

        editModal.style.display = 'block';
    } catch (error) {
        console.error('Error opening edit modal:', error);
        alert(`Could not open the edit form for "${slug}". Reason: ${error.message}`);
    }
}


async function handleAddLocationSubmit(event) {
    event.preventDefault();
    if (!addLocationForm) return;

    const addBtn = addLocationForm.querySelector('button[type="submit"]');
    addBtn.disabled = true; addBtn.textContent = 'Processing...';

    const slug = document.getElementById('form-slug').value.trim();
    if (!slug) {
        alert("กรุณากรอก Slug (Key ที่ไม่ซ้ำกัน)");
        addBtn.disabled = false; addBtn.textContent = 'Add Location'; return;
    }

    if (!/^[a-z0-9_-]{3,}$/.test(slug)) {
        alert("Slug ต้องมีอย่างน้อย 3 ตัวอักษร และประกอบด้วย a-z, 0-9, _, - เท่านั้น");
        addBtn.disabled = false; addBtn.textContent = 'Add Location'; return;
    }

    const imagePrefixInput = document.getElementById('form-image-prefix').value.trim();
    const imageFile = document.getElementById('form-image-file').files[0];
    let uploadedImagePrefix = null;

    if (imageFile && !imagePrefixInput) {
        alert("กรุณากรอก Image Prefix (ต้องตรงกับ Slug) ก่อนอัปโหลดรูปภาพ");
        addBtn.disabled = false; addBtn.textContent = 'Add Location'; return;
    }
    if (imageFile && imagePrefixInput !== slug) {
        alert("Image Prefix ต้องตรงกับ Slug ที่กรอก");
        addBtn.disabled = false; addBtn.textContent = 'Add Location'; return;
    }

    const finalImagePrefixForMeta = imagePrefixInput === slug ? slug : null;


    if (imageFile && slug) {
        const imageFormData = new FormData();
        imageFormData.append('file', imageFile);
        try {
            const imageResponseQuery = await fetch(`${API_BASE_URL}/api/admin/locations/upload-image/?image_prefix=${slug}`, {
                method: 'POST',
                body: imageFormData
            });

            if (!imageResponseQuery.ok) {
                const errorText = await imageResponseQuery.text();
                try {
                    const errorData = JSON.parse(errorText);
                    throw new Error(errorData.detail || 'Image upload failed');
                } catch (parseError) {
                    throw new Error(errorText || 'Image upload failed');
                }
            }
            const imageData = await imageResponseQuery.json();
            uploadedImagePrefix = imageData.image_prefix;
            alert(`อัปโหลดรูปภาพสำเร็จ! บันทึกเป็น: ${imageData.saved_as}`);
        } catch (error) {
            console.error('Image upload error:', error);
            alert(`เกิดข้อผิดพลาดในการอัปโหลดรูปภาพ: ${error.message}`);
            addBtn.disabled = false; addBtn.textContent = 'Add Location'; return;
        }
    }

    const keywordsInput = document.getElementById('form-keywords').value || '';
    const keywordsArray = keywordsInput.split(',')
        .map(k => k.trim())
        .filter(k => k);

    const detailsInput = document.getElementById('form-details').value || '[]';
    let detailsArray = [];
    try {
        detailsArray = JSON.parse(detailsInput);
        if (!Array.isArray(detailsArray)) detailsArray = [];
    } catch (e) {
        console.warn("Could not parse Details JSON string. Defaulting to empty array.", e);
        detailsArray = [];
    }


    const newLocationData = {
        slug: slug,
        title: document.getElementById('form-title').value,
        summary: document.getElementById('form-summary').value,
        category: document.getElementById('form-category').value,
        topic: document.getElementById('form-topic').value,
        keywords: keywordsArray,
        details: detailsArray,
        metadata: uploadedImagePrefix ? { image_prefix: uploadedImagePrefix } : (finalImagePrefixForMeta ? { image_prefix: finalImagePrefixForMeta } : null)
    };

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(newLocationData)
        });
        if (response.status === 201 || response.ok) {
            alert('เพิ่มข้อมูลสถานที่เรียบร้อยแล้ว!');
            addLocationForm.reset();
            fetchAndDisplayLocations(); // Refresh table
        } else {
            let errorDetail = response.statusText;
            try {
                const errorData = await response.json();
                errorDetail = errorData.detail || errorDetail;
                if (response.status === 400 && (errorDetail.includes("Slug") && errorDetail.includes("exists"))) {
                    errorDetail = `Slug '${slug}' นี้มีอยู่แล้ว กรุณาใช้ Slug อื่น`;
                }
            } catch (e) { /* Ignore parsing error if no JSON body */ }
            alert(`เพิ่มข้อมูลไม่สำเร็จ: ${errorDetail}`);
        }
    } catch (error) {
        console.error('Add location error:', error);
        alert('เกิดข้อผิดพลาดขณะเพิ่มข้อมูลสถานที่ (ดู Console)');
    } finally {
        addBtn.disabled = false; addBtn.textContent = 'Add Location';
    }
}

async function handleEditFormSubmit(event) {
    event.preventDefault();
    if (!editLocationForm) return;

    const saveBtn = editLocationForm.querySelector('button[type="submit"]');
    saveBtn.disabled = true; saveBtn.textContent = 'Saving...';

    const slug = document.getElementById('edit-form-slug').value;
    if (!slug) {
        alert("ข้อผิดพลาด: ไม่พบ Slug สำหรับการแก้ไข");
        saveBtn.disabled = false; saveBtn.textContent = 'Save Changes'; return;
    }

    const imagePrefixInput = document.getElementById('edit-form-image-prefix').value.trim();
    const imageFile = document.getElementById('edit-form-image-file').files[0];
    let finalImagePrefix = imagePrefixInput;

    if (imagePrefixInput && imagePrefixInput !== slug) {
        alert("Image Prefix ต้องตรงกับ Slug (หรือไม่ต้องกรอกเพื่อลบ)");
        saveBtn.disabled = false; saveBtn.textContent = 'Save Changes'; return;
    }
    if (imageFile && imagePrefixInput !== slug) {
        alert("Image Prefix ต้องตรงกับ Slug เมื่ออัปโหลดไฟล์ใหม่");
        saveBtn.disabled = false; saveBtn.textContent = 'Save Changes'; return;
    }

    if (imageFile && slug) {
        const imageFormData = new FormData();
        imageFormData.append('file', imageFile);
        try {
            const imageResponse = await fetch(`${API_BASE_URL}/api/admin/locations/upload-image/?image_prefix=${slug}`, {
                method: 'POST',
                body: imageFormData
            });

            if (!imageResponse.ok) {
                const errorText = await imageResponse.text();
                try {
                    const errorData = JSON.parse(errorText);
                    throw new Error(errorData.detail || 'Image upload failed');
                } catch (parseError) {
                    throw new Error(errorText || 'Image upload failed');
                }
            }
            const imageData = await imageResponse.json();
            finalImagePrefix = imageData.image_prefix;
            alert(`อัปโหลดรูปภาพใหม่สำเร็จ! บันทึกเป็น: ${imageData.saved_as}`);

            document.getElementById('edit-form-image-prefix').value = finalImagePrefix;
            document.getElementById('edit-form-image-file').value = '';
        } catch (error) {
            console.error('Image upload error during edit:', error);
            alert(`เกิดข้อผิดพลาดในการอัปโหลดรูปภาพใหม่: ${error.message}`);
            saveBtn.disabled = false; saveBtn.textContent = 'Save Changes'; return;
        }
    }

    const keywordsInput = document.getElementById('edit-form-keywords').value || '';
    const keywordsArray = keywordsInput.split(',')
        .map(k => k.trim())
        .filter(k => k);

    const detailsInput = document.getElementById('edit-form-details').value || '[]';
    let detailsArray = [];
    try {
        detailsArray = JSON.parse(detailsInput);
        if (!Array.isArray(detailsArray)) detailsArray = [];
    } catch (e) {
        console.warn("Could not parse Details JSON string. Defaulting to empty array.", e);
        detailsArray = [];
    }

    const updatedData = {
        slug: slug,
        title: document.getElementById('edit-form-title').value,
        summary: document.getElementById('edit-form-summary').value,
        category: document.getElementById('edit-form-category').value,
        topic: document.getElementById('edit-form-topic').value,
        keywords: keywordsArray,
        details: detailsArray,
        metadata: finalImagePrefix ? { image_prefix: finalImagePrefix } : null
    };

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/${slug}`, {
            method: 'PUT',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(updatedData)
        });
        if (response.ok) {
            alert(`อัปเดตข้อมูล "${slug}" เรียบร้อยแล้ว!`);
            if (editModal) editModal.style.display = 'none';
            fetchAndDisplayLocations(); // Refresh table
        } else {
            let errorDetail = response.statusText;
            try {
                const errorData = await response.json();
                errorDetail = errorData.detail || errorDetail;
            } catch (e) { /* Ignore parsing error */ }
            alert(`อัปเดตข้อมูลไม่สำเร็จ: ${errorDetail}`);
        }
    } catch (error) {
        console.error('Update location error:', error);
        alert('เกิดข้อผิดพลาดขณะอัปเดตข้อมูล (ดู Console)');
    } finally {
        saveBtn.disabled = false; saveBtn.textContent = 'Save Changes';
    }
}

async function handleAnalyzeDocument() {
    if (!fileInput || !fileInput.files || fileInput.files.length === 0) {
        alert('Please select a document file first.');
        return;
    }
    const file = fileInput.files[0];
    const formData = new FormData();
    formData.append('file', file);

    if (!analyzeBtn || !loadingSpinner) {
        console.error("Analyze button or loading spinner not found.");
        return;
    }

    analyzeBtn.disabled = true;
    loadingSpinner.style.display = 'inline-block';
    analyzeBtn.textContent = 'Analyzing...';

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/analyze-document`, {
            method: 'POST',
            body: formData,
        });

        if (!response.ok) {
            let errorDetail = `HTTP error! status: ${response.status}`;
            try {
                const errorData = await response.json();
                errorDetail = errorData.detail || errorDetail;
            } catch (e) { /* Ignore parsing error */ }
            throw new Error(errorDetail);
        }

        const data = await response.json();

        document.getElementById('form-title').value = data.title || '';
        document.getElementById('form-summary').value = data.summary || '';
        document.getElementById('form-category').value = data.category || '';
        document.getElementById('form-topic').value = data.topic || '';
        document.getElementById('form-keywords').value = (data.keywords && Array.isArray(data.keywords))
            ? data.keywords.join(', ')
            : '';
        document.getElementById('form-details').value = (data.details && Array.isArray(data.details))
            ? JSON.stringify(data.details, null, 2)
            : '[]';

        if (data.slug) {
            document.getElementById('form-slug').value = data.slug;
        } else {
            const titleForSlug = data.title || `item_${Date.now()}`;
            let generatedSlug = titleForSlug.toLowerCase().trim()
                .replace(/[\s\(\)\[\]{}]+/g, '-')
                .replace(/[^a-z0-9-]/g, '')
                .replace(/[-]+/g, '-')
                .replace(/^-+|-+$/g, '')
                .substring(0, 50);
            if (!generatedSlug) generatedSlug = `item_${Date.now()}`;
            document.getElementById('form-slug').value = generatedSlug;
        }

        alert('Document analyzed successfully! Form fields have been populated.');

    } catch (error) {
        console.error('Document analysis error:', error);
        alert(`Failed to analyze document: ${error.message}`);
    } finally {
        analyzeBtn.disabled = false;
        loadingSpinner.style.display = 'none';
        analyzeBtn.textContent = 'Analyze Document';
        if (fileInput) fileInput.value = '';
    }
}

// ==========================================================
//  EVENT LISTENERS
// ==========================================================

document.addEventListener('DOMContentLoaded', () => {
    locationsTableBody = document.querySelector('#locations-table tbody');
    addLocationForm = document.getElementById('add-location-form');
    editModal = document.getElementById('edit-modal');
    editLocationForm = document.getElementById('edit-location-form');
    closeModalButton = document.querySelector('#edit-modal .close-button');
    fileInput = document.getElementById('file-input');
    analyzeBtn = document.getElementById('analyze-btn');
    analyzeBtn = document.getElementById('analyze-btn');
    loadingSpinner = document.getElementById('loading-spinner');
    paginationContainer = document.getElementById('pagination-container');

    if (locationsTableBody) {
        fetchAndDisplayLocations();
    } else {
        console.error("Critical Error: locationsTableBody not found.");
    }

    if (addLocationForm) {
        addLocationForm.addEventListener('submit', handleAddLocationSubmit);
    } else {
        console.warn("Add location form (#add-location-form) not found.");
    }

    if (editLocationForm) {
        editLocationForm.addEventListener('submit', handleEditFormSubmit);
    } else {
        console.warn("Edit location form (#edit-location-form) not found.");
    }

    if (closeModalButton && editModal) {
        closeModalButton.addEventListener('click', () => {
            editModal.style.display = 'none';
        });
    }

    if (analyzeBtn) {
        analyzeBtn.addEventListener('click', handleAnalyzeDocument);
    } else {
        console.warn("Analyze document button (#analyze-btn) not found.");
    }

    if (locationsTableBody) {
        locationsTableBody.addEventListener('click', (event) => {
            const targetButton = event.target.closest('button');
            if (!targetButton) return;

            const slug = targetButton.dataset.slug;

            if (targetButton.classList.contains('btn-edit')) {
                openEditModal(slug);
            } else if (targetButton.classList.contains('btn-delete')) {
                deleteLocation(slug);
            }
        });
    }

    window.onclick = function (event) {
        if (editModal && event.target == editModal) {
            editModal.style.display = "none";
        }
    }

    // Check Google Sheets status on load
    checkSheetsStatus();
});

// ==========================================================
//  GOOGLE SHEETS SYNC FUNCTIONS
// ==========================================================

const SHEETS_STORAGE_KEY = 'nongnan_sheets_config';

// Load saved config from localStorage
function loadSheetsConfig() {
    try {
        const saved = localStorage.getItem(SHEETS_STORAGE_KEY);
        return saved ? JSON.parse(saved) : null;
    } catch (e) {
        console.error('Failed to load sheets config:', e);
        return null;
    }
}

// Save config to localStorage
function saveSheetsConfig(config) {
    try {
        localStorage.setItem(SHEETS_STORAGE_KEY, JSON.stringify(config));
    } catch (e) {
        console.error('Failed to save sheets config:', e);
    }
}

// Clear saved config
function clearSheetsConfig() {
    localStorage.removeItem(SHEETS_STORAGE_KEY);
}

async function checkSheetsStatus() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/sheets/status`);
        if (!response.ok) return;

        const data = await response.json();

        // If connected, update UI
        if (data.connection && data.connection.connected) {
            updateSheetsUI(data.connection);

            // Fill URL input with saved URL
            const savedConfig = loadSheetsConfig();
            if (savedConfig && savedConfig.sheet_url) {
                document.getElementById('sheets-url-input').value = savedConfig.sheet_url;
            }
        } else {
            // Not connected - try to auto-reconnect from saved config
            const savedConfig = loadSheetsConfig();
            if (savedConfig && savedConfig.sheet_url) {
                console.log('🔄 Auto-reconnecting to saved Google Sheet...');
                await autoReconnectSheet(savedConfig.sheet_url);
            }
        }
    } catch (error) {
        console.log('Sheets status check failed:', error);
    }
}

// Auto-reconnect without alert
async function autoReconnectSheet(sheetUrl) {
    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/sheets/connect`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ sheet_url: sheetUrl })
        });

        const data = await response.json();

        if (response.ok && data.success) {
            console.log('✅ Auto-reconnected to:', data.status.sheet_title);
            updateSheetsUI(data.status);
            document.getElementById('sheets-url-input').value = sheetUrl;
        } else {
            console.log('❌ Auto-reconnect failed, clearing saved config');
            clearSheetsConfig();
        }
    } catch (error) {
        console.log('Auto-reconnect error:', error);
    }
}

function updateSheetsUI(status) {
    const statusDot = document.getElementById('sheets-status-dot');
    const statusText = document.getElementById('sheets-status-text');
    const sheetsInfo = document.getElementById('sheets-info');
    const sheetsTitle = document.getElementById('sheets-title');
    const sheetsLastSync = document.getElementById('sheets-last-sync');
    const syncBtn = document.getElementById('sheets-sync-btn');
    const disconnectBtn = document.getElementById('sheets-disconnect-btn');
    const urlInput = document.getElementById('sheets-url-input');
    const connectBtn = document.getElementById('sheets-connect-btn');
    const modeSelection = document.getElementById('sheets-mode-selection');

    if (status && status.connected) {
        // Connected state
        statusDot.style.background = '#22c55e';
        statusText.textContent = '✅ เชื่อมต่อแล้ว';
        sheetsTitle.textContent = status.sheet_title || status.sheet_id;
        sheetsLastSync.textContent = status.last_sync || 'ยังไม่ได้ sync';
        sheetsInfo.style.display = 'block';

        // Show sync/disconnect buttons
        syncBtn.style.display = 'inline-block';
        syncBtn.disabled = false;
        disconnectBtn.style.display = 'inline-block';
        disconnectBtn.disabled = false;

        // Disable URL input
        urlInput.disabled = true;
        connectBtn.disabled = true;

        // Hide mode selection when connected
        if (modeSelection) modeSelection.style.display = 'none';
    } else {
        // Disconnected state
        statusDot.style.background = '#ef4444';
        statusText.textContent = 'ยังไม่ได้เชื่อมต่อ';
        sheetsInfo.style.display = 'none';

        // Hide sync/disconnect buttons when not connected
        syncBtn.style.display = 'none';
        disconnectBtn.style.display = 'none';

        // Enable URL input
        urlInput.disabled = false;
        connectBtn.disabled = false;

        // Also hide mode selection when disconnected
        if (modeSelection) modeSelection.style.display = 'none';
    }
}

async function connectGoogleSheet() {
    const urlInput = document.getElementById('sheets-url-input');
    const url = urlInput.value.trim();

    if (!url) {
        alert('กรุณาวาง Google Sheets URL');
        return;
    }

    const connectBtn = document.getElementById('sheets-connect-btn');
    connectBtn.disabled = true;
    connectBtn.textContent = '⏳ กำลังเชื่อมต่อ...';

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/sheets/connect`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ sheet_url: url })
        });

        const data = await response.json();

        if (response.ok && data.success) {
            // Save config to localStorage for persistence
            saveSheetsConfig({ sheet_url: url, sheet_id: data.status.sheet_id });

            alert('✅ ' + data.message);
            updateSheetsUI(data.status);

            // Show mode selection after FIRST connect (manual connect)
            showModeSelection();
        } else {
            alert('❌ ' + (data.detail || 'เชื่อมต่อไม่สำเร็จ'));
        }
    } catch (error) {
        alert('❌ เกิดข้อผิดพลาด: ' + error.message);
    } finally {
        connectBtn.disabled = false;
        connectBtn.textContent = '🔌 เชื่อมต่อ';
    }
}

async function syncGoogleSheet() {
    const syncBtn = document.getElementById('sheets-sync-btn');
    const resultDiv = document.getElementById('sheets-sync-result');

    syncBtn.disabled = true;
    syncBtn.textContent = '⏳ กำลัง Sync...';
    resultDiv.style.display = 'none';

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/sheets/sync-now`, {
            method: 'POST'
        });

        const data = await response.json();

        resultDiv.style.display = 'block';

        if (data.success) {
            const r = data.result;
            resultDiv.innerHTML = `
                <div style="color: #22c55e; font-weight: bold; margin-bottom: 0.5rem;">✅ Sync สำเร็จ!</div>
                <div>➕ สร้างใหม่: ${r.created} รายการ</div>
                <div>📝 อัพเดท: ${r.updated} รายการ</div>
                <div>🗑️ ลบ: ${r.deleted} รายการ</div>
                <div style="font-size: 0.8rem; color: #888; margin-top: 0.5rem;">เวลา: ${r.timestamp}</div>
            `;
            resultDiv.style.borderColor = 'rgba(34,197,94,0.5)';

            // Refresh table
            fetchAndDisplayLocations();

            // Update last sync time
            document.getElementById('sheets-last-sync').textContent = r.timestamp;
        } else {
            resultDiv.innerHTML = `
                <div style="color: #ef4444; font-weight: bold;">❌ Sync ไม่สำเร็จ</div>
                <div>${data.result?.errors?.join(', ') || 'Unknown error'}</div>
            `;
            resultDiv.style.borderColor = 'rgba(239,68,68,0.5)';
        }
    } catch (error) {
        resultDiv.style.display = 'block';
        resultDiv.innerHTML = `<div style="color: #ef4444;">❌ Error: ${error.message}</div>`;
    } finally {
        syncBtn.disabled = false;
        syncBtn.textContent = '🔄 Sync ตอนนี้';
    }
}

async function disconnectGoogleSheet() {
    if (!confirm('ยืนยันยกเลิกการเชื่อมต่อ Google Sheet?')) return;

    stopAutoPolling();

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/sheets/disconnect`, {
            method: 'DELETE'
        });

        if (response.ok) {
            // Clear saved config from localStorage
            clearSheetsConfig();

            alert('✅ ยกเลิกการเชื่อมต่อแล้ว');
            updateSheetsUI({ connected: false });
            document.getElementById('sheets-url-input').value = '';
            document.getElementById('sheets-sync-result').style.display = 'none';
            document.getElementById('sheets-mode-selection').style.display = 'none';
        }
    } catch (error) {
        alert('❌ เกิดข้อผิดพลาด: ' + error.message);
    }
}

// ==========================================================
//  SYNC MODE SELECTION & AUTO POLLING
// ==========================================================

let selectedSyncMode = null;
let pollingInterval = null;
const POLLING_INTERVAL_MS = 5 * 60 * 1000; // 5 minutes

function showModeSelection() {
    document.getElementById('sheets-mode-selection').style.display = 'block';
}

function selectSyncMode(mode) {
    selectedSyncMode = mode;

    // Update card styles
    const pollingCard = document.getElementById('mode-polling-card');
    const webhookCard = document.getElementById('mode-webhook-card');
    const instructionsDiv = document.getElementById('mode-instructions');
    const startPollingBtn = document.getElementById('sheets-start-polling-btn');
    const stopPollingBtn = document.getElementById('sheets-stop-polling-btn');
    const sheetsMode = document.getElementById('sheets-mode');

    // Reset styles
    pollingCard.style.borderWidth = '2px';
    webhookCard.style.borderWidth = '2px';
    pollingCard.style.transform = 'scale(1)';
    webhookCard.style.transform = 'scale(1)';

    if (mode === 'polling') {
        pollingCard.style.borderWidth = '3px';
        pollingCard.style.transform = 'scale(1.02)';
        sheetsMode.textContent = '🔄 Auto Polling (ทุก 5 นาที)';

        startPollingBtn.style.display = 'inline-block';
        startPollingBtn.disabled = false;
        stopPollingBtn.style.display = 'none';

        instructionsDiv.style.display = 'block';
        instructionsDiv.innerHTML = `
            <div style="color: #22c55e; font-weight: bold; margin-bottom: 0.5rem;">✅ โหมด Auto Polling พร้อมใช้งาน!</div>
            <p style="font-size: 0.9rem; color: var(--text-light);">
                กด <strong>"▶️ เริ่ม Auto Sync"</strong> เพื่อเริ่มตรวจสอบ Google Sheet ทุก 5 นาที<br>
                ระบบจะตรวจสอบการเปลี่ยนแปลงและ sync อัตโนมัติ
            </p>
        `;
    } else if (mode === 'webhook') {
        webhookCard.style.borderWidth = '3px';
        webhookCard.style.transform = 'scale(1.02)';
        sheetsMode.textContent = '⚡ Webhook (Real-time)';

        startPollingBtn.style.display = 'none';
        stopPollingBtn.style.display = 'none';

        instructionsDiv.style.display = 'block';
        instructionsDiv.innerHTML = `
            <div style="color: #fbbf24; font-weight: bold; margin-bottom: 0.5rem;">⚡ วิธี Setup Webhook (Real-time)</div>
            <div style="font-size: 0.85rem; color: var(--text-light);">
                <p><strong>ขั้นตอน 1:</strong> เปิด Google Sheet → Extensions → Apps Script</p>
                <p><strong>ขั้นตอน 2:</strong> วาง code นี้:</p>
                <pre style="background: rgba(0,0,0,0.3); padding: 0.5rem; border-radius: 4px; overflow-x: auto; font-size: 0.75rem;">
function onEdit(e) {
  const sheet = e.source.getActiveSheet();
  const row = e.range.getRow();
  if (row === 1) return; // Skip header
  
  const data = {};
  const headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];
  const values = sheet.getRange(row, 1, 1, sheet.getLastColumn()).getValues()[0];
  
  headers.forEach((h, i) => { data[h] = values[i]; });
  
  const webhook = "YOUR_SERVER_URL/api/admin/sheets/webhook";
  UrlFetchApp.fetch(webhook, {
    method: "POST",
    contentType: "application/json",
    payload: JSON.stringify({ event: "edit", row_data: data })
  });
}</pre>
                <p><strong>ขั้นตอน 3:</strong> แทนที่ <code>YOUR_SERVER_URL</code> ด้วย URL ของ server (ต้อง deploy หรือใช้ ngrok)</p>
                <p><strong>ขั้นตอน 4:</strong> ตั้ง Trigger: Edit → Current project's triggers → Add trigger → onEdit</p>
            </div>
        `;
    }
}

function startAutoPolling() {
    if (pollingInterval) {
        console.log('Polling already running');
        return;
    }

    const startBtn = document.getElementById('sheets-start-polling-btn');
    const stopBtn = document.getElementById('sheets-stop-polling-btn');
    const sheetsMode = document.getElementById('sheets-mode');

    startBtn.style.display = 'none';
    stopBtn.style.display = 'inline-block';
    stopBtn.disabled = false;
    sheetsMode.textContent = '🔄 Auto Polling (กำลังทำงาน...)';

    // Sync immediately
    syncGoogleSheet();

    // Start polling
    pollingInterval = setInterval(() => {
        console.log('🔄 Auto polling: syncing Google Sheet...');
        syncGoogleSheet();
    }, POLLING_INTERVAL_MS);

    console.log(`✅ Auto polling started (every ${POLLING_INTERVAL_MS / 1000 / 60} minutes)`);
}

function stopAutoPolling() {
    if (pollingInterval) {
        clearInterval(pollingInterval);
        pollingInterval = null;

        const startBtn = document.getElementById('sheets-start-polling-btn');
        const stopBtn = document.getElementById('sheets-stop-polling-btn');
        const sheetsMode = document.getElementById('sheets-mode');

        if (startBtn) startBtn.style.display = 'inline-block';
        if (stopBtn) stopBtn.style.display = 'none';
        if (sheetsMode) sheetsMode.textContent = '🔄 Auto Polling (หยุดแล้ว)';

        console.log('⏹️ Auto polling stopped');
    }
}

// ==========================================================
//  FIELD VISIBILITY SETTINGS
// ==========================================================

const FIELD_SETTINGS_KEY = 'nongnan_field_settings';
let availableFields = [];
let visibleFields = [];

// Default fields to show
const DEFAULT_VISIBLE_FIELDS = ['preview', 'title', 'category', 'topic', 'summary', 'keywords', 'actions'];
// Fields that cannot be hidden
const REQUIRED_FIELDS = ['title', 'actions'];
// Field display names (Thai)
const FIELD_DISPLAY_NAMES = {
    'preview': '🖼️ รูปภาพ',
    'title': '📍 ชื่อ (Title)',
    'category': '📂 หมวดหมู่',
    'topic': '📝 หัวข้อ',
    'summary': '📄 รายละเอียดย่อ',
    'keywords': '🔖 Keywords',
    'actions': '⚙️ Actions',
    'slug': '🔗 Slug',
    'doc_type': '📋 ประเภทเอกสาร',
    'id': '🆔 ID',
    'metadata': '📊 Metadata',
    'details': '📒 รายละเอียด',
    'location_data': '📍 พิกัด',
    'sources': '📚 แหล่งอ้างอิง',
    'related_info': '🔗 ข้อมูลเกี่ยวข้อง'
};

function loadFieldSettings() {
    try {
        const saved = localStorage.getItem(FIELD_SETTINGS_KEY);
        if (saved) {
            visibleFields = JSON.parse(saved);
        } else {
            visibleFields = [...DEFAULT_VISIBLE_FIELDS];
        }
    } catch (e) {
        console.error('Error loading field settings:', e);
        visibleFields = [...DEFAULT_VISIBLE_FIELDS];
    }
    return visibleFields;
}

function saveFieldSettings() {
    try {
        localStorage.setItem(FIELD_SETTINGS_KEY, JSON.stringify(visibleFields));
        console.log('✅ Field settings saved:', visibleFields);
    } catch (e) {
        console.error('Error saving field settings:', e);
    }
}

async function fetchAvailableFields() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/schema/fields`);
        if (!response.ok) throw new Error('Failed to fetch fields');
        const data = await response.json();

        // Get top-level fields only (no nested)
        availableFields = data.fields
            .filter(f => !f.nested && !f.name.startsWith('_'))
            .map(f => f.name);

        // Add special UI fields
        availableFields = ['preview', ...availableFields, 'actions'];

        console.log('📋 Available fields:', availableFields);
        return availableFields;
    } catch (e) {
        console.error('Error fetching fields:', e);
        return DEFAULT_VISIBLE_FIELDS;
    }
}

function openFieldSettings() {
    const modal = document.getElementById('field-settings-modal');
    const checkboxContainer = document.getElementById('field-checkboxes');

    if (!modal || !checkboxContainer) return;

    modal.style.display = 'block';

    // Render checkboxes
    checkboxContainer.innerHTML = '';

    availableFields.forEach(field => {
        const isRequired = REQUIRED_FIELDS.includes(field);
        const isChecked = visibleFields.includes(field);
        const displayName = FIELD_DISPLAY_NAMES[field] || field;

        const div = document.createElement('div');
        div.style.cssText = 'display: flex; align-items: center; gap: 10px; padding: 8px; border-bottom: 1px solid rgba(255,255,255,0.1);';

        div.innerHTML = `
            <input type="checkbox" id="field-${field}" value="${field}" 
                   ${isChecked ? 'checked' : ''} 
                   ${isRequired ? 'disabled' : ''}>
            <label for="field-${field}" style="flex: 1; cursor: ${isRequired ? 'not-allowed' : 'pointer'};">
                ${displayName}
                ${isRequired ? '<span style="color:#fbbf24;font-size:0.8em;margin-left:5px;">(บังคับ)</span>' : ''}
            </label>
        `;

        checkboxContainer.appendChild(div);
    });
}

function closeFieldSettings() {
    const modal = document.getElementById('field-settings-modal');
    if (modal) modal.style.display = 'none';
}

function applyFieldSettings() {
    const checkboxes = document.querySelectorAll('#field-checkboxes input[type="checkbox"]');
    visibleFields = [];

    checkboxes.forEach(cb => {
        if (cb.checked) {
            visibleFields.push(cb.value);
        }
    });

    // Ensure required fields are always included
    REQUIRED_FIELDS.forEach(f => {
        if (!visibleFields.includes(f)) {
            visibleFields.push(f);
        }
    });

    saveFieldSettings();
    closeFieldSettings();

    // Re-render table with new settings
    renderTableHeaders();
    fetchAndDisplayLocations();

    alert('✅ บันทึกการตั้งค่าเรียบร้อย!');
}

function resetFieldSettings() {
    visibleFields = [...DEFAULT_VISIBLE_FIELDS];
    saveFieldSettings();

    // Re-check all checkboxes
    openFieldSettings();
}

function renderTableHeaders() {
    const thead = document.getElementById('locations-table-head');
    if (!thead) return;

    let headerHtml = '<tr>';

    visibleFields.forEach(field => {
        const displayName = FIELD_DISPLAY_NAMES[field] || field;
        let style = '';

        if (field === 'preview') style = 'width:100px;';
        if (field === 'actions') style = 'width:100px;';
        if (field === 'summary') style = 'max-width:200px;';
        if (field === 'keywords') style = 'max-width:150px;';

        headerHtml += `<th style="${style}">${displayName.replace(/^[^\s]+\s/, '')}</th>`;
    });

    headerHtml += '</tr>';
    thead.innerHTML = headerHtml;
}

// Initialize field settings on page load  
document.addEventListener('DOMContentLoaded', async () => {
    // Load saved settings
    loadFieldSettings();

    // Fetch available fields from API
    await fetchAvailableFields();

    // Render table headers based on settings
    renderTableHeaders();
});