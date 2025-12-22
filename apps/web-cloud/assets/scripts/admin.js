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
    // [V5.4] Adjusted colspan from 6 to 7 to account for the new "Preview" column + Checkbox
    const colSpanCount = (visibleFields.length || 7) + 1;
    locationsTableBody.innerHTML = `<tr><td colspan="${colSpanCount}">Loading data...</td></tr>`;

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
            locationsTableBody.innerHTML = `<tr><td colspan="${colSpanCount}">No locations found. Add one below!</td></tr>`;
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
            let rowHtml = `
                <td style="text-align: center; vertical-align: middle;">
                    <input type="checkbox" class="row-checkbox" value="${location.slug}" onchange="updateBulkActionState()" onclick="event.stopPropagation()" style="transform: scale(1.5); cursor: pointer;">
                </td>
            `;

            // Allow clicking the row to toggle checkbox
            row.onclick = function (e) {
                // Ignore if clicked on button or link
                if (e.target.closest('button') || e.target.closest('a') || e.target.closest('input')) return;

                const cb = this.querySelector('.row-checkbox');
                if (cb) {
                    cb.checked = !cb.checked;
                    updateBulkActionState();
                }
            };
            row.style.cursor = 'pointer';

            visibleFields.forEach(field => {
                let cellContent = '';
                let cellStyle = '';
                // ... (rest of switch case, unchanged logic within loop structure)

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

        // Reset select all button state
        updateBulkActionState();

        renderPaginationControls(totalItems, currentPage, itemsPerPage);

    } catch (error) {
        console.error('Fetch error:', error);
        // [V5.4] Adjusted colspan
        const colSpanCount = (visibleFields.length || 7) + 1;
        locationsTableBody.innerHTML = `<tr><td colspan="${colSpanCount}">Failed to load data. Please check connection.</td></tr>`;
    }
}

async function deleteLocation(slug, silent = false) {
    if (!slug) {
        if (!silent) alert('Error: Invalid slug provided for deletion.');
        return false;
    }
    if (!silent && !confirm(`คุณแน่ใจหรือไม่ว่าจะลบรายการ "${slug}"?`)) return false;

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/${slug}`, { method: 'DELETE' });
        if (response.status === 204) {
            if (!silent) {
                alert(`ลบข้อมูล "${slug}" เรียบร้อยแล้ว!`);
                fetchAndDisplayLocations(); // Refresh table
            }
            return true;
        } else {
            if (!silent) {
                let errorDetail = response.statusText;
                try {
                    const errorData = await response.json();
                    errorDetail = errorData.detail || errorDetail;
                } catch (e) { /* Ignore */ }
                alert(`ลบข้อมูลไม่สำเร็จ: ${errorDetail}`);
            }
            return false;
        }
    } catch (error) {
        console.error('Delete error:', error);
        if (!silent) alert('เกิดข้อผิดพลาดขณะทำการลบ (ดู Console)');
        return false;
    }
}

async function deleteSelectedItems() {
    const selectedCheckboxes = document.querySelectorAll('.row-checkbox:checked');
    if (selectedCheckboxes.length === 0) {
        alert('กรุณาเลือกรายการที่ต้องการลบ');
        return;
    }

    if (!confirm(`⚠️ ยืนยันการลบ ${selectedCheckboxes.length} รายการที่เลือก?\n\nการกระทำนี้ไม่สามารถย้อนกลับได้!`)) {
        return;
    }

    let deletedCount = 0;

    // Show loading state on button
    const btn = document.getElementById('bulk-actions').querySelector('button');
    const originalText = btn.innerHTML;
    btn.innerHTML = '⏳ กำลังลบ...';
    btn.disabled = true;

    try {
        const slugs = Array.from(selectedCheckboxes).map(cb => cb.value);

        // Delete sequentially to avoid DB lock issues
        for (const slug of slugs) {
            const success = await deleteLocation(slug, true); // true = silent mode
            if (success) {
                deletedCount++;
                // Update progress
                btn.innerHTML = `⏳ ลบแล้ว ${deletedCount}/${slugs.length}`;
            }
        }

    } catch (error) {
        console.error('Bulk delete error:', error);
    } finally {
        alert(`ดำเนินการเสร็จสิ้น! ลบไปทั้งหมด ${deletedCount} รายการ`);
        btn.innerHTML = originalText;
        btn.disabled = false;
        fetchAndDisplayLocations();
    }
}

function toggleSelectAllBtn(btn) {
    const isSelecting = btn.dataset.state !== 'selected';
    const checkboxes = document.querySelectorAll('.row-checkbox');

    checkboxes.forEach(cb => {
        cb.checked = isSelecting;
    });

    updateBulkActionState();
}

function updateBulkActionState() {
    const selectedCount = document.querySelectorAll('.row-checkbox:checked').length;
    const bulkActionsContainer = document.getElementById('bulk-actions');
    const selectedCountSpan = document.getElementById('selected-count');

    // Toggle Button Logic
    const selectAllBtn = document.getElementById('select-all-btn');
    const allCheckboxes = document.querySelectorAll('.row-checkbox');

    if (selectAllBtn) {
        if (allCheckboxes.length > 0 && selectedCount === allCheckboxes.length) {
            selectAllBtn.innerText = 'ยกเลิก';
            selectAllBtn.dataset.state = 'selected';
            selectAllBtn.style.background = 'rgba(239,68,68,0.2)';
            selectAllBtn.style.color = '#ef4444';
        } else {
            selectAllBtn.innerText = 'เลือกทั้งหมด';
            selectAllBtn.dataset.state = 'unselected';
            selectAllBtn.style.background = 'rgba(102,126,234,0.1)';
            selectAllBtn.style.color = '#667eea';
        }
    }

    if (selectedCount > 0) {
        bulkActionsContainer.style.display = 'block';
        selectedCountSpan.innerText = selectedCount;
    } else {
        bulkActionsContainer.style.display = 'none';
    }
}

function renderPaginationControls(total, current, limit) {
    if (!paginationContainer) return;
    paginationContainer.innerHTML = '';

    const totalPages = Math.max(1, Math.ceil(total / limit)); // At least 1 page

    // First Button
    const firstBtn = document.createElement('button');
    firstBtn.innerText = 'First';
    firstBtn.className = 'pagination-btn';
    firstBtn.disabled = current <= 1;
    firstBtn.onclick = () => { currentPage = 1; fetchAndDisplayLocations(); };
    paginationContainer.appendChild(firstBtn);

    // Prev Button
    const prevBtn = document.createElement('button');
    prevBtn.innerText = 'Prev';
    prevBtn.className = 'pagination-btn';
    prevBtn.disabled = current <= 1;
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
    nextBtn.disabled = current >= totalPages;
    nextBtn.onclick = () => { if (current < totalPages) { currentPage++; fetchAndDisplayLocations(); } };
    paginationContainer.appendChild(nextBtn);

    // Last Button
    const lastBtn = document.createElement('button');
    lastBtn.innerText = 'Last';
    lastBtn.className = 'pagination-btn';
    lastBtn.disabled = current >= totalPages;
    lastBtn.onclick = () => { currentPage = totalPages; fetchAndDisplayLocations(); };
    paginationContainer.appendChild(lastBtn);
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
    initGoogleSheetsModule();
});

// ==========================================================
//  GOOGLE SHEETS SYNC FUNCTIONS
// ==========================================================



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

    // Add Select All Button Header
    headerHtml += `
        <th style="width: 100px; text-align: center;">
            <button id="select-all-btn" onclick="toggleSelectAllBtn(this)" 
                style="padding: 4px 8px; font-size: 0.8em; border-radius: 4px; border: 1px solid rgba(102,126,234,0.5); background: rgba(102,126,234,0.1); color: #667eea; cursor: pointer;">
                เลือกทั้งหมด
            </button>
        </th>
    `;

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