console.log("%c ADMIN.JS LOADED - V.ULTIMATE FINAL -> V5.4.0 (Image Previews)", "color: lime; font-size: 16px; font-weight: bold;");

// --- Global Variables ---
let locationsTableBody, addLocationForm, editModal, editLocationForm, closeModalButton, fileInput, analyzeBtn, loadingSpinner;
// Assume API_BASE_URL is defined globally in config.js
// const API_BASE_URL = ''; // DO NOT REDECLARE

// --- Core Functions (V5.4.0 - Image Previews) ---

async function fetchAndDisplayLocations() {
    if (!locationsTableBody) {
        console.error("locationsTableBody not found during fetch");
        return;
    }
    // [V5.4] Adjusted colspan from 6 to 7 to account for the new "Preview" column
    locationsTableBody.innerHTML = '<tr><td colspan="7">Loading data...</td></tr>';

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/`);
        if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
        const locations = await response.json();

        locationsTableBody.innerHTML = '';

        if (!Array.isArray(locations) || locations.length === 0) {
            // [V5.4] Adjusted colspan
            locationsTableBody.innerHTML = '<tr><td colspan="7">No locations found. Add one below!</td></tr>';
            return;
        }

        locations.forEach(location => {
            const hasImageLink = location.metadata && location.metadata.image_prefix;
            const imageIndicator = hasImageLink
                ? `<span style="color: #4ade80;">✔️ Yes (${location.metadata.image_prefix})</span>`
                : '<span style="color: #f87171;">❌ No</span>';

            // --- [V5.5] Image Preview Logic (Robust Fallback) ---
            const placeholderImage = 'data:image/svg+xml;charset=UTF-8,%3Csvg%20width%3D%22100%22%20height%3D%2275%22%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%20100%2075%22%20preserveAspectRatio%3D%22none%22%3E%3Cdefs%3E%3Cstyle%20type%3D%22text%2Fcss%22%3E%23holder_1%20text%20%7B%20fill%3A%23AAAAAA%3Bfont-weight%3Abold%3Bfont-family%3AArial%2C%20Helvetica%2C%20Open%20Sans%2C%20sans-serif%2C%20monospace%3Bfont-size%3A10pt%20%7D%20%3C%2Fstyle%3E%3C%2Fdefs%3E%3Cg%20id%3D%22holder_1%22%3E%3Crect%20width%3D%22100%22%20height%3D%2275%22%20fill%3D%22%23EEEEEE%22%3E%3C%2Frect%3E%3Cg%3E%3Ctext%20x%3D%2227.5%22%20y%3D%2242%22%3ENo Image%3C%2Ftext%3E%3C%2Fg%3E%3C%2Fg%3E%3C%2Fsvg%3E';

            let primaryUrl = placeholderImage;
            let secondaryUrl = null;

            if (location.preview_image_url) {
                primaryUrl = `${API_BASE_URL}${location.preview_image_url}`;
            } else {
                // 1. Try Image Prefix
                if (location.metadata && location.metadata.image_prefix) {
                    let prefixName = location.metadata.image_prefix.trim().replace(/\s+/g, '-').replace(/-+$/, '');
                    primaryUrl = `${API_BASE_URL}/static/images/${prefixName}-01.jpg`;
                }

                // 2. Prepare Slug Fallback
                if (location.slug) {
                    let slugName = location.slug.trim().replace(/\s+/g, '-').replace(/-+$/, '');
                    let slugUrl = `${API_BASE_URL}/static/images/${slugName}-01.jpg`;

                    if (primaryUrl === placeholderImage) {
                        primaryUrl = slugUrl; // No prefix, so slug is primary
                    } else if (primaryUrl !== slugUrl) {
                        secondaryUrl = slugUrl; // Prefix exists, so slug is secondary
                    }
                }
            }

            let imgOnError = `this.onerror=null; this.src='${placeholderImage}';`;
            if (secondaryUrl) {
                imgOnError = `if (!this.dataset.triedSlug) { this.dataset.triedSlug=true; this.src='${secondaryUrl}'; } else { this.src='${placeholderImage}'; }`;
            }

            const imagePreviewHtml = `<img src="${primaryUrl}" alt="Preview for ${location.slug}" style="width: 100px; height: 75px; object-fit: cover; border-radius: 4px; background-color: #f0f0f0;" onerror="${imgOnError}">`;
            // --- [END V5.5] ---

            const row = document.createElement('tr');
            row.innerHTML = `
                <td>${imagePreviewHtml}</td>
                <td>${location.slug || 'N/A'}</td>
                <td>${location.title || 'N/A'}</td>
                <td>${location.category || 'N/A'}</td>
                <td>${location.topic || 'N/A'}</td>
                <td>${imageIndicator}</td>
                <td>
                    <button class="btn btn-edit" data-slug="${location.slug}">แก้ไข</button>
                    <button class="btn btn-delete" data-slug="${location.slug}">ลบ</button>
                </td>
            `;
            locationsTableBody.appendChild(row);
        });
    } catch (error) {
        console.error('Fetch error:', error);
        // [V5.4] Adjusted colspan
        locationsTableBody.innerHTML = '<tr><td colspan="7">Failed to load data. Please check connection.</td></tr>';
    }
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

        // --- [V5.5] Populate Image Preview in Modal (Robust Fallback) ---
        const imagePreviewEl = document.getElementById('edit-form-image-preview');
        const noImageTextEl = document.getElementById('edit-form-no-image');

        // Reset state
        if (imagePreviewEl) {
            imagePreviewEl.onerror = null;
            imagePreviewEl.removeAttribute('data-tried-slug');
            imagePreviewEl.style.display = 'none';
        }
        if (noImageTextEl) noImageTextEl.style.display = 'none';

        let primaryUrl = location.preview_image_url ? `${API_BASE_URL}${location.preview_image_url}` : null;
        let secondaryUrl = null;

        if (!primaryUrl) {
            // 1. Try Image Prefix
            if (location.metadata && location.metadata.image_prefix) {
                let prefixName = location.metadata.image_prefix.trim().replace(/\s+/g, '-').replace(/-+$/, '');
                primaryUrl = `${API_BASE_URL}/static/images/${prefixName}-01.jpg`;
            }

            // 2. Prepare Slug Fallback
            if (location.slug) {
                let slugName = location.slug.trim().replace(/\s+/g, '-').replace(/-+$/, '');
                let slugUrl = `${API_BASE_URL}/static/images/${slugName}-01.jpg`;

                if (!primaryUrl) {
                    primaryUrl = slugUrl;
                } else if (primaryUrl !== slugUrl) {
                    secondaryUrl = slugUrl;
                }
            }
        }

        if (primaryUrl && imagePreviewEl && noImageTextEl) {
            imagePreviewEl.src = primaryUrl;
            imagePreviewEl.style.display = 'block';

            imagePreviewEl.onerror = function () {
                if (secondaryUrl && !this.dataset.triedSlug) {
                    this.dataset.triedSlug = "true";
                    this.src = secondaryUrl;
                } else {
                    this.style.display = 'none';
                    noImageTextEl.style.display = 'block';
                }
            };
        } else if (noImageTextEl) {
            noImageTextEl.style.display = 'block';
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


// --- Functions below this line are unchanged from V5.3.1 ---

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
    loadingSpinner = document.getElementById('loading-spinner');

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
});