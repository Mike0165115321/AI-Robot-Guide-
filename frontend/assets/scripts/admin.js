console.log("%c ADMIN.JS LOADED - V.ULTIMATE FINAL -> V5.2", "color: lime; font-size: 16px; font-weight: bold;");

// --- Global Variables ---
let locationsTableBody, addLocationForm, editModal, editLocationForm, closeModalButton, fileInput, analyzeBtn, loadingSpinner;
// Assume API_BASE_URL is defined globally in config.js
// const API_BASE_URL = ''; // DO NOT REDECLARE

// --- Core Functions (V5.2 - Updated for Slug) ---

async function fetchAndDisplayLocations() {
    if (!locationsTableBody) {
        console.error("locationsTableBody not found during fetch");
        return;
    }
    locationsTableBody.innerHTML = '<tr><td colspan="6">Loading data...</td></tr>'; // Add loading state

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/`);
        if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
        const locations = await response.json();

        locationsTableBody.innerHTML = ''; // Clear loading/previous data

        if (!Array.isArray(locations) || locations.length === 0) {
            locationsTableBody.innerHTML = '<tr><td colspan="6">No locations found. Add one below!</td></tr>';
            return;
        }

        locations.forEach(location => {
            const hasImageLink = location.metadata && location.metadata.image_prefix;
            const imageIndicator = hasImageLink
                ? `<span style="color: #4ade80;">✔️ Yes (${location.metadata.image_prefix})</span>`
                : '<span style="color: #f87171;">❌ No</span>';

            const row = document.createElement('tr');
            // --- [ V5.2 FINAL - Cleaned ] ---
            // Removed {/* ... */} comments from this template literal
            row.innerHTML = `
                <td>${location.slug || 'N/A'}</td>
                <td>${location.title || 'N/A'}</td>
                <td>${location.category || 'N/A'}</td>
                <td>${location.topic || 'N/A'}</td>
                <td>${imageIndicator}</td>
                <td>
                    <button class="btn-edit" data-slug="${location.slug}" style="background-color: #3b82f6; color: white; padding: 4px 8px; border-radius: 4px; border: none; cursor: pointer;">แก้ไข</button>
                    <button class="btn-delete" data-slug="${location.slug}" style="background-color: #ef4444; color: white; padding: 4px 8px; border-radius: 4px; border: none; cursor: pointer;">ลบ</button>
                </td>
            `;
            // --- [ END Cleaned ] ---
            locationsTableBody.appendChild(row);
        });
    } catch (error) {
        console.error('Fetch error:', error);
        locationsTableBody.innerHTML = '<tr><td colspan="6">Failed to load data. Please check connection.</td></tr>';
    }
}

// [V5.2 FIX] Change parameter and fetch URL to use slug
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
             // Try parsing JSON, otherwise use status text
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

// [V5.2 FIX] Change parameter and fetch URL to use slug
async function openEditModal(slug) {
     if (!slug) {
        alert('Error: Invalid slug provided for editing.');
        return;
    }
    console.log(`Opening edit modal for slug: ${slug}`);

    // Ensure modal and form elements exist before proceeding
    if (!editModal || !editLocationForm) {
        console.error("Edit modal or form not found.");
        alert("ข้อผิดพลาด: ไม่พบองค์ประกอบฟอร์มแก้ไข");
        return;
    }


    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/${slug}`);
        if (!response.ok) throw new Error(`Failed to fetch location details for "${slug}". Status: ${response.status}`);
        const location = await response.json();

        // Populate hidden fields first
        document.getElementById('edit-form-mongo-id').value = location.mongo_id || ''; // Handle missing mongo_id
        document.getElementById('edit-form-slug').value = location.slug;

        // Populate visible fields (Use || '' as default)
        // [V5.2] Added display-slug population
        const displaySlugField = document.getElementById('display-slug');
        if (displaySlugField) displaySlugField.value = location.slug; // Populate the disabled display field

        document.getElementById('edit-form-title').value = location.title || '';
        document.getElementById('edit-form-summary').value = location.summary || '';
        document.getElementById('edit-form-category').value = location.category || '';
        document.getElementById('edit-form-topic').value = location.topic || '';

        const imagePrefix = (location.metadata && location.metadata.image_prefix) ? location.metadata.image_prefix : '';
        document.getElementById('edit-form-image-prefix').value = imagePrefix;
        document.getElementById('edit-form-image-file').value = ''; // Clear file input

        editModal.style.display = 'block';
    } catch (error) {
        console.error('Error opening edit modal:', error);
        alert(`Could not open the edit form for "${slug}". Reason: ${error.message}`);
    }
}

// [V5.2 FIX] Update to handle 'slug' input
async function handleAddLocationSubmit(event) {
    event.preventDefault();
    if (!addLocationForm) return; // Should not happen if listener is attached

    const addBtn = addLocationForm.querySelector('button[type="submit"]');
    addBtn.disabled = true; addBtn.textContent = 'Processing...';

    const slug = document.getElementById('form-slug').value.trim();
    if (!slug) {
        alert("กรุณากรอก Slug (Key ที่ไม่ซ้ำกัน)");
        addBtn.disabled = false; addBtn.textContent = 'Add Location'; return;
    }
    if (!/^[a-z0-9_-]{3,}$/.test(slug)) { // Corrected regex check for length >= 3
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
    // Only set prefix if it's explicitly provided and matches slug, or if no image is uploaded
    const finalImagePrefixForMeta = imagePrefixInput === slug ? slug : null;


    if (imageFile && slug) { // Now slug acts as the prefix target
        const imageFormData = new FormData();
        imageFormData.append('file', imageFile);
        try {
             // Send slug via query parameter as expected by admin_api.py V5.2
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
            uploadedImagePrefix = imageData.image_prefix; // Confirm the prefix used
            alert(`อัปโหลดรูปภาพสำเร็จ! บันทึกเป็น: ${imageData.saved_as}`);
        } catch (error) {
            console.error('Image upload error:', error);
            alert(`เกิดข้อผิดพลาดในการอัปโหลดรูปภาพ: ${error.message}`);
            addBtn.disabled = false; addBtn.textContent = 'Add Location'; return;
        }
    }

    const newLocationData = {
        slug: slug,
        title: document.getElementById('form-title').value,
        summary: document.getElementById('form-summary').value,
        category: document.getElementById('form-category').value,
        topic: document.getElementById('form-topic').value,
        // Use confirmed prefix if uploaded, else use the input prefix if it matched slug, else null
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
                 if (response.status === 400 && errorDetail.includes("unique") || errorDetail.includes("duplicate key")) {
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

// [V5.2 FIX] Update to handle 'slug' input and URL
async function handleEditFormSubmit(event) {
    event.preventDefault();
     if (!editLocationForm) return; // Should not happen

    const saveBtn = editLocationForm.querySelector('button[type="submit"]');
    saveBtn.disabled = true; saveBtn.textContent = 'Saving...';

    const slug = document.getElementById('edit-form-slug').value;
    if (!slug) {
         alert("ข้อผิดพลาด: ไม่พบ Slug สำหรับการแก้ไข");
         saveBtn.disabled = false; saveBtn.textContent = 'Save Changes'; return;
    }

    // const mongoId = document.getElementById('edit-form-mongo-id').value; // Not used for API call

    const imagePrefixInput = document.getElementById('edit-form-image-prefix').value.trim();
    const imageFile = document.getElementById('edit-form-image-file').files[0];
    let finalImagePrefix = imagePrefixInput; // Assume current/edited prefix unless changed by upload

     // Validation: If a prefix is provided, it must match the slug (or be empty to remove)
    if (imagePrefixInput && imagePrefixInput !== slug) {
         alert("Image Prefix ต้องตรงกับ Slug (หรือไม่ต้องกรอกเพื่อลบ)");
         saveBtn.disabled = false; saveBtn.textContent = 'Save Changes'; return;
    }
     // Validation: If uploading a file, the prefix input must match the slug
     if (imageFile && imagePrefixInput !== slug) {
         alert("Image Prefix ต้องตรงกับ Slug เมื่ออัปโหลดไฟล์ใหม่");
         saveBtn.disabled = false; saveBtn.textContent = 'Save Changes'; return;
     }


    if (imageFile && slug) { // Upload if file selected AND prefix matches slug
        const imageFormData = new FormData();
        imageFormData.append('file', imageFile);
        try {
             // Use slug as the prefix for uploading
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
            finalImagePrefix = imageData.image_prefix; // Update prefix based on upload result
            alert(`อัปโหลดรูปภาพใหม่สำเร็จ! บันทึกเป็น: ${imageData.saved_as}`);
             // Update the prefix field visually
             document.getElementById('edit-form-image-prefix').value = finalImagePrefix;
             document.getElementById('edit-form-image-file').value = ''; // Clear file input
        } catch (error) {
             console.error('Image upload error during edit:', error);
            alert(`เกิดข้อผิดพลาดในการอัปโหลดรูปภาพใหม่: ${error.message}`);
            saveBtn.disabled = false; saveBtn.textContent = 'Save Changes'; return;
        }
    }

    const updatedData = {
        slug: slug, // Include slug in the body
        title: document.getElementById('edit-form-title').value,
        summary: document.getElementById('edit-form-summary').value,
        category: document.getElementById('edit-form-category').value,
        topic: document.getElementById('edit-form-topic').value,
        // Use final prefix (either original, edited, or from upload result)
        // If finalImagePrefix is empty string, set metadata to null
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
            if(editModal) editModal.style.display = 'none';
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
    // ... (This function looks okay, ensure form IDs match HTML) ...
     if (!fileInput || !fileInput.files || fileInput.files.length === 0) {
        alert('Please select a document file first.');
        return;
    }
    const file = fileInput.files[0];
    const formData = new FormData();
    formData.append('file', file);

    // Ensure buttons/spinners exist
    if (!analyzeBtn || !loadingSpinner) {
        console.error("Analyze button or loading spinner not found.");
        return;
    }

    analyzeBtn.disabled = true;
    loadingSpinner.style.display = 'inline-block';
    analyzeBtn.textContent = 'Analyzing...'; // Use textContent for button text

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

        // Populate the "Add Location" form (ensure form elements exist)
        const formTitle = document.getElementById('form-title');
        const formSummary = document.getElementById('form-summary');
        const formCategory = document.getElementById('form-category');
        const formTopic = document.getElementById('form-topic');
        const formSlug = document.getElementById('form-slug'); // Target the slug field

        if (formTitle) formTitle.value = data.title || '';
        if (formSummary) formSummary.value = data.summary || '';
        if (formCategory) formCategory.value = data.category || '';
        if (formTopic) formTopic.value = data.topic || '';

        // Auto-generate and populate slug
        if (formSlug) {
            // Use the same robust slug generation logic as add_image_links.py if possible
            const titleForSlug = data.title || `item_${Date.now()}`;
            let generatedSlug = titleForSlug.toLowerCase().trim()
                                     .replace(/[\s\(\)\[\]{}]+/g, '_') // Replace spaces and brackets with _
                                     .replace(/[^a-z0-9_-]/g, '')    // Remove invalid chars
                                     .replace(/[-_]+/g, '_')         // Collapse multiple _ or -
                                     .replace(/^-+|-+$/g, '')       // Trim leading/trailing _ or -
                                     .substring(0, 50);            // Max length
            if (!generatedSlug) generatedSlug = `item_${Date.now()}`; // Fallback if empty
            formSlug.value = generatedSlug;
        }


        alert('Document analyzed successfully! Form fields have been populated.');

    } catch (error) {
        console.error('Document analysis error:', error);
        alert(`Failed to analyze document: ${error.message}`);
    } finally {
        analyzeBtn.disabled = false;
        loadingSpinner.style.display = 'none';
        analyzeBtn.textContent = 'Analyze Document'; // Reset button text
        if(fileInput) fileInput.value = ''; // Clear file input
    }
}


// ==========================================================
//  EVENT LISTENERS (V5.2 - Updated for Slug)
// ==========================================================

document.addEventListener('DOMContentLoaded', () => {
    // --- Get Element References ---
    locationsTableBody = document.querySelector('#locations-table tbody');
    addLocationForm = document.getElementById('add-location-form');
    editModal = document.getElementById('edit-modal');
    editLocationForm = document.getElementById('edit-location-form');
    // More specific selector for close button inside modal
    closeModalButton = document.querySelector('#edit-modal .close-button');
    fileInput = document.getElementById('file-input');
    analyzeBtn = document.getElementById('analyze-btn');
    loadingSpinner = document.getElementById('loading-spinner');

    // --- Initial Data Load ---
    if (locationsTableBody) {
        fetchAndDisplayLocations();
    } else {
        console.error("Critical Error: locationsTableBody not found.");
    }

    // --- Form Event Listeners ---
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

    // --- Modal Close Button ---
    if (closeModalButton && editModal) {
        closeModalButton.addEventListener('click', () => {
             editModal.style.display = 'none';
        });
    } else {
        if (!closeModalButton) console.warn("Close modal button (.close-button inside #edit-modal) not found.");
        if (!editModal) console.warn("Edit modal (#edit-modal) not found.");
    }

    // --- Document Analysis Button ---
     if (analyzeBtn) {
        analyzeBtn.addEventListener('click', handleAnalyzeDocument);
    } else {
        console.warn("Analyze document button (#analyze-btn) not found.");
    }

    // --- Event Delegation for Edit/Delete Buttons ---
    if (locationsTableBody) {
        locationsTableBody.addEventListener('click', (event) => {
            const targetButton = event.target.closest('button');
            if (!targetButton) return;

            // [V5.2 FIX] Read 'data-slug'
            const slug = targetButton.dataset.slug;

            if (targetButton.classList.contains('btn-edit')) {
                console.log(`Edit button clicked for slug: ${slug}`);
                openEditModal(slug);
            } else if (targetButton.classList.contains('btn-delete')) {
                console.log(`Delete button clicked for slug: ${slug}`);
                deleteLocation(slug);
            }
        });
    }

    // --- Close Modal on Outside Click ---
    window.onclick = function (event) {
        // Ensure editModal exists before checking target
        if (editModal && event.target == editModal) {
            editModal.style.display = "none";
        }
    }
}); // End DOMContentLoaded