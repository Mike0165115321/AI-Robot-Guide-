// /frontend/assets/scripts/admin.js (ฉบับแก้ไขสมบูรณ์ขั้นสุดท้าย)

console.log("%c RUNNING LATEST ADMIN.JS - V.FINAL (Event Delegation)", "color: lime; font-size: 16px; font-weight: bold;");

const locationsTableBody = document.querySelector('#locations-table tbody');
const addLocationForm = document.getElementById('add-location-form');
const editModal = document.getElementById('edit-modal');
const editLocationForm = document.getElementById('edit-location-form');
const closeModalButton = document.querySelector('.close-button');
const fileInput = document.getElementById('file-input');
const analyzeBtn = document.getElementById('analyze-btn');
const loadingSpinner = document.getElementById('loading-spinner');

async function fetchAndDisplayLocations() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/`);
        if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
        const locations = await response.json();

        locationsTableBody.innerHTML = '';

        locations.forEach(location => {
            const row = document.createElement('tr');
            row.innerHTML = `
                <td>${location.id}</td>
                <td>${location.title}</td>
                <td>${location.category}</td>
                <td>${location.topic}</td>
                <td>
                    <button class="btn-edit" data-id="${location._id}">Edit</button> 
                    <button class="btn-delete" data-id="${location._id}">Delete</button>
                </td>
            `;
            locationsTableBody.appendChild(row);
        });
    } catch (error) {
        console.error('Fetch error:', error);
        locationsTableBody.innerHTML = '<tr><td colspan="5">Failed to load data.</td></tr>';
    }
}

async function deleteLocation(mongoId) {
    if (!confirm(`Are you sure you want to delete item with ID: ${mongoId}?`)) return;
    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/${mongoId}`, { method: 'DELETE' });
        if (response.status === 204) {
            alert('Location deleted successfully!');
            fetchAndDisplayLocations();
        } else {
            const errorData = await response.json();
            alert(`Failed to delete location: ${errorData.detail}`);
        }
    } catch (error) {
        console.error('Delete error:', error);
        alert('An error occurred while deleting the location.');
    }
}

async function openEditModal(mongoId) {
    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/${mongoId}`);
        if (!response.ok) {
            const errorData = await response.json();
            console.error("Server response:", errorData);
            throw new Error(errorData.detail || 'Failed to fetch location details.');
        }
        const location = await response.json();

        document.getElementById('edit-form-mongo-id').value = location._id;
        document.getElementById('edit-form-original-id').value = location.id;
        document.getElementById('edit-form-title').value = location.title;
        document.getElementById('edit-form-summary').value = location.summary;
        document.getElementById('edit-form-category').value = location.category;
        document.getElementById('edit-form-topic').value = location.topic;
        document.getElementById('edit-form-image-url').value = location.image_url || '';
        editModal.style.display = 'block';
    } catch (error) {
        console.error('Edit error:', error);
        alert(`Could not open the edit form. Reason: ${error.message}`);
    }
}

async function handleAddLocationSubmit(event) {
    event.preventDefault();
    const newLocationData = {
        id: `NEW_${Date.now()}`, title: document.getElementById('form-title').value,
        summary: document.getElementById('form-summary').value, category: document.getElementById('form-category').value,
        topic: document.getElementById('form-topic').value, image_url: document.getElementById('form-image-url').value,
        details: [], keywords: [], related_info: [], sources: []
    };
    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/`, {
            method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(newLocationData)
        });
        if (response.status === 201) {
            alert('Location added successfully!');
            addLocationForm.reset(); fetchAndDisplayLocations();
        } else {
            const errorData = await response.json();
            alert(`Failed to add location: ${errorData.detail}`);
        }
    } catch (error) {
        console.error('Add location error:', error);
        alert('An error occurred while adding the location.');
    }
}

async function handleEditFormSubmit(event) {
    event.preventDefault();
    const mongoId = document.getElementById('edit-form-mongo-id').value;
    const updatedData = {
        id: document.getElementById('edit-form-original-id').value,
        title: document.getElementById('edit-form-title').value,
        summary: document.getElementById('edit-form-summary').value,
        category: document.getElementById('edit-form-category').value,
        topic: document.getElementById('edit-form-topic').value,
        image_url: document.getElementById('edit-form-image-url').value,
    };
    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/${mongoId}`, {
            method: 'PUT', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(updatedData)
        });
        if (response.ok) {
            alert('Location updated successfully!');
            editModal.style.display = 'none';
            fetchAndDisplayLocations();
        } else {
            const errorData = await response.json();
            alert(`Failed to update location: ${errorData.detail}`);
        }
    } catch (error) {
        console.error('Update error:', error);
        alert('An error occurred while updating the location.');
    }
}
async function handleAnalyzeDocument() {
    if (fileInput.files.length === 0) {
        alert('กรุณาเลือกไฟล์ PDF หรือรูปภาพก่อนครับ');
        return;
    }

    const file = fileInput.files[0];
    const formData = new FormData();
    formData.append('file', file);

    loadingSpinner.style.display = 'block';
    analyzeBtn.disabled = true;
    analyzeBtn.textContent = 'กำลังวิเคราะห์...';

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/locations/analyze-document`, {
            method: 'POST',
            body: formData,
        });

        if (!response.ok) {
            const errorData = await response.json();
            throw new Error(errorData.detail || 'เกิดข้อผิดพลาดในการประมวลผลไฟล์');
        }

        const extractedData = await response.json();

        document.getElementById('form-title').value = extractedData.title || '';
        document.getElementById('form-summary').value = extractedData.summary || '';
        document.getElementById('form-category').value = extractedData.category || '';
        document.getElementById('form-topic').value = extractedData.topic || '';

        if (extractedData.details && Array.isArray(extractedData.details)) {
            const detailsText = extractedData.details.map(detail => {
                return `หัวข้อ: ${detail.heading}\n\n${detail.content}\n--------------------\n`;
            }).join('\n');

            document.getElementById('form-details-preview').value = detailsText;
        } else {
            document.getElementById('form-details-preview').value = 'AI ไม่พบข้อมูลรายละเอียดเชิงลึก';
        }

        console.log('Keywords extracted:', extractedData.keywords);

        alert('AI สกัดข้อมูลและเติมลงในฟอร์มเรียบร้อยแล้ว! กรุณาตรวจสอบและกด Add Location');

    } catch (error) {
        console.error('Analysis Error:', error);
        alert(`เกิดข้อผิดพลาด: ${error.message}`);
    } finally {
        loadingSpinner.style.display = 'none';
        analyzeBtn.disabled = false;
        analyzeBtn.textContent = 'ประมวลผลด้วย AI';
        fileInput.value = '';
    }
}

document.addEventListener('DOMContentLoaded', () => {
    fetchAndDisplayLocations();
    addLocationForm.addEventListener('submit', handleAddLocationSubmit);
    editLocationForm.addEventListener('submit', handleEditFormSubmit);
    closeModalButton.addEventListener('click', () => { editModal.style.display = 'none'; });
    locationsTableBody.addEventListener('click', (event) => {
        const editButton = event.target.closest('.btn-edit');
        if (editButton) {
            const mongoId = editButton.dataset.id;
            openEditModal(mongoId);
            return;
        }

        const deleteButton = event.target.closest('.btn-delete');
        if (deleteButton) {
            const mongoId = deleteButton.dataset.id;
            deleteLocation(mongoId);
            return;
        }
    });
    analyzeBtn.addEventListener('click', handleAnalyzeDocument);
});

window.onclick = function (event) {
    if (event.target == editModal) {
        editModal.style.display = "none";
    }
}