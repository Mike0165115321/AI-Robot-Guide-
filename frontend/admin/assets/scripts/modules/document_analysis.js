
async function handleAnalyzeDocument() {
    const fileInput = document.getElementById('file-input');
    const analyzeBtn = document.getElementById('analyze-btn');
    const loadingSpinner = document.getElementById('loading-spinner');

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
