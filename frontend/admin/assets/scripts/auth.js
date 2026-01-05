// Basic Auth Logic for Admin Pages
// Uses global API_BASE_URL from config.js

document.addEventListener('DOMContentLoaded', () => {
    const loginForm = document.getElementById('login-form');
    if (loginForm) {
        loginForm.addEventListener('submit', handleLogin);
    }

    // Check auth on pages (Redirect to login if unauthorized)
    if (!window.location.pathname.includes('login.html')) {
        checkAuth();
    } else {
        // If on login page, check if already logged in
        checkAuth(true);
    }
});

async function handleLogin(e) {
    e.preventDefault();

    const username = document.getElementById('username').value;
    const password = document.getElementById('password').value;
    const errorMsg = document.getElementById('error-msg');
    const btn = e.target.querySelector('button');

    // Clear error
    errorMsg.innerText = '';
    btn.disabled = true;
    btn.innerText = 'กำลังตรวจสอบ...';

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/auth/login`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ username, password }),
            credentials: 'include' // Important for CORS Cookies
        });

        if (response.ok) {
            window.location.href = 'index.html';
        } else {
            const data = await response.json();
            throw new Error(data.detail || 'Login failed');
        }
    } catch (error) {
        errorMsg.innerText = `❌ ${error.message}`;
        btn.disabled = false;
        btn.innerText = 'เข้าสู่ระบบ';
    }
}

async function checkAuth(isLoginPage = false) {
    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/auth/me`, {
            credentials: 'include'
        });

        if (response.ok) {
            if (isLoginPage) {
                window.location.href = 'index.html'; // Already logged in
            }
        } else {
            if (!isLoginPage) {
                // Token invalid or expired
                window.location.href = 'login.html';
            }
        }
    } catch (e) {
        console.error("Auth check failed", e);
        if (!isLoginPage) window.location.href = 'login.html';
    }
}

async function logout() {
    try {
        await fetch(`${API_BASE_URL}/api/admin/auth/logout`, {
            method: 'POST',
            credentials: 'include'
        });
        window.location.href = 'login.html';
    } catch (e) {
        console.error("Logout failed", e);
        window.location.href = 'login.html';
    }
}

window.logout = logout; // Expose to global scope for button click
