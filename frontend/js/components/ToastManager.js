/**
 * # Toast Manager
 * 
 * จัดการ Toast Notifications
 * - Non-blocking notifications
 * - Auto-dismiss or click to dismiss
 * - Support different types: info, success, warning, error
 * 
 * @example
 * toastManager.show('สวัสดีค่ะ!', 'info');
 * toastManager.success('บันทึกสำเร็จ');
 * toastManager.error('เกิดข้อผิดพลาด');
 */

class ToastManager {
    constructor() {
        this.container = null;
        this.defaultDuration = 5000; // 5 seconds
    }

    /**
     * Get or create container
     * @private
     */
    _getContainer() {
        if (!this.container) {
            this.container = document.getElementById('toast-container');

            // Create container if not exists
            if (!this.container) {
                this.container = document.createElement('div');
                this.container.id = 'toast-container';
                this.container.style.cssText = `
                    position: fixed;
                    top: 80px;
                    right: 20px;
                    z-index: 9999;
                    display: flex;
                    flex-direction: column;
                    gap: 10px;
                    max-width: 350px;
                `;
                document.body.appendChild(this.container);
            }
        }
        return this.container;
    }

    /**
     * แสดง Toast
     * @param {string} message - ข้อความ
     * @param {string} type - ประเภท: info, success, warning, error
     * @param {Object} options - ตัวเลือกเพิ่มเติม
     */
    show(message, type = 'info', options = {}) {
        const {
            duration = this.defaultDuration,
            icon = this._getIcon(type),
            title = null,
            onClick = null,
            closable = true
        } = options;

        const container = this._getContainer();

        // Create toast element
        const toast = document.createElement('div');
        toast.className = `toast toast-${type}`;
        toast.style.cssText = `
            display: flex;
            align-items: flex-start;
            gap: 12px;
            padding: 14px 16px;
            background: ${this._getBackground(type)};
            border: 1px solid ${this._getBorderColor(type)};
            border-radius: 12px;
            color: white;
            box-shadow: 0 4px 20px rgba(0, 0, 0, 0.3);
            animation: toast-slide-in 0.3s ease-out;
            cursor: ${onClick ? 'pointer' : 'default'};
            transition: transform 0.2s, opacity 0.3s;
        `;

        // Build HTML
        toast.innerHTML = `
            <div style="font-size: 1.3rem; flex-shrink: 0;">${icon}</div>
            <div style="flex: 1; min-width: 0;">
                ${title ? `<div style="font-weight: 600; margin-bottom: 4px;">${title}</div>` : ''}
                <div style="font-size: 0.9rem; opacity: 0.95; word-wrap: break-word;">${message}</div>
            </div>
            ${closable ? `<button class="toast-close" style="
                background: none;
                border: none;
                color: white;
                opacity: 0.6;
                cursor: pointer;
                font-size: 1rem;
                padding: 0;
                line-height: 1;
            ">✕</button>` : ''}
        `;

        // Add click handlers
        if (onClick) {
            toast.addEventListener('click', (e) => {
                if (!e.target.classList.contains('toast-close')) {
                    onClick();
                    this._removeToast(toast);
                }
            });
        }

        // Close button
        const closeBtn = toast.querySelector('.toast-close');
        if (closeBtn) {
            closeBtn.addEventListener('click', (e) => {
                e.stopPropagation();
                this._removeToast(toast);
            });
        }

        // Hover effect
        toast.addEventListener('mouseover', () => {
            toast.style.transform = 'scale(1.02)';
        });
        toast.addEventListener('mouseout', () => {
            toast.style.transform = 'scale(1)';
        });

        // Add to container
        container.appendChild(toast);

        // Auto-dismiss
        if (duration > 0) {
            setTimeout(() => this._removeToast(toast), duration);
        }

        return toast;
    }

    /**
     * Shortcut methods
     */
    info(message, options = {}) {
        return this.show(message, 'info', options);
    }

    success(message, options = {}) {
        return this.show(message, 'success', options);
    }

    warning(message, options = {}) {
        return this.show(message, 'warning', options);
    }

    error(message, options = {}) {
        return this.show(message, 'error', { duration: 8000, ...options });
    }

    /**
     * Alert toast (clickable to show details)
     */
    alert(alertData, onClick) {
        return this.show(alertData.summary || alertData.message, 'warning', {
            title: '⚠️ แจ้งเตือน',
            icon: '🚨',
            duration: 15000,
            onClick: onClick
        });
    }

    /**
     * Remove toast with animation
     * @private
     */
    _removeToast(toast) {
        if (!toast || !toast.parentElement) return;

        toast.style.opacity = '0';
        toast.style.transform = 'translateX(100%)';
        setTimeout(() => toast.remove(), 300);
    }

    /**
     * Get icon for type
     * @private
     */
    _getIcon(type) {
        const icons = {
            info: 'ℹ️',
            success: '✅',
            warning: '⚠️',
            error: '❌'
        };
        return icons[type] || icons.info;
    }

    /**
     * Get background color for type
     * @private
     */
    _getBackground(type) {
        const colors = {
            info: 'rgba(59, 130, 246, 0.95)',
            success: 'rgba(34, 197, 94, 0.95)',
            warning: 'rgba(234, 179, 8, 0.95)',
            error: 'rgba(239, 68, 68, 0.95)'
        };
        return colors[type] || colors.info;
    }

    /**
     * Get border color for type
     * @private
     */
    _getBorderColor(type) {
        const colors = {
            info: 'rgba(96, 165, 250, 0.5)',
            success: 'rgba(74, 222, 128, 0.5)',
            warning: 'rgba(250, 204, 21, 0.5)',
            error: 'rgba(248, 113, 113, 0.5)'
        };
        return colors[type] || colors.info;
    }

    /**
     * Clear all toasts
     */
    clearAll() {
        const container = this._getContainer();
        container.innerHTML = '';
    }
}

// Add CSS animation
const style = document.createElement('style');
style.textContent = `
    @keyframes toast-slide-in {
        from {
            opacity: 0;
            transform: translateX(100%);
        }
        to {
            opacity: 1;
            transform: translateX(0);
        }
    }
`;
document.head.appendChild(style);

// Singleton
export const toastManager = new ToastManager();
export default toastManager;
