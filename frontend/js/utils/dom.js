/**
 * # DOM Utilities - Helper functions สำหรับ DOM
 */

/**
 * # เลือก element เดียว
 * @param {string} selector - CSS selector
 * @param {Element} parent - Parent element (optional)
 * @returns {Element|null}
 */
export function $(selector, parent = document) {
    return parent.querySelector(selector);
}

/**
 * # เลือกหลาย elements
 * @param {string} selector - CSS selector
 * @param {Element} parent - Parent element (optional)
 * @returns {NodeList}
 */
export function $$(selector, parent = document) {
    return parent.querySelectorAll(selector);
}

/**
 * # สร้าง element ใหม่
 * @param {string} tag - HTML tag
 * @param {Object} options - { classes, attrs, text, html, children }
 * @returns {Element}
 */
export function createElement(tag, options = {}) {
    const el = document.createElement(tag);

    if (options.classes) {
        el.className = Array.isArray(options.classes)
            ? options.classes.join(' ')
            : options.classes;
    }

    if (options.attrs) {
        Object.entries(options.attrs).forEach(([key, value]) => {
            el.setAttribute(key, value);
        });
    }

    if (options.text) {
        el.textContent = options.text;
    }

    if (options.html) {
        el.innerHTML = options.html;
    }

    if (options.children) {
        options.children.forEach(child => el.appendChild(child));
    }

    return el;
}

/**
 * # เพิ่ม Event Listener
 * @param {Element} el - Element
 * @param {string} event - Event name
 * @param {Function} handler - Handler function
 * @param {Object} options - Options
 */
export function on(el, event, handler, options = {}) {
    el.addEventListener(event, handler, options);
}

/**
 * # ลบ Event Listener
 */
export function off(el, event, handler) {
    el.removeEventListener(event, handler);
}

/**
 * # Delegate event
 * @param {Element} parent - Parent element
 * @param {string} event - Event name
 * @param {string} selector - Child selector
 * @param {Function} handler - Handler function
 */
export function delegate(parent, event, selector, handler) {
    parent.addEventListener(event, (e) => {
        const target = e.target.closest(selector);
        if (target && parent.contains(target)) {
            handler(e, target);
        }
    });
}

/**
 * # Show element
 */
export function show(el) {
    el.classList.remove('hidden');
}

/**
 * # Hide element
 */
export function hide(el) {
    el.classList.add('hidden');
}

/**
 * # Toggle class
 */
export function toggle(el, className) {
    el.classList.toggle(className);
}
