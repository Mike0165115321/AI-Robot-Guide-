// /lib/utils.ts
// Utility functions

import { type ClassValue, clsx } from 'clsx';

/**
 * Merge class names with Tailwind CSS
 */
export function cn(...inputs: ClassValue[]) {
    return clsx(inputs);
}

/**
 * สร้าง slug จาก title
 */
export function generateSlug(title: string): string {
    return title
        .toLowerCase()
        .trim()
        .replace(/\s+/g, '-')
        .replace(/[^\u0E00-\u0E7Fa-z0-9-]/g, '') // Keep Thai and English
        .replace(/-+/g, '-')
        .replace(/^-|-$/g, '');
}

/**
 * สร้าง Image URL จาก slug และ prefix
 */
export function getImageUrl(slug: string, prefix?: string): string {
    if (!prefix) return '';

    // Clean up the slug
    const cleanSlug = slug
        .replace(/\s+/g, '-')
        .replace(/-+$/, '');

    return `/api/images/${cleanSlug}/${prefix}1.jpg`;
}

/**
 * Format date สำหรับภาษาไทย
 */
export function formatThaiDate(date: Date | string): string {
    const d = typeof date === 'string' ? new Date(date) : date;
    return d.toLocaleDateString('th-TH', {
        year: 'numeric',
        month: 'long',
        day: 'numeric',
        hour: '2-digit',
        minute: '2-digit',
    });
}

/**
 * Animate number counting up
 */
export function animateValue(
    element: HTMLElement,
    start: number,
    end: number,
    duration: number
): void {
    const startTime = performance.now();

    function step(currentTime: number) {
        const elapsed = currentTime - startTime;
        const progress = Math.min(elapsed / duration, 1);

        const current = Math.floor(progress * (end - start) + start);
        element.textContent = current.toLocaleString();

        if (progress < 1) {
            requestAnimationFrame(step);
        }
    }

    requestAnimationFrame(step);
}

/**
 * Debounce function
 */
export function debounce<T extends (...args: unknown[]) => unknown>(
    func: T,
    wait: number
): (...args: Parameters<T>) => void {
    let timeout: NodeJS.Timeout | null = null;

    return (...args: Parameters<T>) => {
        if (timeout) clearTimeout(timeout);
        timeout = setTimeout(() => func(...args), wait);
    };
}

/**
 * Sleep function for async/await
 */
export function sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
}
