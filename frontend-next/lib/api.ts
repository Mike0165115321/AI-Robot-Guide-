// /lib/api.ts
// API Client สำหรับเรียก FastAPI Backend

import { API_BASE_URL } from './config';

/**
 * Generic fetch wrapper with error handling
 */
async function fetchAPI<T>(
    endpoint: string,
    options: RequestInit = {}
): Promise<T> {
    const url = `${API_BASE_URL}${endpoint}`;

    const defaultHeaders: HeadersInit = {
        'Content-Type': 'application/json',
    };

    const config: RequestInit = {
        ...options,
        headers: {
            ...defaultHeaders,
            ...options.headers,
        },
    };

    const response = await fetch(url, config);

    if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `API Error: ${response.status}`);
    }

    return response.json();
}

// ==================== Location APIs ====================

export interface Location {
    _id?: string;
    slug: string;
    title: string;
    summary?: string;
    category?: string;
    topic?: string;
    keywords?: string[];
    details?: Array<{ heading: string; content: string }>;
    image_prefix?: string;
    source?: string;
    from_sheets?: boolean;
}

export interface PaginatedLocations {
    locations: Location[];
    total: number;
    page: number;
    limit: number;
}

export const locationApi = {
    // Get all locations with pagination
    getAll: (page = 1, limit = 10) =>
        fetchAPI<PaginatedLocations>(`/locations/?page=${page}&limit=${limit}`),

    // Get single location by slug
    getBySlug: (slug: string) =>
        fetchAPI<Location>(`/locations/${slug}`),

    // Create new location
    create: (data: Omit<Location, '_id'>) =>
        fetchAPI<Location>('/locations/', {
            method: 'POST',
            body: JSON.stringify(data),
        }),

    // Update location
    update: (slug: string, data: Partial<Location>) =>
        fetchAPI<Location>(`/locations/${slug}`, {
            method: 'PUT',
            body: JSON.stringify(data),
        }),

    // Delete location
    delete: (slug: string) =>
        fetchAPI<{ message: string }>(`/locations/${slug}`, {
            method: 'DELETE',
        }),

    // Get available fields
    getFields: () =>
        fetchAPI<{ fields: string[] }>('/locations/fields'),
};

// ==================== Chat APIs ====================

export interface ChatMessage {
    role: 'user' | 'assistant';
    content: string;
}

export interface ChatResponse {
    response: string;
    emotion?: string;
    image_url?: string;
    image_gallery?: string[];
    action?: string;
    action_payload?: Record<string, unknown>;
    sources?: Array<{ title: string; slug: string }>;
    suggested_questions?: string[];
    audio_data?: string;
}

export const chatApi = {
    // Send chat message (REST fallback)
    send: (message: string, conversation_id?: string) =>
        fetchAPI<ChatResponse>('/chat/', {
            method: 'POST',
            body: JSON.stringify({ message, conversation_id }),
        }),
};

// ==================== Analytics APIs ====================

export interface AnalyticsSummary {
    total_conversations: number;
    top_locations: Array<{ name: string; count: number }>;
    user_origins: Array<{ origin: string; count: number }>;
    user_provinces: Array<{ province: string; count: number }>;
    interests: Array<{ interest: string; count: number }>;
}

export const analyticsApi = {
    getSummary: () =>
        fetchAPI<AnalyticsSummary>('/analytics/summary'),
};

// ==================== Google Sheets APIs ====================

export interface SheetsStatus {
    connected: boolean;
    sheet_title?: string;
    last_sync?: string;
    mode?: string;
    row_count?: number;
}

export interface SheetsConnectResponse {
    success: boolean;
    message: string;
    sheet_title?: string;
    row_count?: number;
}

export interface SheetsSyncResponse {
    success: boolean;
    message: string;
    stats?: {
        created: number;
        updated: number;
        unchanged: number;
    };
}

export const sheetsApi = {
    getStatus: () =>
        fetchAPI<SheetsStatus>('/sheets/status'),

    connect: (sheetUrl: string) =>
        fetchAPI<SheetsConnectResponse>('/sheets/connect', {
            method: 'POST',
            body: JSON.stringify({ sheet_url: sheetUrl }),
        }),

    sync: () =>
        fetchAPI<SheetsSyncResponse>('/sheets/sync', {
            method: 'POST',
        }),

    disconnect: () =>
        fetchAPI<{ success: boolean }>('/sheets/disconnect', {
            method: 'POST',
        }),
};

// ==================== Music APIs ====================

export interface Song {
    title: string;
    videoId: string;
    thumbnail?: string;
}

export interface MusicSearchResponse {
    songs: Song[];
}

export const musicApi = {
    search: (query: string) =>
        fetchAPI<MusicSearchResponse>(`/music/search?q=${encodeURIComponent(query)}`),
};

export default {
    location: locationApi,
    chat: chatApi,
    analytics: analyticsApi,
    sheets: sheetsApi,
    music: musicApi,
};
