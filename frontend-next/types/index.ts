// /types/index.ts
// TypeScript interfaces สำหรับ project

// ==================== Location Types ====================
export interface Location {
    _id?: string;
    slug: string;
    title: string;
    summary?: string;
    category?: string;
    topic?: string;
    keywords?: string[];
    details?: LocationDetail[];
    image_prefix?: string;
    source?: string;
    from_sheets?: boolean;
    location_data?: LocationCoordinates;
}

export interface LocationDetail {
    heading: string;
    content: string;
}

export interface LocationCoordinates {
    lat?: number;
    lng?: number;
    address?: string;
}

// ==================== Chat Types ====================
export interface ChatMessage {
    id: string;
    role: 'user' | 'assistant';
    content: string;
    timestamp: Date;
    imageUrl?: string;
    imageGallery?: string[];
    emotion?: string;
    sources?: ChatSource[];
    suggestedQuestions?: string[];
}

export interface ChatSource {
    title: string;
    slug: string;
}

export interface ChatResponse {
    response: string;
    emotion?: 'normal' | 'happy' | 'thinking' | 'excited' | 'confused';
    image_url?: string;
    image_gallery?: string[];
    action?: string;
    action_payload?: Record<string, unknown>;
    sources?: ChatSource[];
    suggested_questions?: string[];
    audio_data?: string;
}

// ==================== Analytics Types ====================
export interface AnalyticsSummary {
    total_conversations: number;
    top_locations: TopLocation[];
    user_origins: UserOrigin[];
    user_provinces: UserProvince[];
    interests: UserInterest[];
}

export interface TopLocation {
    name: string;
    count: number;
}

export interface UserOrigin {
    origin: string;
    count: number;
}

export interface UserProvince {
    province: string;
    count: number;
}

export interface UserInterest {
    interest: string;
    count: number;
}

// ==================== Music Types ====================
export interface Song {
    title: string;
    videoId: string;
    thumbnail?: string;
}

// ==================== UI State Types ====================
export interface ModalState {
    isOpen: boolean;
    data?: unknown;
}

export interface PaginationState {
    currentPage: number;
    totalItems: number;
    itemsPerPage: number;
}

// ==================== API Response Types ====================
export interface ApiResponse<T> {
    success: boolean;
    data?: T;
    error?: string;
}

export interface PaginatedResponse<T> {
    items: T[];
    total: number;
    page: number;
    limit: number;
    hasMore: boolean;
}
