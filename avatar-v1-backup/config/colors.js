/**
 * config/colors.js - Design System Color Tokens
 * Premium palette for Nan Robot V2.0 (Enhanced)
 */

export const AvatarColors = {
    // Primary Brand Colors (Nan Robot Blue/Cyan)
    primary: {
        base: 'hsl(198, 100%, 63%)',    // Vivid Sky Blue
        glow: 'hsla(198, 100%, 63%, 0.5)',
        dark: 'hsl(198, 100%, 40%)'
    },

    // Mood-specific Palette
    moods: {
        normal: {
            eye: '#40c4ff',
            accent: '#00e5ff',
            glow: 'rgba(64, 196, 255, 0.5)'
        },
        speaking: {
            eye: '#4ade80',
            accent: '#22c55e',
            glow: 'rgba(74, 222, 128, 0.6)'
        },
        thinking: {
            eye: '#fbbf24',
            accent: '#f59e0b',
            glow: 'rgba(251, 191, 36, 0.5)'
        },
        listening: {
            eye: '#f472b6',
            accent: '#ec4899',
            glow: 'rgba(244, 114, 182, 0.5)'
        },
        happy: {
            eye: '#34d399',
            accent: '#10b981',
            glow: 'rgba(52, 211, 153, 0.6)'
        },
        curious: {
            eye: '#a78bfa',
            accent: '#8b5cf6',
            glow: 'rgba(167, 139, 250, 0.5)'
        },
        sleepy: {
            eye: '#64748b',
            accent: '#475569',
            glow: 'rgba(100, 116, 139, 0.3)'
        },
        error: {
            eye: '#ff5252',
            accent: '#d32f2f',
            glow: 'rgba(255, 82, 82, 0.5)'
        }
    },

    // UI & Shell
    shell: {
        background: 'radial-gradient(circle at 50% 50%, #1a2a3a 0%, #0a0f14 100%)',
        glass: 'rgba(255, 255, 255, 0.05)',
        border: 'rgba(255, 255, 255, 0.1)'
    }
};
