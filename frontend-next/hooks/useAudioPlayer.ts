// /hooks/useAudioPlayer.ts
// Audio Player Hook for TTS

'use client';

import { useState, useCallback, useRef, useEffect } from 'react';

interface UseAudioPlayerOptions {
    onPlay?: () => void;
    onPause?: () => void;
    onEnded?: () => void;
    onError?: (error: string) => void;
}

interface UseAudioPlayerReturn {
    isPlaying: boolean;
    isLoading: boolean;
    currentTime: number;
    duration: number;
    playAudio: (src: string | Blob) => void;
    playBase64: (base64Data: string, mimeType?: string) => void;
    pause: () => void;
    resume: () => void;
    stop: () => void;
    setVolume: (volume: number) => void;
}

export function useAudioPlayer(
    options: UseAudioPlayerOptions = {}
): UseAudioPlayerReturn {
    const { onPlay, onPause, onEnded, onError } = options;

    const [isPlaying, setIsPlaying] = useState(false);
    const [isLoading, setIsLoading] = useState(false);
    const [currentTime, setCurrentTime] = useState(0);
    const [duration, setDuration] = useState(0);

    const audioRef = useRef<HTMLAudioElement | null>(null);
    const animationRef = useRef<number | null>(null);

    // Initialize audio element
    useEffect(() => {
        const audio = new Audio();

        audio.onplay = () => {
            setIsPlaying(true);
            onPlay?.();
            updateProgress();
        };

        audio.onpause = () => {
            setIsPlaying(false);
            onPause?.();
            if (animationRef.current) {
                cancelAnimationFrame(animationRef.current);
            }
        };

        audio.onended = () => {
            setIsPlaying(false);
            setCurrentTime(0);
            onEnded?.();
            if (animationRef.current) {
                cancelAnimationFrame(animationRef.current);
            }
        };

        audio.onerror = () => {
            setIsLoading(false);
            setIsPlaying(false);
            onError?.('Failed to load audio');
        };

        audio.onloadedmetadata = () => {
            setDuration(audio.duration);
            setIsLoading(false);
        };

        audio.oncanplaythrough = () => {
            setIsLoading(false);
        };

        audioRef.current = audio;

        return () => {
            audio.pause();
            audio.src = '';
            if (animationRef.current) {
                cancelAnimationFrame(animationRef.current);
            }
        };
    }, [onPlay, onPause, onEnded, onError]);

    const updateProgress = useCallback(() => {
        if (audioRef.current) {
            setCurrentTime(audioRef.current.currentTime);
            animationRef.current = requestAnimationFrame(updateProgress);
        }
    }, []);

    const playAudio = useCallback((src: string | Blob) => {
        if (!audioRef.current) return;

        setIsLoading(true);

        if (src instanceof Blob) {
            audioRef.current.src = URL.createObjectURL(src);
        } else {
            audioRef.current.src = src;
        }

        audioRef.current.play().catch((e) => {
            console.error('[Audio] Play failed:', e);
            setIsLoading(false);
            onError?.('Playback failed');
        });
    }, [onError]);

    const playBase64 = useCallback((base64Data: string, mimeType = 'audio/wav') => {
        if (!audioRef.current) return;

        setIsLoading(true);

        try {
            const byteCharacters = atob(base64Data);
            const byteNumbers = new Array(byteCharacters.length);

            for (let i = 0; i < byteCharacters.length; i++) {
                byteNumbers[i] = byteCharacters.charCodeAt(i);
            }

            const byteArray = new Uint8Array(byteNumbers);
            const blob = new Blob([byteArray], { type: mimeType });

            playAudio(blob);
        } catch (e) {
            console.error('[Audio] Base64 decode failed:', e);
            setIsLoading(false);
            onError?.('Invalid audio data');
        }
    }, [playAudio, onError]);

    const pause = useCallback(() => {
        audioRef.current?.pause();
    }, []);

    const resume = useCallback(() => {
        audioRef.current?.play();
    }, []);

    const stop = useCallback(() => {
        if (audioRef.current) {
            audioRef.current.pause();
            audioRef.current.currentTime = 0;
            setCurrentTime(0);
            setIsPlaying(false);
        }
    }, []);

    const setVolume = useCallback((volume: number) => {
        if (audioRef.current) {
            audioRef.current.volume = Math.max(0, Math.min(1, volume));
        }
    }, []);

    return {
        isPlaying,
        isLoading,
        currentTime,
        duration,
        playAudio,
        playBase64,
        pause,
        resume,
        stop,
        setVolume,
    };
}

export default useAudioPlayer;
