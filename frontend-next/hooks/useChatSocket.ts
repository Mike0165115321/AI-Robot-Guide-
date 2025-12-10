// /hooks/useChatSocket.ts
// WebSocket Hook สำหรับ Chat System

'use client';

import { useState, useEffect, useCallback, useRef } from 'react';
import { WS_BASE_URL } from '@/lib/config';
import type { ChatResponse } from '@/types';

interface UseChatSocketOptions {
    onMessage?: (data: ChatResponse) => void;
    onConnect?: () => void;
    onDisconnect?: () => void;
    onError?: (error: Event) => void;
    autoConnect?: boolean;
}

interface UseChatSocketReturn {
    isConnected: boolean;
    isConnecting: boolean;
    sendMessage: (message: string) => void;
    connect: () => void;
    disconnect: () => void;
}

export function useChatSocket(options: UseChatSocketOptions = {}): UseChatSocketReturn {
    const {
        onMessage,
        onConnect,
        onDisconnect,
        onError,
        autoConnect = true,
    } = options;

    const [isConnected, setIsConnected] = useState(false);
    const [isConnecting, setIsConnecting] = useState(false);
    const socketRef = useRef<WebSocket | null>(null);
    const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);

    const connect = useCallback(() => {
        if (socketRef.current?.readyState === WebSocket.OPEN) {
            return;
        }

        setIsConnecting(true);

        try {
            const ws = new WebSocket(`${WS_BASE_URL}/ws/chat`);

            ws.onopen = () => {
                console.log('[WebSocket] Connected');
                setIsConnected(true);
                setIsConnecting(false);
                onConnect?.();
            };

            ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data) as ChatResponse;
                    onMessage?.(data);
                } catch (e) {
                    console.error('[WebSocket] Failed to parse message:', e);
                }
            };

            ws.onclose = (event) => {
                console.log('[WebSocket] Disconnected', event.code);
                setIsConnected(false);
                setIsConnecting(false);
                onDisconnect?.();

                // Auto-reconnect after 3 seconds if not intentionally closed
                if (event.code !== 1000) {
                    reconnectTimeoutRef.current = setTimeout(() => {
                        console.log('[WebSocket] Attempting to reconnect...');
                        connect();
                    }, 3000);
                }
            };

            ws.onerror = (error) => {
                console.error('[WebSocket] Error:', error);
                setIsConnecting(false);
                onError?.(error);
            };

            socketRef.current = ws;
        } catch (e) {
            console.error('[WebSocket] Failed to create connection:', e);
            setIsConnecting(false);
        }
    }, [onMessage, onConnect, onDisconnect, onError]);

    const disconnect = useCallback(() => {
        if (reconnectTimeoutRef.current) {
            clearTimeout(reconnectTimeoutRef.current);
        }

        if (socketRef.current) {
            socketRef.current.close(1000, 'User disconnected');
            socketRef.current = null;
        }

        setIsConnected(false);
    }, []);

    const sendMessage = useCallback((message: string) => {
        if (socketRef.current?.readyState === WebSocket.OPEN) {
            socketRef.current.send(JSON.stringify({ message }));
        } else {
            console.warn('[WebSocket] Cannot send - not connected');
        }
    }, []);

    // Auto-connect on mount
    useEffect(() => {
        if (autoConnect) {
            connect();
        }

        return () => {
            disconnect();
        };
    }, [autoConnect, connect, disconnect]);

    return {
        isConnected,
        isConnecting,
        sendMessage,
        connect,
        disconnect,
    };
}

export default useChatSocket;
