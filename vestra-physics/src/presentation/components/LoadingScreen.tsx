// src/presentation/components/LoadingScreen.tsx

import React from 'react';
import { useSimulationStore } from '../state/useSimulationStore';

/**
 * A full-screen overlay displayed during the initial application load.
 * - Shows while `isLoading` is true AND `isReady` is false.
 * - Once the simulation is ready for the first time, this screen disappears.
 * - Subsequent loading states (e.g. changing sizes) use the `StatusPanel` instead.
 */
export const LoadingScreen: React.FC = () => {
    const { isLoading, isReady } = useSimulationStore();

    // Only show if loading and NOT ready (initial load)
    // If we are resizing (ready = true, but loading = true), we rely on the StatusPanel instead.
    if (!isLoading || isReady) return null;

    return (
        <div style={{
            position: 'absolute',
            top: 0,
            left: 0,
            width: '100vw',
            height: '100vh',
            background: '#1a1a1a',
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'center',
            color: 'white',
            fontFamily: 'monospace',
            zIndex: 50
        }}>
            <div style={{
                width: '40px',
                height: '40px',
                border: '4px solid #333',
                borderTop: '4px solid #4488ff',
                borderRadius: '50%',
                animation: 'spin 1s linear infinite',
                marginBottom: '20px'
            }} />
            <style>{`
                @keyframes spin {
                    0% { transform: rotate(0deg); }
                    100% { transform: rotate(360deg); }
                }
            `}</style>
            <h2>Initializing Vestra Physics Engine</h2>
            <p style={{ color: '#666' }}>Loading Assets & Compiling WASM...</p>
        </div>
    );
};