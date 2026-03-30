// src/presentation/App.tsx

import React from 'react';
import { Scene } from './canvas/Scene';
import { Overlay } from './components/overlay/Overlay';
import { LoadingScreen } from './components/LoadingScreen';

export const App: React.FC = () => {
    return (
        <div style={{
            width: '100vw',
            height: '100vh',
            position: 'relative',
            background: '#1a1a1a',
            overflow: 'hidden'
        }}>
            {/* The 3D Scene */}
            <Scene />

            {/* The UI Overlay (Controls, Status) */}
            <Overlay />

            {/* The Loading Screen (Blocks UI until assets are ready) */}
            <LoadingScreen />
        </div>
    );
};