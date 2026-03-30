// src/presentation/canvas/FpsTracker.tsx

import React, { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { useSimulationStore } from '../state/useSimulationStore';

/**
 * Invisible component that monitors the render loop frequency (FPS).
 * Updates the global SimulationStore once per second with the calculated FPS.
 */
export const FpsTracker: React.FC = () => {
    const setFps = useSimulationStore(state => state.setFps);
    const frameCount = useRef(0);
    const lastTime = useRef(performance.now());

    useFrame(() => {
        frameCount.current++;
        const currentTime = performance.now();

        // Update FPS every 1 second
        if (currentTime >= lastTime.current + 1000) {
            const fps = Math.round((frameCount.current * 1000) / (currentTime - lastTime.current));
            setFps(fps);

            frameCount.current = 0;
            lastTime.current = currentTime;
        }
    });

    return null; // This component doesn't render anything to the 3D scene
};
