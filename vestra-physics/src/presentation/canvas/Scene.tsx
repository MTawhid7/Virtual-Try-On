// src/presentation/canvas/Scene.tsx

import React, { useEffect } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import { useSimulationStore } from '../state/useSimulationStore';
import { Lights } from './Lights';
import { CameraRig } from './CameraRig';
import { MannequinMesh } from './MannequinMesh';
import { GarmentMesh } from './GarmentMesh';
import { FpsTracker } from './FpsTracker';

/**
 * The main 3D scene container.
 * Sets up the Canvas, Lights, Camera, and renders the simulation objects (Mannequin, Garment).
 * Also handles initial data loading via the SimulationStore.
 */
export const Scene: React.FC = () => {
    const { loadAndInitialize, isReady, isInteracting } = useSimulationStore();

    // Trigger initial load on mount
    useEffect(() => {
        loadAndInitialize();
    }, [loadAndInitialize]);

    return (
        <Canvas
            shadows="soft"
            camera={{ position: [0, 1.5, 2.5], fov: 50 }}
            dpr={[1, 2]} // Handle High-DPI screens
        >
            <FpsTracker />
            <color attach="background" args={['#1a1a1a']} />

            <Lights />
            <CameraRig />

            <OrbitControls
                makeDefault
                enabled={!isInteracting} // Disable rotation while dragging cloth
                minDistance={1.0}
                maxDistance={5.0}
                target={[0, 1, 0]}
            />

            {isReady && (
                <group>
                    <MannequinMesh />
                    <GarmentMesh />
                </group>
            )}

            <Grid
                args={[10, 10]}
                cellColor="#444"
                sectionColor="#666"
                fadeDistance={20}
            />
        </Canvas>
    );
};