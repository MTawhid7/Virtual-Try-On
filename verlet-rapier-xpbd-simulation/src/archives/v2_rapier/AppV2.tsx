// src/v2/AppV2.tsx

import { Canvas } from '@react-three/fiber';
import { Physics } from '@react-three/rapier';
import { Scene } from './components/canvas/Scene';
import { Suspense } from 'react';
import { PHYSICS_CONFIG } from './config/physics.config';

export const AppV2 = () => {
    return (
        <div style={{ width: '100vw', height: '100vh', background: '#f0f0f0' }}>
            <Canvas
                shadows
                camera={{ position: [0, 1.5, 4], fov: 45 }}
                gl={{
                    antialias: true,
                    alpha: false,
                    powerPreference: 'high-performance',
                }}
            >
                <Suspense fallback={null}>
                    <Physics
                        gravity={PHYSICS_CONFIG.gravity}
                        timeStep={PHYSICS_CONFIG.timeStep}
                        numSolverIterations={PHYSICS_CONFIG.solverIterations}

                    >
                        <Scene />
                    </Physics>
                </Suspense>
            </Canvas>
        </div>
    );
};