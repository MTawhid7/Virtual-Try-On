// src/v3/AppV3.tsx
import { Canvas } from '@react-three/fiber';
import { SceneV3 } from './presentation/SceneV3';

export const AppV3 = () => {
    return (
        <div style={{ width: '100vw', height: '100vh', background: '#e0e0e0' }}>
            <Canvas
                shadows
                camera={{ position: [0, 1.2, 2.5], fov: 45 }}
                gl={{ antialias: true }}
            >
                <SceneV3 />
            </Canvas>
        </div>
    );
};