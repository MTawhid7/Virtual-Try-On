// src/v3/presentation/SceneV3.tsx
import { OrbitControls, Environment, ContactShadows } from '@react-three/drei';
import { VirtualTryOn } from './components/VirtualTryOn';
// DebugOverlay is now integrated into VirtualTryOn or passed down
// Since VirtualTryOn owns the engine, we should render DebugOverlay THERE.

export const SceneV3 = () => {
    return (
        <>
            <OrbitControls target={[0, 1.0, 0]} makeDefault />
            <ambientLight intensity={0.5} />
            <directionalLight position={[5, 5, 5]} intensity={1} castShadow />
            <Environment preset="city" />
            <ContactShadows resolution={1024} scale={10} blur={1} opacity={0.5} far={1} color="#000000" />

            <VirtualTryOn />
        </>
    );
};