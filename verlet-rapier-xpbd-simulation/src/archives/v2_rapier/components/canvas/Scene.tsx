import { OrbitControls, Environment, ContactShadows } from '@react-three/drei';
import { Mannequin } from '../avatar/Mannequin';
import { Shirt } from '../garment/Shirt'; // Import Shirt

export const Scene = () => {
    return (
        <>
            <OrbitControls target={[0, 1, 0]} makeDefault />

            <ambientLight intensity={0.5} />
            <directionalLight
                position={[5, 10, 5]}
                intensity={1}
                castShadow
                shadow-mapSize={[1024, 1024]}
            />
            <Environment preset="city" />
            <ContactShadows resolution={1024} scale={10} blur={1} opacity={0.5} far={1} color="#8a8a8a" />

            <Mannequin />
            <Shirt /> {/* Replaced <Cloth /> */}
        </>
    );
};