// src/v2/components/garment/GarmentVisualizer.tsx

import * as THREE from 'three';
import type { ThreeEvent } from '@react-three/fiber';

interface GarmentVisualizerProps {
    visualRef: React.RefObject<THREE.Mesh>;
    proxyRef: React.RefObject<THREE.Mesh>;
    visualGeometry: THREE.BufferGeometry;
    physicsGeometry: THREE.BufferGeometry;
    showProxy: boolean;
    visualColor: string;
    visualRoughness: number;
    visualMetalness: number;
    onPointerDown: (e: ThreeEvent<PointerEvent>) => void;
    onPointerUp: (e: ThreeEvent<PointerEvent> | THREE.Event) => void;
}

/**
 * Pure rendering component for the garment
 * Separates visual presentation from physics logic
 */
export const GarmentVisualizer = ({
    visualRef,
    proxyRef,
    visualGeometry,
    physicsGeometry,
    showProxy,
    visualColor,
    visualRoughness,
    visualMetalness,
    onPointerDown,
    onPointerUp,
}: GarmentVisualizerProps) => {
    return (
        <group>
            {/* HIGH-QUALITY VISUAL MESH */}
            <mesh
                ref={visualRef}
                geometry={visualGeometry}
                castShadow
                receiveShadow
                onPointerDown={onPointerDown}
                onPointerUp={onPointerUp}
                onPointerLeave={onPointerUp}
            >
                <meshStandardMaterial
                    color={visualColor}
                    side={THREE.DoubleSide}
                    roughness={visualRoughness}
                    metalness={visualMetalness}
                    flatShading={false}
                />
            </mesh>

            {/* LOW-POLY PHYSICS PROXY (Debug Visualization) */}
            <mesh
                ref={proxyRef}
                geometry={physicsGeometry}
                visible={showProxy}
            >
                <meshBasicMaterial
                    color="yellow"
                    wireframe={true}
                    opacity={0.6}
                    transparent={true}
                    depthTest={true}
                    depthWrite={false}
                />
            </mesh>
        </group>
    );
};