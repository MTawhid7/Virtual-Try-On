// src/presentation/canvas/MannequinMesh.tsx

import React from 'react';
import { useSimulationStore } from '../state/useSimulationStore';

/**
 * Renders the high-resolution visual mesh of the mannequin.
 * This mesh is purely for display and shadows; the physics engine uses a separate low-poly collider.
 */
export const MannequinMesh: React.FC = () => {
    const { assets } = useSimulationStore();

    // Use the High-Res Visual Body (not the low-poly collider)
    const geometry = assets?.visualBody;

    if (!geometry) return null;

    return (
        <mesh geometry={geometry} receiveShadow castShadow>
            <meshStandardMaterial
                color="#f5f5f5"      // Off-white / Clay look
                roughness={0.6}      // Matte but smooth
                metalness={0.1}      // Slight specular highlight
            />
        </mesh>
    );
};