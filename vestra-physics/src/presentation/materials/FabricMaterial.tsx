// src/presentation/materials/FabricMateria.tsx

import React, { useMemo } from 'react';
import * as THREE from 'three';
import { TextureGenerator } from '../../infrastructure/rendering/TextureGenerator';

/**
 * Props for the FabricMaterial.
 */
interface FabricMaterialProps {
    /** The base color of the fabric. Defaults to Blue (#4488ff). */
    color?: string;
}

/**
 * A specialized PBR material that simulates the look of cotton fabric.
 * Uses a procedurally generated normal map to create a realistic weave pattern.
 */
export const FabricMaterial: React.FC<FabricMaterialProps> = ({ color = "#4488ff" }) => {
    // Generate the weave texture once on mount.
    // We use useMemo to prevent regenerating the canvas on every render.
    const normalMap = useMemo(() => {
        return TextureGenerator.generateFabricNormal();
    }, []);

    return (
        <meshStandardMaterial
            color={color}

            // 1. TEXTURE (The Weave)
            // The normal map adds the tactile "bump" of the threads.
            normalMap={normalMap}
            normalScale={new THREE.Vector2(0.4, 0.4)} // 0.4 keeps it subtle; 1.0 would look like burlap.

            // 2. SURFACE PROPERTIES
            roughness={0.8} // High roughness = Matte finish (Cotton/Wool)
            metalness={0.0} // No metallic reflection

            // 3. VISIBILITY
            // Render both sides of the faces so the inside of the shirt is visible.
            side={THREE.DoubleSide}

        // Note: We do NOT set shadowSide={DoubleSide}.
        // Leaving it default allows light to pass through the "back" of the front face,
        // effectively illuminating the inside of the shirt, which looks more natural
        // than a pitch-black interior.
        />
    );
};