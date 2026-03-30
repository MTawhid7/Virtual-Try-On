// src/presentation/canvas/GarmentMesh.tsx

import React, { useRef, useMemo, useEffect } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { useSimulationStore } from '../state/useSimulationStore';
import { useGarmentInteraction } from '../hooks/useGarmentInteraction';
import { FabricMaterial } from '../materials/FabricMaterial';

/**
 * Renders the garment mesh and synchronizes it with the physics simulation.
 * Handles the "Zero-Copy" memory binding between WASM and WebGL.
 */
export const GarmentMesh: React.FC = () => {
    const meshRef = useRef<THREE.Mesh>(null);
    const { assets, scaledVertices, engine, step, isRunning } = useSimulationStore();

    // 1. Construct Initial Geometry
    // We use useMemo to create the geometry container.
    // Initially, it uses the 'scaledVertices' (CPU array) for the static pose.
    // Once simulation starts, we will hot-swap the position attribute with the WASM buffer.
    const geometry = useMemo(() => {
        if (!assets || !scaledVertices) return null;

        const geo = new THREE.BufferGeometry();

        // Initial Position (CPU Copy)
        const positionBuffer = new Float32Array(scaledVertices);
        geo.setAttribute('position', new THREE.BufferAttribute(positionBuffer, 3));

        // Static Attributes
        geo.setAttribute('normal', new THREE.BufferAttribute(assets.garment.normals, 3));
        geo.setIndex(new THREE.BufferAttribute(assets.garment.indices, 1));

        if (assets.garment.uvs && assets.garment.uvs.length > 0) {
            geo.setAttribute('uv', new THREE.BufferAttribute(assets.garment.uvs, 2));
        }

        if (assets.garment.tangents && assets.garment.tangents.length > 0) {
            geo.setAttribute('tangent', new THREE.BufferAttribute(assets.garment.tangents, 4));
        }

        console.log(`[GarmentMesh] Geometry Created. Verts: ${positionBuffer.length / 3}`);
        return geo;
    }, [assets, scaledVertices]);

    useEffect(() => {
        if (!geometry) {
            console.warn("[GarmentMesh] Geometry is null!");
        } else {
            console.log("[GarmentMesh] Geometry Ready.", geometry.getAttribute('position').count);
        }
    }, [geometry]);

    // 2. Attach Interaction Logic
    useGarmentInteraction(meshRef);

    // 3. The Render Loop (Runs every frame, e.g., 60fps)
    useFrame((_, delta) => {
        if (!engine || !meshRef.current || !geometry) return;

        // A. Step Physics
        // We accumulate time in the engine and step fixed time steps (e.g. 30Hz)
        if (isRunning) {
            // Clamp delta to prevent "Spiral of Death" if tab was backgrounded
            const safeDelta = Math.min(delta, 0.1);
            step(safeDelta);
        }

        // B. Sync Physics -> Visuals (Zero-Copy)
        // The engine returns a direct view into WASM memory (InterleavedBufferAttribute)
        // This avoids copying data from WASM to JS to Three.js every frame.
        const physicsAttribute = engine.getPositions() as THREE.BufferAttribute | THREE.InterleavedBufferAttribute;
        const normalsAttribute = engine.getNormals() as THREE.BufferAttribute | THREE.InterleavedBufferAttribute;

        // Check if we need to bind the Zero-Copy position buffer
        // This happens on the first frame of simulation, or if WASM memory resized.
        if (geometry.attributes.position !== physicsAttribute) {
            geometry.setAttribute('position', physicsAttribute);
        }

        // Bind Zero-Copy normal buffer (computed in WASM for performance)
        if (geometry.attributes.normal !== normalsAttribute) {
            geometry.setAttribute('normal', normalsAttribute);
        }

        // Flag buffers for GPU upload
        // This tells Three.js that the underlying data has changed and needs to be resent to the GPU.
        if (physicsAttribute instanceof THREE.InterleavedBufferAttribute) {
            physicsAttribute.data.needsUpdate = true;
        } else {
            physicsAttribute.needsUpdate = true;
        }

        if (normalsAttribute instanceof THREE.InterleavedBufferAttribute) {
            normalsAttribute.data.needsUpdate = true;
        } else {
            normalsAttribute.needsUpdate = true;
        }

        // NOTE: computeVertexNormals() has been moved to WASM (Rust)
        // to free up the main thread and achieve a smooth 60 FPS.
    });

    if (!geometry) return null;

    return (
        <mesh ref={meshRef} geometry={geometry} castShadow receiveShadow frustumCulled={false}>
            <FabricMaterial color="#3b82f6" />
        </mesh>
    );
};