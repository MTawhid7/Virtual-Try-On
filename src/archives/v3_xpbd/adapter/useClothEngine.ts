// src/v3/adapter/useClothEngine.ts
import { useEffect, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { Solver } from '../engine/core/Solver';
import { computeSkinning, type SkinningData } from '../utils/skinning';
import { PHYSICS_CONSTANTS } from '../shared/constants';

export function useClothEngine(
    proxyMesh: THREE.Mesh | null,
    visualMesh: THREE.Mesh | null,
    mannequinMesh: THREE.Mesh | null
) {
    const engineRef = useRef<Solver | null>(null);
    const skinningRef = useRef<SkinningData | null>(null);
    const initialized = useRef(false);

    const accumulator = useRef(0);
    const FIXED_STEP = 1 / 60; // 16.6ms
    // --- NEW: Safety cap to prevent freezing ---
    const MAX_STEPS = 2;

    const pA = new THREE.Vector3();
    const pB = new THREE.Vector3();
    const pC = new THREE.Vector3();

    useEffect(() => {
        if (!proxyMesh || !visualMesh || !mannequinMesh || initialized.current) return;

        console.log('[Adapter] Initializing Physics Engine V3...');

        // Debug: Check bounds to verify pinHeight
        proxyMesh.geometry.computeBoundingBox();
        const bounds = proxyMesh.geometry.boundingBox;
        if (bounds) {
            console.log(`[Physics] Proxy Mesh Height Range: Y=[${bounds.min.y.toFixed(2)}, ${bounds.max.y.toFixed(2)}]`);
            console.log(`[Physics] Pin Radius Threshold: ${PHYSICS_CONSTANTS.pinDepth}`);
            if (bounds.max.y < PHYSICS_CONSTANTS.pinDepth) {
                console.warn("⚠️ WARNING: Mesh is shorter than pinDepth! No vertices will be pinned.");
            }
        }

        proxyMesh.updateMatrixWorld(true);
        mannequinMesh.updateMatrixWorld(true);

        const engine = new Solver(proxyMesh, mannequinMesh);
        engineRef.current = engine;
        skinningRef.current = computeSkinning(visualMesh, proxyMesh);
        initialized.current = true;
    }, [proxyMesh, visualMesh, mannequinMesh]);

    useFrame((_, delta) => {
        const engine = engineRef.current;
        const skinning = skinningRef.current;
        if (!engine || !proxyMesh || !visualMesh || !skinning) return;

        // --- FIX: Prevent Spiral of Death ---
        // 1. Cap the delta time (e.g., if tab was inactive)
        let frameTime = Math.min(delta, 0.1);
        accumulator.current += frameTime;

        // 2. Run fixed steps with a safety exit
        let steps = 0;
        while (accumulator.current >= FIXED_STEP && steps < MAX_STEPS) {
            engine.update(FIXED_STEP);
            accumulator.current -= FIXED_STEP;
            steps++;
        }

        // 3. If we are still behind, discard the accumulated time to avoid catch-up lag
        if (accumulator.current > FIXED_STEP) {
            accumulator.current = 0;
        }
        // ------------------------------------

        const visualPos = visualMesh.geometry.attributes.position;
        const visualNormals = visualMesh.geometry.attributes.normal;
        const proxyPos = engine.data.positions;
        const physicsIndex = proxyMesh.geometry.index!;
        const { indices, weights } = skinning;

        const BIAS = 0.005;

        for (let i = 0; i < visualPos.count; i++) {
            const faceIdx = indices[i];
            const w1 = weights[i * 3];
            const w2 = weights[i * 3 + 1];
            const w3 = weights[i * 3 + 2];

            const idxA = physicsIndex.getX(faceIdx * 3) * 3;
            const idxB = physicsIndex.getX(faceIdx * 3 + 1) * 3;
            const idxC = physicsIndex.getX(faceIdx * 3 + 2) * 3;

            pA.set(proxyPos[idxA], proxyPos[idxA + 1], proxyPos[idxA + 2]);
            pB.set(proxyPos[idxB], proxyPos[idxB + 1], proxyPos[idxB + 2]);
            pC.set(proxyPos[idxC], proxyPos[idxC + 1], proxyPos[idxC + 2]);

            const x = pA.x * w1 + pB.x * w2 + pC.x * w3;
            const y = pA.y * w1 + pB.y * w2 + pC.y * w3;
            const z = pA.z * w1 + pB.z * w2 + pC.z * w3;

            const nx = visualNormals.getX(i);
            const ny = visualNormals.getY(i);
            const nz = visualNormals.getZ(i);

            visualPos.setXYZ(i, x + nx * BIAS, y + ny * BIAS, z + nz * BIAS);
        }

        visualPos.needsUpdate = true;
        visualMesh.geometry.computeVertexNormals();

        if (PHYSICS_CONSTANTS.debug.showProxy) {
            engine.data.syncToMesh(proxyMesh);
        }
    });

    return { engine: engineRef.current };
}