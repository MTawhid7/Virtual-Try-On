// src/v2/components/garment/Shirt.tsx

import { useRef, useMemo, useState } from 'react';
import { useFrame } from '@react-three/fiber';
import { Html } from '@react-three/drei';
import * as THREE from 'three';
import { useGarmentLoader } from '../../hooks/useGarmentLoader';
import { useGarmentPhysics } from '../../hooks/useGarmentPhysics';
import { useGarmentInteraction } from '../../hooks/useGarmentInteraction';
import { useSettlingForce } from '../../hooks/useSettlingForce';
import { usePhysicsDebug } from '../../hooks/usePhysicsDebug';
import { GarmentVisualizer } from './GarmentVisualizer';
import { DebugPanel } from '../ui/DebugPanel';
import { MaterialSelector } from '../ui/MaterialSelector';
import { ControlPanel } from '../ui/ControlPanel';
import type { MaterialType } from '../../types/garment.types';
import { MATERIAL_VISUAL_PROPERTIES } from '../../utils/materials';
import { DEBUG_FLAGS } from '../../config/debug.config';
import { PerformanceTimer, validateRigidBody, validateVector3 } from '../../utils/validation';

export const Shirt = () => {
    const visualRef = useRef<THREE.Mesh>(null!);
    const proxyRef = useRef<THREE.Mesh>(null!);

    // State
    const [materialType, setMaterialType] = useState<MaterialType>('cotton');
    const [showProxy, setShowProxy] = useState(false);


    // Load assets
    const data = useGarmentLoader();
    const startPos = useMemo(() => new THREE.Vector3(0, 1.15, 0), []);

    // Initialize physics
    const { bodies, initialized } = useGarmentPhysics(
        data?.physicsGeometry || null,
        data?.constraints || [],
        data?.vertexCount || 0,
        startPos,
        materialType
    );

    // Setup interaction
    const { handlePointerDown, handlePointerUp, isDragging, draggedIndex } =
        useGarmentInteraction(bodies, visualRef);

    // Auto-settling
    const { isActive: isSettling, progress: settlingProgress } =
        useSettlingForce(bodies, initialized);

    // Debug metrics
    const { debugInfo, performanceMetrics } = usePhysicsDebug(bodies, initialized);

    // Performance timer
    const timer = useRef(new PerformanceTimer());

    // Skinning update loop
    useFrame(() => {
        if (!initialized.current || !visualRef.current || !data) return;
        if (bodies.current.length === 0) return;

        timer.current.start();

        const visualPos = visualRef.current.geometry.attributes.position;
        const { indices, weights } = data.skinning;
        const physicsIndex = data.physicsGeometry.index!;

        // Update proxy mesh for debugging
        if (proxyRef.current) {
            const proxyPos = proxyRef.current.geometry.attributes.position;
            let updatedCount = 0;

            for (let i = 0; i < bodies.current.length; i++) {
                if (!validateRigidBody(bodies.current[i], i)) continue;

                try {
                    const t = bodies.current[i].translation();
                    if (validateVector3(t, `Proxy[${i}]`)) {
                        proxyPos.setXYZ(i, t.x, t.y, t.z);
                        updatedCount++;
                    }
                } catch (e) {
                    continue;
                }
            }

            proxyPos.needsUpdate = true;
        }

        // Update visual mesh using barycentric skinning
        const pA = new THREE.Vector3();
        const pB = new THREE.Vector3();
        const pC = new THREE.Vector3();
        const finalPos = new THREE.Vector3();
        let skinningErrors = 0;
        let successfulUpdates = 0;

        // Optimization: prevent running if critical data is missing
        if (visualPos.count === 0 || indices.length === 0) return;

        for (let i = 0; i < visualPos.count; i++) {
            const faceIdx = indices[i];
            const w1 = weights[i * 3];
            const w2 = weights[i * 3 + 1];
            const w3 = weights[i * 3 + 2];

            // Get physics triangle vertices
            const idxA = physicsIndex.getX(faceIdx * 3);
            const idxB = physicsIndex.getX(faceIdx * 3 + 1);
            const idxC = physicsIndex.getX(faceIdx * 3 + 2);

            // Safety check for indices
            if (idxA >= bodies.current.length || idxB >= bodies.current.length || idxC >= bodies.current.length) {
                skinningErrors++;
                continue;
            }

            const bA = bodies.current[idxA];
            const bB = bodies.current[idxB];
            const bC = bodies.current[idxC];

            if (!validateRigidBody(bA, idxA) ||
                !validateRigidBody(bB, idxB) ||
                !validateRigidBody(bC, idxC)) {
                skinningErrors++;
                continue;
            }

            try {
                const tA = bA.translation();
                const tB = bB.translation();
                const tC = bC.translation();

                pA.set(tA.x, tA.y, tA.z);
                pB.set(tB.x, tB.y, tB.z);
                pC.set(tC.x, tC.y, tC.z);

                // Compute barycentric interpolation
                finalPos.set(0, 0, 0)
                    .addScaledVector(pA, w1)
                    .addScaledVector(pB, w2)
                    .addScaledVector(pC, w3);

                visualPos.setXYZ(i, finalPos.x, finalPos.y, finalPos.z);
                successfulUpdates++;

            } catch (e) {
                skinningErrors++;
            }
        }

        // Log errors if significant (once per second max to avoid spam, TODO)
        if (skinningErrors > visualPos.count * 0.5 && DEBUG_FLAGS.logNaNErrors) {
            // Throttled logging could go here
        }

        visualPos.needsUpdate = true;
        visualRef.current.geometry.computeVertexNormals();

        timer.current.end('skinning');
    });

    // Event handlers
    const handleMaterialChange = (newMaterial: MaterialType) => {
        console.log(`🔄 Changing material to: ${newMaterial}`);
        setMaterialType(newMaterial);
    };

    const handleReset = () => {
        console.log('🔄 Resetting simulation');
    };

    const handleToggleProxy = () => {
        setShowProxy(prev => !prev);
    };

    if (!data) {
        return (
            <div style={{
                position: 'absolute',
                top: '50%',
                left: '50%',
                transform: 'translate(-50%, -50%)',
                color: '#fff',
                fontSize: '1.2rem',
                background: 'rgba(0,0,0,0.8)',
                padding: '20px',
                borderRadius: '10px',
            }}>
                Loading garment assets...
            </div>
        );
    }

    const visualProps = MATERIAL_VISUAL_PROPERTIES[materialType];

    return (
        <>
            <GarmentVisualizer
                visualRef={visualRef}
                proxyRef={proxyRef}
                visualGeometry={data.visualGeometry}
                physicsGeometry={data.physicsGeometry}
                showProxy={showProxy}
                visualColor={visualProps.color}
                visualRoughness={visualProps.roughness}
                visualMetalness={visualProps.metalness}
                onPointerDown={handlePointerDown}
                onPointerUp={handlePointerUp}
            />

            {/* UI Controls - Wrapped in Html to render outside Canvas context */}
            <Html fullscreen style={{ pointerEvents: 'none' }}>
                <div style={{ pointerEvents: 'auto' }}>
                    <ControlPanel
                        showProxy={showProxy}
                        onToggleProxy={handleToggleProxy}
                        onReset={handleReset}
                    />
                </div>

                <div style={{ pointerEvents: 'auto' }}>
                    <MaterialSelector
                        currentMaterial={materialType}
                        onMaterialChange={handleMaterialChange}
                    />
                </div>

                {DEBUG_FLAGS.enablePerformanceMetrics && (
                    <DebugPanel
                        debugInfo={debugInfo}
                        performanceMetrics={performanceMetrics}
                        settlingProgress={settlingProgress}
                        isSettling={isSettling}
                    />
                )}

                {/* Interaction feedback */}
                {isDragging && (
                    <div style={{
                        position: 'absolute',
                        top: '50%',
                        left: '50%',
                        transform: 'translate(-50%, -50%)',
                        color: '#fff',
                        background: 'rgba(74, 158, 255, 0.9)',
                        padding: '8px 16px',
                        borderRadius: '20px',
                        fontSize: '14px',
                        fontFamily: 'system-ui, sans-serif',
                        pointerEvents: 'none',
                        fontWeight: '500',
                    }}>
                        🎯 Dragging vertex {draggedIndex}
                    </div>
                )}
            </Html>
        </>
    );
};