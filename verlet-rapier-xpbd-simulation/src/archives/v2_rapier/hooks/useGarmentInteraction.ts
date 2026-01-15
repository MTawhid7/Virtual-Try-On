// src/v2/hooks/useGarmentInteraction.ts

import { useState, useCallback, useRef } from 'react';
import { useThree, useFrame } from '@react-three/fiber';
import type { ThreeEvent } from '@react-three/fiber';
import { useRapier, RapierRigidBody } from '@react-three/rapier';
import * as THREE from 'three';
import { INTERACTION_CONFIG } from '../config/physics.config';
import { DEBUG_FLAGS } from '../config/debug.config';
import { validateRigidBody, validateVector3 } from '../utils/validation';

export function useGarmentInteraction(
    bodies: React.MutableRefObject<RapierRigidBody[]>,
    meshRef: React.MutableRefObject<THREE.Mesh | null>
) {
    const { rapier } = useRapier();
    const { camera, raycaster, pointer } = useThree();

    const [isDragging, setIsDragging] = useState(false);
    const draggedBodyRef = useRef<RapierRigidBody | null>(null);
    const draggedIndexRef = useRef<number>(-1);
    const targetPositionRef = useRef<THREE.Vector3>(new THREE.Vector3());
    const dragStartTimeRef = useRef<number>(0);

    const handlePointerDown = useCallback((e: ThreeEvent<PointerEvent>) => {
        e.stopPropagation();
        if (!meshRef.current) return;

        const intersects = raycaster.intersectObject(meshRef.current);

        if (intersects.length > 0) {
            const point = intersects[0].point;
            let minDist = Infinity;
            let closestIdx = -1;

            // Find closest physics body to click point
            for (let i = 0; i < bodies.current.length; i++) {
                if (!validateRigidBody(bodies.current[i], i)) continue;

                try {
                    const t = bodies.current[i].translation();
                    const distSq =
                        Math.pow(t.x - point.x, 2) +
                        Math.pow(t.y - point.y, 2) +
                        Math.pow(t.z - point.z, 2);

                    if (distSq < minDist) {
                        minDist = distSq;
                        closestIdx = i;
                    }
                } catch (e) {
                    continue;
                }
            }

            const dist = Math.sqrt(minDist);
            if (closestIdx !== -1 && dist < INTERACTION_CONFIG.grabRadius) {
                const body = bodies.current[closestIdx];

                // Skip pinned bodies
                if (body.bodyType() === rapier.RigidBodyType.Fixed) {
                    if (DEBUG_FLAGS.enableLogging) {
                        console.log('🔒 Attempted to grab pinned vertex');
                    }
                    return;
                }

                draggedBodyRef.current = body;
                draggedIndexRef.current = closestIdx;
                setIsDragging(true);
                dragStartTimeRef.current = performance.now();

                if (DEBUG_FLAGS.enableLogging) {
                    console.log(`🎯 Grabbed vertex ${closestIdx} (distance: ${dist.toFixed(3)}m)`);
                }
            }
        }
    }, [meshRef, bodies, raycaster, rapier]);

    const handlePointerUp = useCallback(() => {
        if (isDragging) {
            const duration = performance.now() - dragStartTimeRef.current;

            if (DEBUG_FLAGS.enableLogging) {
                console.log(`✋ Released vertex ${draggedIndexRef.current} (held for ${duration.toFixed(0)}ms)`);
            }

            draggedBodyRef.current = null;
            draggedIndexRef.current = -1;
            setIsDragging(false);
        }
    }, [isDragging]);

    // Apply mouse spring force every frame while dragging
    useFrame(() => {
        if (!isDragging || !draggedBodyRef.current) return;

        const body = draggedBodyRef.current;
        if (!validateRigidBody(body, draggedIndexRef.current)) {
            handlePointerUp();
            return;
        }

        // Calculate target position on drag plane
        raycaster.setFromCamera(pointer, camera);
        const cameraDir = camera.getWorldDirection(new THREE.Vector3());
        const plane = new THREE.Plane(cameraDir, INTERACTION_CONFIG.dragPlaneOffset);

        if (!raycaster.ray.intersectPlane(plane, targetPositionRef.current)) {
            return;
        }

        // Clamp distance from camera
        const camPos = camera.position;
        const dist = targetPositionRef.current.distanceTo(camPos);
        if (dist > INTERACTION_CONFIG.maxDragDistance) {
            targetPositionRef.current.sub(camPos)
                .normalize()
                .multiplyScalar(INTERACTION_CONFIG.maxDragDistance)
                .add(camPos);
        }

        // Validate target position
        if (!validateVector3(targetPositionRef.current, 'Mouse target position')) {
            return;
        }

        // Calculate spring force: F = k * (target - current) - damping * velocity
        try {
            const currentPos = body.translation();
            const currentVel = body.linvel();

            const force = new THREE.Vector3(
                (targetPositionRef.current.x - currentPos.x) * INTERACTION_CONFIG.springStiffness,
                (targetPositionRef.current.y - currentPos.y) * INTERACTION_CONFIG.springStiffness,
                (targetPositionRef.current.z - currentPos.z) * INTERACTION_CONFIG.springStiffness
            );

            // Add damping (opposes velocity)
            force.x -= currentVel.x * INTERACTION_CONFIG.springDamping;
            force.y -= currentVel.y * INTERACTION_CONFIG.springDamping;
            force.z -= currentVel.z * INTERACTION_CONFIG.springDamping;

            // Validate and apply force
            if (validateVector3(force, 'Mouse spring force')) {
                body.applyImpulse(force, true);
            }
        } catch (e) {
            if (DEBUG_FLAGS.enableLogging) {
                console.error('Error applying mouse spring force:', e);
            }
            handlePointerUp();
        }
    });

    return {
        handlePointerDown,
        handlePointerUp,
        isDragging,
        draggedIndex: draggedIndexRef.current,
    };
}