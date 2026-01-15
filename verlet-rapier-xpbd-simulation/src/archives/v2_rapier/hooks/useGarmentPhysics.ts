// src/v2/hooks/useGarmentPhysics.ts

import { useEffect, useRef } from 'react';
import { useRapier, RapierRigidBody } from '@react-three/rapier';
import * as THREE from 'three';
import type { MaterialType } from '../types/garment.types';
import { getMaterialParams } from '../utils/materials';
import { PHYSICS_CONFIG } from '../config/physics.config';
import { DEBUG_FLAGS } from '../config/debug.config';
import { validateVector3 } from '../utils/validation';

export function useGarmentPhysics(
    geometry: THREE.BufferGeometry | null,
    constraints: [number, number, number][],
    vertexCount: number,
    startPos: THREE.Vector3,
    materialType: MaterialType = 'cotton'
) {
    const { world, rapier } = useRapier();
    const bodies = useRef<RapierRigidBody[]>([]);
    const initialized = useRef(false);
    const materialParams = getMaterialParams(materialType);

    useEffect(() => {
        if (!geometry || initialized.current) return;

        console.group('🔧 [Physics] Initializing Garment Physics');
        console.log('Material:', materialParams.name);
        console.log('Vertices:', vertexCount);
        console.log('Constraints:', constraints.length);

        const startTime = performance.now();
        const posAttr = geometry.attributes.position;
        const tempBodies: RapierRigidBody[] = [];

        // 1. Calculate pinning threshold
        let maxY = -Infinity;
        for (let i = 0; i < vertexCount; i++) {
            const y = posAttr.getY(i);
            if (y > maxY) maxY = y;
        }
        const pinThreshold = maxY - 0.02;

        console.log('Pin threshold (Y):', pinThreshold.toFixed(3));

        // 2. Create rigid bodies for each vertex
        let pinnedCount = 0;
        for (let i = 0; i < vertexCount; i++) {
            const x = posAttr.getX(i) + startPos.x;
            const y = posAttr.getY(i) + startPos.y;
            const z = posAttr.getZ(i) + startPos.z;

            // Validate position
            if (!validateVector3({ x, y, z }, `Body[${i}] initial position`)) {
                console.error(`Skipping body ${i} due to invalid position`);
                continue;
            }

            const rigidBodyDesc = rapier.RigidBodyDesc.dynamic()
                .setTranslation(x, y, z)
                .setLinearDamping(materialParams.linearDamping)
                .setAngularDamping(materialParams.angularDamping)
                .setGravityScale(materialParams.gravityScale)
                .setCcdEnabled(true);

            const rigidBody = world.createRigidBody(rigidBodyDesc);

            // Create collision sphere
            // Membership: Group 2 (Garment)
            // Filter: Only interact with Group 1 (Mannequin)
            const interactionGroups =
                (PHYSICS_CONFIG.collisionGroups.garment << 16) |
                PHYSICS_CONFIG.collisionGroups.mannequin;

            const colliderDesc = rapier.ColliderDesc.ball(materialParams.colliderRadius)
                .setMass(materialParams.mass)
                .setFriction(materialParams.friction)
                .setRestitution(materialParams.restitution)
                .setCollisionGroups(interactionGroups);

            world.createCollider(colliderDesc, rigidBody);
            tempBodies.push(rigidBody);

            // Pin vertices at the top (neck area)
            if (posAttr.getY(i) > pinThreshold) {
                rigidBody.setBodyType(rapier.RigidBodyType.Fixed, true);
                pinnedCount++;
            }
        }

        console.log('Bodies created:', tempBodies.length);
        console.log('Pinned vertices:', pinnedCount);

        // 3. Create spring constraints
        let springCount = 0;
        constraints.forEach(([iA, iB, dist]) => {
            if (iA >= tempBodies.length || iB >= tempBodies.length) {
                console.warn(`Invalid constraint indices: ${iA}, ${iB}`);
                return;
            }

            const params = rapier.JointData.spring(
                dist * materialParams.restLengthScale,
                materialParams.stiffness,
                materialParams.damping,
                { x: 0, y: 0, z: 0 },
                { x: 0, y: 0, z: 0 }
            );

            world.createImpulseJoint(params, tempBodies[iA], tempBodies[iB], true);
            springCount++;
        });

        console.log('Springs created:', springCount);

        bodies.current = tempBodies;
        initialized.current = true;

        const duration = performance.now() - startTime;
        console.log(`✅ Physics initialized in ${duration.toFixed(2)}ms`);
        console.groupEnd();

        // Cleanup function
        return () => {
            console.log('🧹 [Physics] Cleaning up physics bodies');
            initialized.current = false;

            tempBodies.forEach((b, idx) => {
                try {
                    if (world.getRigidBody(b.handle)) {
                        world.removeRigidBody(b);
                    }
                } catch (e) {
                    if (DEBUG_FLAGS.enableLogging) {
                        console.warn(`Failed to remove body ${idx}:`, e);
                    }
                }
            });

            bodies.current = [];
        };
        // Auto-release pinning after 1 second
        useEffect(() => {
            if (!initialized.current) return;

            const timer = setTimeout(() => {
                console.log('🔓 [Physics] Releasing pinned vertices');
                bodies.current.forEach(body => {
                    if (body && body.bodyType() === rapier.RigidBodyType.Fixed) {
                        body.setBodyType(rapier.RigidBodyType.Dynamic, true);
                    }
                });
            }, 1000);

            return () => clearTimeout(timer);
        }, [initialized.current, bodies.current]);

    }, [geometry, vertexCount, constraints, world, rapier, startPos, materialType]);

    return {
        bodies,
        initialized,
        materialParams
    };
}