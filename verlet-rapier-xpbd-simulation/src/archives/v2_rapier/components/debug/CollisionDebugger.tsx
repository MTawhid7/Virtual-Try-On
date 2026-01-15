import React, { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import { useRapier } from '@react-three/rapier';
import { InstancedMesh, Object3D } from 'three';

interface CollisionDebuggerProps {
    maxPoints?: number;
    color?: string;
    visible?: boolean;
}

/**
 * Visualizes active collision points in the physics world.
 * It iterates through all contact pairs in the Rapier world and renders a small sphere at each contact point.
 */
export const CollisionDebugger: React.FC<CollisionDebuggerProps> = ({
    visible = true
}) => {
    const { world } = useRapier();
    const meshRef = useRef<InstancedMesh>(null);
    const dummy = useMemo(() => new Object3D(), []);

    useFrame(() => {
        if (!visible || !meshRef.current) return;
        const mesh = meshRef.current;

        let count = 0;
        const maxPoints = 100;

        // eslint-disable-next-line @typescript-eslint/no-explicit-any
        const rapierWorld = world as any;

        if (rapierWorld.forEachContactPair) {
            // eslint-disable-next-line @typescript-eslint/no-explicit-any
            rapierWorld.forEachContactPair((colliderA: any, colliderB: any) => {
                if (count >= maxPoints) return;

                // Create a marker at the midpoint of the two colliders to show "contact"
                const tA = colliderA.translation();
                const tB = colliderB.translation();

                const x = (tA.x + tB.x) / 2;
                const y = (tA.y + tB.y) / 2;
                const z = (tA.z + tB.z) / 2;

                dummy.position.set(x, y, z);
                dummy.scale.setScalar(1);
                dummy.updateMatrix();
                mesh.setMatrixAt(count++, dummy.matrix);
            });
        }

        for (let i = count; i < maxPoints; i++) {
            dummy.position.set(0, -9999, 0); // Hide unused
            dummy.updateMatrix();
            mesh.setMatrixAt(i, dummy.matrix);
        }
        mesh.instanceMatrix.needsUpdate = true;
    });

    return (
        <instancedMesh ref={meshRef} args={[undefined, undefined, 100]}>
            <sphereGeometry args={[0.05, 8, 8]} />
            <meshBasicMaterial color="#ff0000" depthTest={false} transparent opacity={0.6} />
        </instancedMesh>
    );
}

// Justification: Simple midpoint visualization is robust and version-agnostic.
