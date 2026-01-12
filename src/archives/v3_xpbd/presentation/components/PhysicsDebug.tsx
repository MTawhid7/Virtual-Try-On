// src/v3/presentation/components/PhysicsDebug.tsx
import { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { Solver } from '../../engine/core/Solver';

export const PhysicsDebug = ({ engine }: { engine: Solver | null }) => {
    const pointsRef = useRef<THREE.InstancedMesh>(null);
    const linesRef = useRef<THREE.LineSegments>(null);
    const dummy = new THREE.Object3D();
    const MAX_POINTS = 1000;

    const lineGeometry = useMemo(() => {
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(2000 * 3), 3));
        return geo;
    }, []);

    useFrame(() => {
        if (!engine || !pointsRef.current || !linesRef.current) return;

        const { points, colors, lines } = engine.collider.debugData;
        const count = Math.min(points.length, MAX_POINTS);

        // Update Points
        pointsRef.current.count = count;
        for (let i = 0; i < count; i++) {
            dummy.position.copy(points[i]);
            dummy.scale.setScalar(1);
            dummy.updateMatrix();
            pointsRef.current.setMatrixAt(i, dummy.matrix);
            pointsRef.current.setColorAt(i, colors[i]);
        }
        pointsRef.current.instanceMatrix.needsUpdate = true;
        if (pointsRef.current.instanceColor) pointsRef.current.instanceColor.needsUpdate = true;

        // Update Lines
        const linePos = linesRef.current.geometry.attributes.position;
        const lineCount = Math.min(lines.length, 2000);
        linesRef.current.geometry.setDrawRange(0, lineCount);

        for (let i = 0; i < lineCount; i++) {
            linePos.setXYZ(i, lines[i].x, lines[i].y, lines[i].z);
        }
        linePos.needsUpdate = true;
    });

    return (
        <group renderOrder={999}>
            {/* Points: Always on top */}
            <instancedMesh ref={pointsRef} args={[undefined, undefined, MAX_POINTS]}>
                <sphereGeometry args={[0.008, 8, 8]} />
                <meshBasicMaterial color="white" toneMapped={false} depthTest={false} transparent opacity={0.9} />
            </instancedMesh>

            {/* Lines: Push direction */}
            <lineSegments ref={linesRef} geometry={lineGeometry}>
                <lineBasicMaterial color="cyan" depthTest={false} transparent opacity={0.6} linewidth={2} />
            </lineSegments>
        </group>
    );
};