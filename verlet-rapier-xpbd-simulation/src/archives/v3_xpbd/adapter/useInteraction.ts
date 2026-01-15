// src/v3/adapter/useInteraction.ts
import { useEffect, useRef } from 'react';
import { useThree, useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { OrbitControls } from 'three-stdlib';
import { Solver } from '../engine/core/Solver';

export function useInteraction(
    solver: Solver | null,
    visualMesh: THREE.Mesh | null
) {
    const { gl, camera, raycaster, pointer, controls } = useThree();

    const isDragging = useRef(false);
    const dragPlane = useRef(new THREE.Plane());
    const mousePos3D = useRef(new THREE.Vector3());

    // Velocity Smoothing
    const historySize = 5;
    const positionHistory = useRef<THREE.Vector3[]>([]);
    const lastTime = useRef(0);

    useEffect(() => {
        const canvas = gl.domElement;

        const handleDown = (e: PointerEvent) => {
            if (!visualMesh || !solver) return;

            raycaster.setFromCamera(pointer, camera);
            const intersects = raycaster.intersectObject(visualMesh, true);

            if (intersects.length > 0) {
                const hit = intersects[0];

                // Find nearest particle
                let minDst = Infinity;
                let idx = -1;
                const pos = solver.data.positions;

                for (let i = 0; i < solver.data.count; i++) {
                    const dx = pos[i * 3] - hit.point.x;
                    const dy = pos[i * 3 + 1] - hit.point.y;
                    const dz = pos[i * 3 + 2] - hit.point.z;
                    const dst = dx * dx + dy * dy + dz * dz;

                    if (dst < minDst) {
                        minDst = dst;
                        idx = i;
                    }
                }

                if (minDst < 0.01 && idx !== -1) { // 10cm threshold squared (0.1^2 = 0.01)
                    if (controls) (controls as OrbitControls).enabled = false;

                    isDragging.current = true;

                    // Setup Plane
                    const normal = new THREE.Vector3();
                    camera.getWorldDirection(normal);
                    dragPlane.current.setFromNormalAndCoplanarPoint(normal, hit.point);

                    // Start Solver Interaction
                    solver.startInteraction(idx, hit.point);

                    // Reset History
                    positionHistory.current = [hit.point.clone()];
                    lastTime.current = performance.now();

                    e.stopPropagation();
                }
            }
        };

        const handleUp = () => {
            if (isDragging.current && solver) {
                // Calculate Release Velocity
                const now = performance.now();
                const dt = (now - lastTime.current) / 1000;

                let velocity = new THREE.Vector3(0, 0, 0);

                if (positionHistory.current.length >= 2 && dt > 0) {
                    const last = positionHistory.current[positionHistory.current.length - 1];
                    const first = positionHistory.current[0];
                    // Simple average velocity over the drag history
                    velocity.subVectors(last, first).divideScalar(dt * positionHistory.current.length); // Approx
                }

                // Clamp Velocity
                const maxSpeed = 5.0;
                if (velocity.length() > maxSpeed) velocity.setLength(maxSpeed);

                solver.endInteraction(velocity);

                isDragging.current = false;
                if (controls) (controls as OrbitControls).enabled = true;
            }
        };

        canvas.addEventListener('pointerdown', handleDown);
        window.addEventListener('pointerup', handleUp);
        window.addEventListener('pointerleave', handleUp);

        return () => {
            canvas.removeEventListener('pointerdown', handleDown);
            window.removeEventListener('pointerup', handleUp);
            window.removeEventListener('pointerleave', handleUp);
        };
    }, [gl, camera, visualMesh, solver, pointer, raycaster, controls]);

    useFrame(() => {
        if (!isDragging.current || !solver) return;

        raycaster.setFromCamera(pointer, camera);
        if (raycaster.ray.intersectPlane(dragPlane.current, mousePos3D.current)) {

            // Update Solver
            solver.updateInteraction(mousePos3D.current);

            // Update History
            positionHistory.current.push(mousePos3D.current.clone());
            if (positionHistory.current.length > historySize) {
                positionHistory.current.shift();
            }
            lastTime.current = performance.now();
        }
    });
}