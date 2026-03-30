// src/presentation/hooks/useGarmentInteraction.ts

import { useRef, useEffect } from 'react';
import { useThree } from '@react-three/fiber';
import * as THREE from 'three';
import { useSimulationStore } from '../state/useSimulationStore';

/**
 * Enables "Grab and Drag" interaction for the garment mesh.
 *
 * @param meshRef - Reference to the rendered Three.js mesh.
 */
export function useGarmentInteraction(meshRef: React.RefObject<THREE.Mesh | null>) {
    const { camera, raycaster, gl } = useThree();
    const { grabParticle, moveParticle, releaseParticle, isRunning, setInteracting } = useSimulationStore();

    // State refs to track drag status without triggering re-renders
    const isDragging = useRef(false);

    // A virtual plane that faces the camera. We project mouse movements onto this plane
    // to determine where the particle should move in 3D space.
    // This provides a consistent "depth" for the drag relative to the view.
    const dragPlane = useRef(new THREE.Plane());
    const intersectPoint = new THREE.Vector3();

    useEffect(() => {
        const canvas = gl.domElement;

        const handleDown = (e: PointerEvent) => {
            // Interaction is only allowed when simulation is running
            if (!isRunning || !meshRef.current) return;

            // 1. Normalized Device Coordinates (NDC)
            const rect = canvas.getBoundingClientRect();
            const x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
            const y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

            // 2. Raycast against the Garment
            raycaster.setFromCamera(new THREE.Vector2(x, y), camera);
            const intersects = raycaster.intersectObject(meshRef.current);

            if (intersects.length > 0) {
                const hit = intersects[0];

                if (hit.face) {
                    // 3. Find the closest vertex in the hit triangle
                    // The raycast gives us the face index (a, b, c). We check which one is closest to the hit point.
                    const geo = meshRef.current.geometry;
                    const posAttr = geo.attributes.position;

                    const a = hit.face.a;
                    const b = hit.face.b;
                    const c = hit.face.c;

                    const pA = new THREE.Vector3().fromBufferAttribute(posAttr, a);
                    const pB = new THREE.Vector3().fromBufferAttribute(posAttr, b);
                    const pC = new THREE.Vector3().fromBufferAttribute(posAttr, c);

                    const dA = pA.distanceToSquared(hit.point);
                    const dB = pB.distanceToSquared(hit.point);
                    const dC = pC.distanceToSquared(hit.point);

                    let closestIdx = a;
                    if (dB < dA && dB < dC) closestIdx = b;
                    if (dC < dA && dC < dB) closestIdx = c;

                    // 4. Start Dragging
                    isDragging.current = true;
                    setInteracting(true);

                    // Define the drag plane perpendicular to the camera view
                    const normal = new THREE.Vector3();
                    camera.getWorldDirection(normal);
                    dragPlane.current.setFromNormalAndCoplanarPoint(normal, hit.point);

                    // Notify Physics Engine
                    grabParticle(closestIdx, [hit.point.x, hit.point.y, hit.point.z]);

                    // Visual Feedback
                    canvas.style.cursor = 'grabbing';
                }
            }
        };

        const handleMove = (e: PointerEvent) => {
            if (!isDragging.current) return;

            const rect = canvas.getBoundingClientRect();
            const x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
            const y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

            raycaster.setFromCamera(new THREE.Vector2(x, y), camera);

            // Project ray onto the drag plane
            if (raycaster.ray.intersectPlane(dragPlane.current, intersectPoint)) {
                moveParticle([intersectPoint.x, intersectPoint.y, intersectPoint.z]);
            }
        };

        const handleUp = () => {
            if (isDragging.current) {
                isDragging.current = false;
                setInteracting(false);
                releaseParticle();
                canvas.style.cursor = 'auto';
            }
        };

        // Attach listeners
        // We attach 'move' and 'up' to window to handle dragging outside the canvas bounds
        canvas.addEventListener('pointerdown', handleDown);
        window.addEventListener('pointermove', handleMove);
        window.addEventListener('pointerup', handleUp);
        canvas.addEventListener('pointerleave', handleUp);

        return () => {
            canvas.removeEventListener('pointerdown', handleDown);
            window.removeEventListener('pointermove', handleMove);
            window.removeEventListener('pointerup', handleUp);
            canvas.removeEventListener('pointerleave', handleUp);
        };
    }, [camera, raycaster, gl, isRunning, grabParticle, moveParticle, releaseParticle, meshRef, setInteracting]);
}