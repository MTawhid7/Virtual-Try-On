// src/v4/adapter/useInteraction.ts
import { useEffect, useRef } from 'react';
import { useThree, useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { Bridge } from './Bridge';
import { OrbitControls } from 'three-stdlib';

export function useInteraction(bridge: Bridge, visualMeshRef: React.MutableRefObject<THREE.Mesh | null>) {
    const { gl, camera, raycaster, pointer, controls } = useThree();

    const isDragging = useRef(false);
    const dragPlane = useRef(new THREE.Plane());
    const intersectPoint = new THREE.Vector3();

    useEffect(() => {
        const canvas = gl.domElement;

        const handleDown = () => {
            if (!visualMeshRef.current) return;

            // 1. Raycast
            raycaster.setFromCamera(pointer, camera);
            const intersects = raycaster.intersectObject(visualMeshRef.current);

            if (intersects.length > 0) {
                const hit = intersects[0];

                // 2. Find Particle
                const particleIdx = bridge.findNearestParticle(hit.point);

                if (particleIdx !== -1) {
                    // 3. Lock Controls
                    if (controls) (controls as OrbitControls).enabled = false;
                    isDragging.current = true;

                    // 4. Setup Drag Plane (facing camera)
                    const normal = new THREE.Vector3();
                    camera.getWorldDirection(normal);
                    dragPlane.current.setFromNormalAndCoplanarPoint(normal, hit.point);

                    // 5. Start Physics Grab
                    bridge.startInteraction(particleIdx, hit.point.x, hit.point.y, hit.point.z);
                }
            }
        };

        const handleUp = () => {
            if (isDragging.current) {
                isDragging.current = false;
                if (controls) (controls as OrbitControls).enabled = true;
                bridge.endInteraction();
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
    }, [gl, camera, pointer, controls, bridge, visualMeshRef]);

    useFrame(() => {
        if (!isDragging.current) return;

        raycaster.setFromCamera(pointer, camera);
        if (raycaster.ray.intersectPlane(dragPlane.current, intersectPoint)) {
            bridge.updateInteraction(intersectPoint.x, intersectPoint.y, intersectPoint.z);
        }
    });
}