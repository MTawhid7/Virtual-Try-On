import { useGLTF } from '@react-three/drei';
// FIX: Removed CuboidCollider
import { RigidBody } from '@react-three/rapier';
import { useEffect, useMemo } from 'react';
import * as THREE from 'three';

export const Mannequin = () => {
    const { scene } = useGLTF('/mannequin.glb');

    const position = useMemo(() => {
        const box = new THREE.Box3().setFromObject(scene);
        const center = box.getCenter(new THREE.Vector3());
        const x = -center.x;
        const y = -box.min.y;
        const z = -center.z;
        return [x, y, z] as [number, number, number];
    }, [scene]);

    useEffect(() => {
        scene.traverse((child) => {
            if ((child as THREE.Mesh).isMesh) {
                const mesh = child as THREE.Mesh;
                mesh.castShadow = true;
                mesh.receiveShadow = true;
                mesh.material = new THREE.MeshStandardMaterial({
                    color: '#eeeeee',
                    roughness: 0.2,
                    metalness: 0.1
                });
            }
        });
    }, [scene]);

    return (
        <RigidBody
            type="fixed"
            colliders="trimesh"
            position={position}
            friction={2.0}
            // Group 1, Interact with everything (default filter)
            // 0x0001 (Membership) | 0xFFFF (Filter)
            collisionGroups={0x0001FFFF}
        >
            <primitive object={scene} />
        </RigidBody>
    );
};