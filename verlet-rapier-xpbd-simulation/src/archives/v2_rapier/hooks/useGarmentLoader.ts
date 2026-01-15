import { useGLTF } from '@react-three/drei';
import { useMemo } from 'react';
import * as THREE from 'three';
import { processShirtGeometry } from '../utils/geometry';
import { computeSkinning } from '../utils/skinning';

export function useGarmentLoader() {
    // Load both GLBs
    const proxyGLTF = useGLTF('/shirt_proxy.glb');
    const visualGLTF = useGLTF('/shirt_visual.glb');

    const data = useMemo(() => {
        // 1. Extract Meshes
        const proxyMesh = findMesh(proxyGLTF.scene);
        const visualMesh = findMesh(visualGLTF.scene);

        if (!proxyMesh || !visualMesh) return null;

        // 2. Process Physics Geometry (Weld, find edges for Rapier)
        // We only need constraints for the PROXY
        const physicsData = processShirtGeometry(proxyMesh);

        // 3. Compute Skinning Mapping
        // This connects the high-poly visual to the low-poly physics
        const skinningData = computeSkinning(visualMesh, proxyMesh);

        return {
            physicsGeometry: physicsData.geometry,
            constraints: physicsData.constraints,
            vertexCount: physicsData.vertexCount,
            visualGeometry: visualMesh.geometry, // Keep original high-poly geometry
            skinning: skinningData
        };
    }, [proxyGLTF.scene, visualGLTF.scene]);

    return data;
}

// Helper to find the first mesh in a GLTF scene
function findMesh(scene: THREE.Group): THREE.Mesh | null {
    let mesh: THREE.Mesh | null = null;
    scene.traverse((child) => {
        if ((child as THREE.Mesh).isMesh && !mesh) mesh = child as THREE.Mesh;
    });
    return mesh;
}