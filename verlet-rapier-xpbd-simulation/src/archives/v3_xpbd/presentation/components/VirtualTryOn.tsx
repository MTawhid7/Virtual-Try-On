// src/v3/presentation/components/VirtualTryOn.tsx
import { useGLTF } from '@react-three/drei';
import { useMemo } from 'react';
import * as THREE from 'three';
import * as BufferGeometryUtils from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { useClothEngine } from '../../adapter/useClothEngine';
import { useInteraction } from '../../adapter/useInteraction';
import { PHYSICS_CONSTANTS } from '../../shared/constants';


function findFirstMesh(scene: THREE.Group | THREE.Scene | THREE.Object3D): THREE.Mesh | null {
    let mesh: THREE.Mesh | null = null;
    scene.traverse((child) => {
        if ((child as THREE.Mesh).isMesh && !mesh) {
            mesh = child as THREE.Mesh;
        }
    });
    return mesh;
}

export const VirtualTryOn = () => {
    const mannequinGLTF = useGLTF('/mannequin.glb');
    const proxyGLTF = useGLTF('/shirt_proxy.glb');
    const visualGLTF = useGLTF('/shirt_visual.glb');

    const mannequinMesh = useMemo(() => findFirstMesh(mannequinGLTF.scene), [mannequinGLTF]);

    const proxy = useMemo(() => {
        const rawMesh = findFirstMesh(proxyGLTF.scene);
        if (!rawMesh) return null;
        const geometry = rawMesh.geometry.clone();
        if (geometry.attributes.normal) geometry.deleteAttribute('normal');
        if (geometry.attributes.uv) geometry.deleteAttribute('uv');
        const weldedGeo = BufferGeometryUtils.mergeVertices(geometry, 0.001);
        weldedGeo.computeVertexNormals();
        const mesh = new THREE.Mesh(weldedGeo, rawMesh.material);
        mesh.position.copy(rawMesh.position);
        mesh.scale.copy(rawMesh.scale);
        mesh.quaternion.copy(rawMesh.quaternion);
        mesh.updateMatrixWorld(true);
        return mesh;
    }, [proxyGLTF]);

    const visual = useMemo(() => findFirstMesh(visualGLTF.scene), [visualGLTF]);

    const { engine } = useClothEngine(proxy, visual, mannequinMesh);
    useInteraction(engine, visual);

    return (
        <group>
            <primitive object={mannequinGLTF.scene} />

            <primitive object={visualGLTF.scene}>
                <meshStandardMaterial color="#4488ff" roughness={0.6} side={THREE.DoubleSide} />
            </primitive>

            {PHYSICS_CONSTANTS.debug.showProxy && (
                <primitive object={proxyGLTF.scene}>
                    <meshBasicMaterial color="yellow" wireframe transparent opacity={0.5} depthTest={false} />
                </primitive>
            )}


        </group>
    );
};