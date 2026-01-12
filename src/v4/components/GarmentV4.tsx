// src/v4/components/GarmentV4.tsx
import { useEffect, useRef, useState } from 'react';
import { useFrame } from '@react-three/fiber';
import { useGLTF } from '@react-three/drei';
import * as THREE from 'three';
import * as BufferGeometryUtils from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { Bridge } from '../adapter/Bridge';
import { SDFVisualizer } from './SDFVisualizer';
import { computeSkinning, type SkinningData } from '../utils/skinning';
import { useInteraction } from '../adapter/useInteraction';

export const GarmentV4 = () => {
    const proxyGLTF = useGLTF('/shirt_proxy.glb');
    const visualGLTF = useGLTF('/shirt_visual.glb');

    const bridgeRef = useRef<Bridge>(new Bridge());
    const visualRef = useRef<THREE.Mesh | null>(null);

    const [visualGeometry, setVisualGeometry] = useState<THREE.BufferGeometry | null>(null);
    const [ready, setReady] = useState(false);

    const skinningRef = useRef<SkinningData | null>(null);
    const physicsGeoRef = useRef<THREE.BufferGeometry | null>(null);

    useEffect(() => {
        const proxyMesh = proxyGLTF.scene.getObjectByProperty('isMesh', true) as THREE.Mesh | undefined;
        const visualMesh = visualGLTF.scene.getObjectByProperty('isMesh', true) as THREE.Mesh | undefined;

        if (proxyMesh && visualMesh) {
            const rawProxyGeo = proxyMesh.geometry.clone();
            rawProxyGeo.deleteAttribute('normal');
            rawProxyGeo.deleteAttribute('uv');

            const weldedProxyGeo = BufferGeometryUtils.mergeVertices(rawProxyGeo, 0.001);
            console.log(`[Garment] Welded Proxy: ${rawProxyGeo.attributes.position.count} -> ${weldedProxyGeo.attributes.position.count} verts`);

            const visualGeo = visualMesh.geometry.clone();

            bridgeRef.current.init(weldedProxyGeo).then(() => {
                skinningRef.current = computeSkinning(visualGeo, weldedProxyGeo);
                physicsGeoRef.current = weldedProxyGeo;

                setVisualGeometry(visualGeo);
                setReady(true);
            });
        }
    }, [proxyGLTF, visualGLTF]);

    useFrame((_, delta) => {
        if (!ready || !visualRef.current || !bridgeRef.current.positions || !skinningRef.current || !physicsGeoRef.current) return;

        const dt = Math.min(delta, 0.032);
        bridgeRef.current.update(dt);

        const visualGeo = visualRef.current.geometry;
        const visualPos = visualGeo.attributes.position;
        const physicsPos = bridgeRef.current.positions;
        const physicsIndex = physicsGeoRef.current.index!;

        const { indices, weights } = skinningRef.current;

        const pA = new THREE.Vector3();
        const pB = new THREE.Vector3();
        const pC = new THREE.Vector3();

        for (let i = 0; i < visualPos.count; i++) {
            const faceIdx = indices[i];
            const w1 = weights[i * 3];
            const w2 = weights[i * 3 + 1];
            const w3 = weights[i * 3 + 2];

            const idxA = physicsIndex.getX(faceIdx * 3) * 3;
            const idxB = physicsIndex.getX(faceIdx * 3 + 1) * 3;
            const idxC = physicsIndex.getX(faceIdx * 3 + 2) * 3;

            if (idxA >= physicsPos.length || idxB >= physicsPos.length || idxC >= physicsPos.length) continue;

            pA.set(physicsPos[idxA], physicsPos[idxA + 1], physicsPos[idxA + 2]);
            pB.set(physicsPos[idxB], physicsPos[idxB + 1], physicsPos[idxB + 2]);
            pC.set(physicsPos[idxC], physicsPos[idxC + 1], physicsPos[idxC + 2]);

            const x = pA.x * w1 + pB.x * w2 + pC.x * w3;
            const y = pA.y * w1 + pB.y * w2 + pC.y * w3;
            const z = pA.z * w1 + pB.z * w2 + pC.z * w3;

            visualPos.setXYZ(i, x, y, z);
        }

        visualPos.needsUpdate = true;
        visualGeo.computeVertexNormals();

        // --- FIX: Update Bounding Sphere for Raycasting ---
        // Without this, the raycaster thinks the shirt is still at the origin,
        // so clicks on the moving shirt are ignored.
        visualGeo.computeBoundingSphere();
    });

    useInteraction(bridgeRef.current, visualRef);

    if (!visualGeometry) return null;

    return (
        <group>
            <mesh ref={visualRef} geometry={visualGeometry}>
                {/* UPGRADE: MeshPhysicalMaterial for cloth realism */}
                <meshPhysicalMaterial
                    color="#4488ff"
                    side={THREE.DoubleSide}
                    roughness={0.7}       // Fabric is rough
                    clearcoat={0.0}       // No plastic shine
                    sheen={1.0}           // Fabric sheen (velvet/cotton effect)
                    sheenColor="#ffffff"
                    thickness={0.01}      // Subsurface scattering hint
                />
            </mesh>

            <SDFVisualizer bridge={bridgeRef.current} />
        </group>
    );
};