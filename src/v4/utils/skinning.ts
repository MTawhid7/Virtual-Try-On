// src/v4/utils/skinning.ts
import * as THREE from 'three';
import { computeBoundsTree, disposeBoundsTree, acceleratedRaycast } from 'three-mesh-bvh';

// Patch Three.js
(THREE.BufferGeometry.prototype as any).computeBoundsTree = computeBoundsTree;
(THREE.BufferGeometry.prototype as any).disposeBoundsTree = disposeBoundsTree;
(THREE.Mesh.prototype as any).raycast = acceleratedRaycast;

export interface SkinningData {
    indices: Int32Array;
    weights: Float32Array;
}

export const computeSkinning = (visualGeo: THREE.BufferGeometry, physicsGeo: THREE.BufferGeometry): SkinningData => {
    const visualPos = visualGeo.attributes.position;

    // Ensure physics mesh has BVH
    if (!(physicsGeo as any).boundsTree) {
        (physicsGeo as any).computeBoundsTree();
    }

    const vertexCount = visualPos.count;
    const indices = new Int32Array(vertexCount);
    const weights = new Float32Array(vertexCount * 3);

    const tempPos = new THREE.Vector3();
    const tempHit = { point: new THREE.Vector3(), distance: 0, faceIndex: -1 };

    for (let i = 0; i < vertexCount; i++) {
        tempPos.fromBufferAttribute(visualPos, i);

        // Find closest triangle on physics mesh
        const hit = (physicsGeo as any).boundsTree.closestPointToPoint(tempPos, tempHit);

        if (hit) {
            indices[i] = hit.faceIndex;

            // Barycentric coordinates
            const face = physicsGeo.index!;
            const ia = face.getX(hit.faceIndex * 3);
            const ib = face.getX(hit.faceIndex * 3 + 1);
            const ic = face.getX(hit.faceIndex * 3 + 2);

            const va = new THREE.Vector3().fromBufferAttribute(physicsGeo.attributes.position, ia);
            const vb = new THREE.Vector3().fromBufferAttribute(physicsGeo.attributes.position, ib);
            const vc = new THREE.Vector3().fromBufferAttribute(physicsGeo.attributes.position, ic);

            const tri = new THREE.Triangle(va, vb, vc);
            const bary = new THREE.Vector3();
            tri.getBarycoord(hit.point, bary);

            weights[i * 3] = bary.x;
            weights[i * 3 + 1] = bary.y;
            weights[i * 3 + 2] = bary.z;
        } else {
            indices[i] = 0;
            weights[i * 3] = 1; weights[i * 3 + 1] = 0; weights[i * 3 + 2] = 0;
        }
    }

    return { indices, weights };
};