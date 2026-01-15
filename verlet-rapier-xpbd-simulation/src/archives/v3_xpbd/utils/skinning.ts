// src/v3/utils/skinning.ts
import * as THREE from 'three';
import { computeBoundsTree, disposeBoundsTree, acceleratedRaycast } from 'three-mesh-bvh';

// Patch Three.js prototypes for BVH (Runtime logic)
// Use 'as any' to avoid strict type checks during assignment
(THREE.BufferGeometry.prototype as any).computeBoundsTree = computeBoundsTree;
(THREE.BufferGeometry.prototype as any).disposeBoundsTree = disposeBoundsTree;
(THREE.Mesh.prototype as any).raycast = acceleratedRaycast;

export interface SkinningData {
    indices: Int32Array;
    weights: Float32Array;
}

export const computeSkinning = (visualMesh: THREE.Mesh, physicsMesh: THREE.Mesh): SkinningData => {
    const visualPos = visualMesh.geometry.attributes.position;
    const physicsGeo = physicsMesh.geometry;

    // 1. Ensure physics mesh has a BVH for fast lookup
    // Check if boundsTree exists (it's added by the patch above)
    if (!(physicsGeo as any).boundsTree) {
        (physicsGeo as any).computeBoundsTree();
    }

    const vertexCount = visualPos.count;
    const indices = new Int32Array(vertexCount);
    const weights = new Float32Array(vertexCount * 3);

    const tempPos = new THREE.Vector3();
    const tempHit = { point: new THREE.Vector3(), distance: 0, faceIndex: -1 };

    // 2. Loop through every vertex of the High-Res Visual Mesh
    for (let i = 0; i < vertexCount; i++) {
        tempPos.fromBufferAttribute(visualPos, i);

        // Find the closest point on the Low-Res Proxy
        // We cast to 'any' here because the type definition might vary slightly between versions
        const hit = (physicsGeo as any).boundsTree.closestPointToPoint(tempPos, tempHit);

        if (hit) {
            // A. Store which triangle handles this vertex
            indices[i] = hit.faceIndex;

            // B. Calculate Barycentric Weights
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
            // Fallback
            indices[i] = 0;
            weights[i * 3] = 1; weights[i * 3 + 1] = 0; weights[i * 3 + 2] = 0;
        }
    }

    return { indices, weights };
};