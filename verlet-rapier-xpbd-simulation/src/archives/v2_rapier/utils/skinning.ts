import * as THREE from 'three';
import { acceleratedRaycast, computeBoundsTree, disposeBoundsTree } from 'three-mesh-bvh';

// Enable BVH extensions
THREE.Mesh.prototype.raycast = acceleratedRaycast;
THREE.BufferGeometry.prototype.computeBoundsTree = computeBoundsTree;
THREE.BufferGeometry.prototype.disposeBoundsTree = disposeBoundsTree;

export interface SkinningData {
    indices: Int32Array; // Stores the triangle index for each visual vertex
    weights: Float32Array; // Stores the barycentric weights (u, v, w)
}

export const computeSkinning = (visualMesh: THREE.Mesh, physicsMesh: THREE.Mesh): SkinningData => {
    const visualPos = visualMesh.geometry.attributes.position;
    const physicsGeo = physicsMesh.geometry;

    // 1. Ensure physics mesh has a BVH for fast lookup
    if (!physicsGeo.boundsTree) {
        physicsGeo.computeBoundsTree();
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
        const hit = physicsGeo.boundsTree!.closestPointToPoint(tempPos, tempHit);

        if (hit) {
            // A. Store which triangle handles this vertex
            indices[i] = hit.faceIndex;

            // B. Calculate Barycentric Weights (How close is it to corner A vs B vs C?)
            // We need the 3 vertices of the proxy triangle
            const face = physicsGeo.index!;
            const ia = face.getX(hit.faceIndex * 3);
            const ib = face.getY(hit.faceIndex * 3);
            const ic = face.getZ(hit.faceIndex * 3);

            const va = new THREE.Vector3().fromBufferAttribute(physicsGeo.attributes.position, ia);
            const vb = new THREE.Vector3().fromBufferAttribute(physicsGeo.attributes.position, ib);
            const vc = new THREE.Vector3().fromBufferAttribute(physicsGeo.attributes.position, ic);

            const tri = new THREE.Triangle(va, vb, vc);
            const bary = new THREE.Vector3();
            tri.getBarycoord(hit.point, bary); // Populates 'bary' with u, v, w

            weights[i * 3] = bary.x;
            weights[i * 3 + 1] = bary.y;
            weights[i * 3 + 2] = bary.z;
        } else {
            // Fallback (should never happen if proxy encloses visual)
            indices[i] = 0;
            weights[i * 3] = 1; weights[i * 3 + 1] = 0; weights[i * 3 + 2] = 0;
        }
    }

    return { indices, weights };
};