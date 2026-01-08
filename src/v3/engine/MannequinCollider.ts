// src/v3/engine/MannequinCollider.ts
import * as THREE from 'three';
import { MeshBVH, acceleratedRaycast, computeBoundsTree, disposeBoundsTree } from 'three-mesh-bvh';

// --- FIX: Apply the patches so 'acceleratedRaycast' is used ---
(THREE.BufferGeometry.prototype as any).computeBoundsTree = computeBoundsTree;
(THREE.BufferGeometry.prototype as any).disposeBoundsTree = disposeBoundsTree;
(THREE.Mesh.prototype as any).raycast = acceleratedRaycast;

export interface CollisionResult {
    collided: boolean;
    rescued: boolean;
    normal: THREE.Vector3;
}

export class MannequinCollider {
    private bvh: MeshBVH | null = null;
    private mesh: THREE.Mesh | null = null;
    private inverseMatrix = new THREE.Matrix4();
    private tempVec = new THREE.Vector3();
    private tempTarget = { point: new THREE.Vector3(), distance: 0, faceIndex: -1 };
    private ray = new THREE.Ray();
    private rayDir = new THREE.Vector3(0.577, 0.577, 0.577).normalize();
    private tempNormal = new THREE.Vector3();
    private tempTri = new THREE.Triangle();

    // Tuning
    private readonly SURFACE_OFFSET = 0.015;
    private readonly RESCUE_OFFSET = 0.02;

    public setMesh(mesh: THREE.Mesh) {
        this.mesh = mesh;
        // Ensure BVH is computed
        if (!(this.mesh.geometry as any).boundsTree) {
            (this.mesh.geometry as any).computeBoundsTree();
        }
        this.bvh = (this.mesh.geometry as any).boundsTree;
    }

    public updateMatrix() {
        if (this.mesh) {
            this.mesh.updateMatrixWorld();
            this.inverseMatrix.copy(this.mesh.matrixWorld).invert();
        }
    }

    public collide(position: THREE.Vector3): CollisionResult {
        const result: CollisionResult = {
            collided: false,
            rescued: false,
            normal: new THREE.Vector3(0, 1, 0)
        };

        if (!this.bvh || !this.mesh) return result;

        const localPos = this.tempVec.copy(position).applyMatrix4(this.inverseMatrix);
        const hit = this.bvh.closestPointToPoint(localPos, this.tempTarget);
        if (!hit) return result;

        this.getFaceNormal(hit.faceIndex, this.tempNormal);

        this.ray.origin.copy(localPos);
        this.ray.direction.copy(this.rayDir);
        const rayHits = this.bvh.raycast(this.ray, THREE.DoubleSide);
        const isInside = rayHits.length % 2 !== 0;

        const dist = localPos.distanceTo(hit.point);

        if (isInside) {
            const targetLocal = hit.point.clone().addScaledVector(this.tempNormal, this.RESCUE_OFFSET);
            position.copy(targetLocal).applyMatrix4(this.mesh.matrixWorld);
            result.normal.copy(this.tempNormal).transformDirection(this.mesh.matrixWorld).normalize();
            result.collided = true;
            result.rescued = true;
        }
        else if (dist < this.SURFACE_OFFSET) {
            const targetLocal = hit.point.clone().addScaledVector(this.tempNormal, this.SURFACE_OFFSET);
            position.copy(targetLocal).applyMatrix4(this.mesh.matrixWorld);
            result.normal.copy(this.tempNormal).transformDirection(this.mesh.matrixWorld).normalize();
            result.collided = true;
            result.rescued = false;
        }

        return result;
    }

    private getFaceNormal(faceIndex: number, target: THREE.Vector3) {
        if (!this.mesh) return;
        const geo = this.mesh.geometry;
        const pos = geo.attributes.position;
        const idx = geo.index;

        const getIdx = (i: number) => idx ? idx.getX(i) : i;

        const a = getIdx(faceIndex * 3);
        const b = getIdx(faceIndex * 3 + 1);
        const c = getIdx(faceIndex * 3 + 2);

        this.tempTri.setFromAttributeAndIndices(pos, a, b, c);
        this.tempTri.getNormal(target);
    }
}