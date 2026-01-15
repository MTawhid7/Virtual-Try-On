// ... imports same as before
import { Mesh, Vector3, Matrix4, BufferAttribute } from 'three';
import { MeshBVH, acceleratedRaycast, computeBoundsTree, disposeBoundsTree } from 'three-mesh-bvh';
import * as THREE from 'three';
import type { Particle } from '../types';

// Patch Three.js
THREE.BufferGeometry.prototype.computeBoundsTree = computeBoundsTree;
THREE.BufferGeometry.prototype.disposeBoundsTree = disposeBoundsTree;
THREE.Mesh.prototype.raycast = acceleratedRaycast;

export class BVHCollider {
    private bvh: MeshBVH | null = null;
    private mesh: Mesh | null = null;
    private inverseMatrix = new Matrix4();
    private tempPos = new Vector3();
    private tempHit = { point: new Vector3(), distance: 0, faceIndex: -1 };

    private faceNormal = new Vector3();
    private tempNormal = new Vector3();

    setCollider(mesh: Mesh) {
        this.mesh = mesh;
        mesh.updateMatrixWorld(true);

        if (!mesh.geometry.boundsTree) {
            mesh.geometry.computeBoundsTree();
        }
        this.bvh = mesh.geometry.boundsTree || null;
    }

    solve(particles: Particle[]) {
        if (!this.bvh || !this.mesh) return;

        const thickness = 0.02;
        // FIX: Increased friction to prevent sliding (0.9 = very grippy)
        const friction = 0.9;

        this.inverseMatrix.copy(this.mesh.matrixWorld).invert();

        const indexAttr = this.mesh.geometry.index;
        const normalAttr = this.mesh.geometry.attributes.normal;

        for (const p of particles) {
            this.tempPos.copy(p.position).applyMatrix4(this.inverseMatrix);

            const hit = this.bvh.closestPointToPoint(this.tempPos, this.tempHit);

            if (hit) {
                if (indexAttr) {
                    const i3 = hit.faceIndex * 3;
                    const a = indexAttr.getX(i3);
                    const b = indexAttr.getX(i3 + 1);
                    const c = indexAttr.getX(i3 + 2);

                    this.faceNormal.set(0, 0, 0);
                    this.tempNormal.fromBufferAttribute(normalAttr as BufferAttribute, a);
                    this.faceNormal.add(this.tempNormal);
                    this.tempNormal.fromBufferAttribute(normalAttr as BufferAttribute, b);
                    this.faceNormal.add(this.tempNormal);
                    this.tempNormal.fromBufferAttribute(normalAttr as BufferAttribute, c);
                    this.faceNormal.add(this.tempNormal);
                    this.faceNormal.normalize();
                }

                const vecToParticle = this.tempPos.clone().sub(hit.point);
                const distanceToPlane = vecToParticle.dot(this.faceNormal);

                if (distanceToPlane < thickness) {
                    this.tempPos.copy(hit.point).add(this.faceNormal.clone().multiplyScalar(thickness));
                    const newWorldPos = this.tempPos.clone().applyMatrix4(this.mesh.matrixWorld);

                    p.position.copy(newWorldPos);

                    const velocity = p.position.clone().sub(p.prevPosition);
                    velocity.multiplyScalar(1 - friction);
                    p.prevPosition.copy(p.position).sub(velocity);
                }
            }
        }
    }
}