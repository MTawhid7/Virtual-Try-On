// src/infrastructure/geometry/analysis/orientation/PCASolver.ts

import * as THREE from 'three';

export class PCASolver {
    static computeCentroid(geometry: THREE.BufferGeometry): THREE.Vector3 {
        const pos = geometry.attributes.position;
        const c = new THREE.Vector3();
        let count = 0;
        const stride = 5;
        for (let i = 0; i < pos.count; i += stride) {
            c.x += pos.getX(i);
            c.y += pos.getY(i);
            c.z += pos.getZ(i);
            count++;
        }
        return c.divideScalar(count);
    }

    /**
     * Computes the primary axis of variance (Principal Component) using Power Iteration.
     * This avoids computationally expensive SVD/Eigendecomposition while being sufficient for finding the "Up" vector.
     */
    static computePrincipalAxis(geometry: THREE.BufferGeometry, centroid: THREE.Vector3): THREE.Vector3 {
        const pos = geometry.attributes.position;
        let xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
        const stride = 5;

        for (let i = 0; i < pos.count; i += stride) {
            const x = pos.getX(i) - centroid.x;
            const y = pos.getY(i) - centroid.y;
            const z = pos.getZ(i) - centroid.z;
            xx += x * x; xy += x * y; xz += x * z;
            yy += y * y; yz += y * z; zz += z * z;
        }

        // Power Iteration to find dominant eigenvector
        let v = new THREE.Vector3(0, 1, 0).normalize();
        for (let i = 0; i < 10; i++) {
            const next = new THREE.Vector3(
                v.x * xx + v.y * xy + v.z * xz,
                v.x * xy + v.y * yy + v.z * yz,
                v.x * xz + v.y * yz + v.z * zz
            );
            v = next.normalize();
        }
        return v;
    }

    static computePrincipalAxisXZ(geometry: THREE.BufferGeometry, centroid: THREE.Vector3): THREE.Vector3 {
        const pos = geometry.attributes.position;
        let xx = 0, xz = 0, zz = 0;
        const stride = 5;

        for (let i = 0; i < pos.count; i += stride) {
            const x = pos.getX(i) - centroid.x;
            const z = pos.getZ(i) - centroid.z;
            xx += x * x; xz += x * z; zz += z * z;
        }

        let v = new THREE.Vector3(1, 0, 0).normalize();
        for (let i = 0; i < 10; i++) {
            const next = new THREE.Vector3(
                v.x * xx + v.z * xz,
                0,
                v.x * xz + v.z * zz
            );
            v = next.normalize();
        }
        return v;
    }
}