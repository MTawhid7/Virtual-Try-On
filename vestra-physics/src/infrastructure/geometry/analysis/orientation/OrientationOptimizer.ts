// src/infrastructure/geometry/analysis/orientation/OrientationOptimizer.ts

import * as THREE from 'three';
import { PCASolver } from './PCASolver';
import { ValidationUtils } from './ValidationUtils';
import { SliceAnalyzer } from '../slice/SliceAnalyzer';

// --- VOTERS (Internal Logic) ---

// --- VOTERS (Internal Logic) ---

/**
 * Heuristic: The chest mass center is typically pushed 'forward' relative to the geometric center
 * due to the pectoral muscles/bust.
 */
class ChestVoter {
    static vote(geometry: THREE.BufferGeometry, box: THREE.Box3): { forward: number, backward: number } {
        const height = box.max.y - box.min.y;
        const chestMinY = box.min.y + (height * 0.65);
        const chestMaxY = box.min.y + (height * 0.75);

        const data = ValidationUtils.getSliceCentroidAndBounds(geometry, chestMinY, chestMaxY);
        if (!data) return { forward: 0, backward: 0 };

        const geoCenterZ = (data.minZ + data.maxZ) / 2;
        const massCenterZ = data.centroidZ;
        const bias = massCenterZ - geoCenterZ;

        if (bias > 0.005) return { forward: 1, backward: 0 };
        if (bias < -0.005) return { forward: 0, backward: 1 };
        return { forward: 0, backward: 0 };
    }
}

/**
 * Heuristic: Feet typically protrude significantly 'forward' from the body's core axis.
 */
class FeetVoter {
    static vote(geometry: THREE.BufferGeometry, box: THREE.Box3, coreZ: number): { forward: number, backward: number } {
        const height = box.max.y - box.min.y;
        const feetMinY = box.min.y;
        const feetMaxY = box.min.y + (height * 0.05);

        const extremes = ValidationUtils.getZExtremesInSlice(geometry, feetMinY, feetMaxY);
        if (!extremes) return { forward: 0, backward: 0 };

        const distPlus = Math.abs(extremes.max - coreZ);
        const distMinus = Math.abs(coreZ - extremes.min);

        if (distPlus > distMinus * 1.2) return { forward: 1, backward: 0 };
        if (distMinus > distPlus * 1.2) return { forward: 0, backward: 1 };
        return { forward: 0, backward: 0 };
    }
}

/**
 * Heuristic: The face/head typically protrudes 'forward' relative to the neck.
 */
class HeadVoter {
    static vote(geometry: THREE.BufferGeometry, box: THREE.Box3, coreZ: number): { forward: number, backward: number } {
        const height = box.max.y - box.min.y;
        const headMinY = box.min.y + (height * 0.88);
        const headMaxY = box.max.y;

        const extremes = ValidationUtils.getZExtremesInSlice(geometry, headMinY, headMaxY);
        if (!extremes) return { forward: 0, backward: 0 };

        const distPlus = Math.abs(extremes.max - coreZ);
        const distMinus = Math.abs(coreZ - extremes.min);

        if (distPlus > distMinus * 1.1) return { forward: 2, backward: 0 };
        if (distMinus > distPlus * 1.1) return { forward: 0, backward: 2 };
        return { forward: 0, backward: 0 };
    }
}

// --- MAIN CLASS ---

export class OrientationOptimizer {
    static alignAxes(mesh: THREE.Mesh) {
        const geometry = mesh.geometry;

        // 1. Vertical (Spine -> Y)
        const centroid = PCASolver.computeCentroid(geometry);
        const axisY = PCASolver.computePrincipalAxis(geometry, centroid);
        const targetY = new THREE.Vector3(0, 1, 0);
        const quatY = new THREE.Quaternion().setFromUnitVectors(axisY, targetY);
        this.applyRotation(geometry, centroid, quatY);

        // 2. Horizontal (Shoulders -> X)
        const centroid2 = PCASolver.computeCentroid(geometry);
        const axisX = PCASolver.computePrincipalAxisXZ(geometry, centroid2);
        const targetX = new THREE.Vector3(1, 0, 0);
        const quatX = new THREE.Quaternion().setFromUnitVectors(axisX, targetX);
        this.applyRotation(geometry, centroid2, quatX);
    }

    static validateOrientation(mesh: THREE.Mesh) {
        const geometry = mesh.geometry;
        geometry.computeBoundingBox();
        const box = geometry.boundingBox!;
        const height = box.max.y - box.min.y;

        // 1. Check Up/Down (Vertical Validator)
        const topWidth = SliceAnalyzer.getSliceWidth(geometry, box.max.y - height * 0.2, box.max.y);
        const bottomWidth = SliceAnalyzer.getSliceWidth(geometry, box.min.y, box.min.y + height * 0.2);

        if (bottomWidth > 0.05 && topWidth < bottomWidth) {
            console.log("[OrientationOptimizer] Upside Down Detected. Flipping 180° on Z.");
            ValidationUtils.rotateGeometry(geometry, 'z', Math.PI);
        }

        // 2. Check Front/Back (Forward Validator)
        geometry.computeBoundingBox(); // Recompute after potential flip
        const newBox = geometry.boundingBox!;
        const coreZ = ValidationUtils.getCoreZ(geometry, newBox.min.y, height);

        let forwardScore = 0;
        let backwardScore = 0;

        const head = HeadVoter.vote(geometry, newBox, coreZ);
        const feet = FeetVoter.vote(geometry, newBox, coreZ);
        const chest = ChestVoter.vote(geometry, newBox);

        forwardScore = head.forward + feet.forward + chest.forward;
        backwardScore = head.backward + feet.backward + chest.backward;

        if (backwardScore > forwardScore) {
            console.log("[OrientationOptimizer] Backwards Detected. Rotating Y 180°.");
            ValidationUtils.rotateGeometry(geometry, 'y', Math.PI);
        }
    }

    private static applyRotation(geometry: THREE.BufferGeometry, center: THREE.Vector3, q: THREE.Quaternion) {
        geometry.translate(-center.x, -center.y, -center.z);
        geometry.applyQuaternion(q);
        geometry.translate(center.x, center.y, center.z);
        geometry.computeBoundingBox();
    }
}