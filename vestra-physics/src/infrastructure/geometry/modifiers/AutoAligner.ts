// src/infrastructure/geometry/modifiers/AutoAligner.ts

import * as THREE from 'three';
import { MeshAnalyzer } from '../analysis/MeshAnalyzer';

/**
 * Automatically aligns geometry to a standardized coordinate system.
 * Ensures that imported meshes (Body and Garment) are correctly positioned relative to each other
 * and the world origin (0,0,0) before simulation starts.
 */
export class AutoAligner {
    /**
     * Centers the body mesh on (0,0,0) and places the feet on the ground (Y=0).
     */
    static alignBody(geometry: THREE.BufferGeometry): THREE.Box3 {
        // 1. Compute Box
        geometry.computeBoundingBox();
        const box = geometry.boundingBox!;

        // 2. Center X and Z on (0,0)
        const centerX = (box.min.x + box.max.x) / 2;
        const centerZ = (box.min.z + box.max.z) / 2;

        // 3. Floor Y on 0
        const offsetY = -box.min.y;

        geometry.translate(-centerX, offsetY, -centerZ);
        geometry.computeBoundingBox();

        return geometry.boundingBox!;
    }

    /**
     * Aligns the garment to the body's neck position.
     * Uses anatomical analysis to find the true neck, even if the body is leaning.
     */
    static alignGarmentToBody(garmentGeo: THREE.BufferGeometry, bodyGeo: THREE.BufferGeometry) {
        // 1. Analyze Body for Neck Height
        const anchors = MeshAnalyzer.analyzeBody(bodyGeo);

        garmentGeo.computeBoundingBox();
        const gBox = garmentGeo.boundingBox!;

        // 2. Center Shirt X/Z on (0,0)
        // Since Body is at (0,0), this aligns them perfectly.
        const shirtCenterX = (gBox.min.x + gBox.max.x) / 2;
        const shirtCenterZ = (gBox.min.z + gBox.max.z) / 2;

        const moveX = -shirtCenterX;
        const moveZ = -shirtCenterZ;

        // 3. Align Height (Neck to Neck)
        const shirtTopY = gBox.max.y;

        // Use detected neck Y, or fallback to Body Top if analysis fails
        bodyGeo.computeBoundingBox();
        const bodyTopY = bodyGeo.boundingBox!.max.y;
        const targetY = (anchors.neckY > 0) ? anchors.neckY : (bodyTopY * 0.87);

        const moveY = (targetY - shirtTopY) + 0.02; // 2cm buffer

        garmentGeo.translate(moveX, moveY, moveZ);

        garmentGeo.computeBoundingBox();
        garmentGeo.computeBoundingSphere();
    }
}