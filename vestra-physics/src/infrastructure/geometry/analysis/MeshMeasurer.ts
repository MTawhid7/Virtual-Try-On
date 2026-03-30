// src/infrastructure/geometry/analysis/MeshMeasurer.ts

import * as THREE from 'three';

/**
 * Utility for measuring physical dimensions of a mesh.
 * Used to determine the size/bounding box of garments or bodies.
 */
export class MeshMeasurer {
    /**
     * Calculates the width and length of a mesh in Centimeters (cm).
     * Assumes the input mesh is in Meters.
     */
    static measure(mesh: THREE.Mesh): { width: number, length: number } {
        mesh.geometry.computeBoundingBox();
        const box = mesh.geometry.boundingBox!;

        // Calculate Dimensions in Meters
        const widthMeters = box.max.x - box.min.x;
        const lengthMeters = box.max.y - box.min.y;

        // Convert to CM
        return {
            width: widthMeters * 100,
            length: lengthMeters * 100
        };
    }
}