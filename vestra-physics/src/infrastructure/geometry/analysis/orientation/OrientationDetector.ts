// src/infrastructure/geometry/analysis/orientation/OrientationDetector.ts

import * as THREE from 'three';
import { SliceAnalyzer } from '../slice/SliceAnalyzer';

/**
 * Detects the orientation of a humanoid mesh.
 * Specifically checks if the mesh is upside-down, which is a common issue with different 3D coordinate systems (Y-up vs Z-up).
 */
export class OrientationDetector {
    /**
     * Checks if the mesh is upside down.
     * Heuristic: A standing human has 1 island at the top (Head) and 2 at the bottom (Feet).
     * If we find 2 islands at the "top" and 1 at the "bottom", it's likely upside down.
     */
    static isUpsideDown(geometry: THREE.BufferGeometry, box: THREE.Box3): boolean {
        const height = box.max.y - box.min.y;

        // Analyze Top 5%
        const topIslands = SliceAnalyzer.getIslandsInSlice(
            geometry,
            box.max.y - (height * 0.05),
            box.max.y
        );

        // Analyze Bottom 5%
        const bottomIslands = SliceAnalyzer.getIslandsInSlice(
            geometry,
            box.min.y,
            box.min.y + (height * 0.05)
        );

        const topCount = topIslands.length;
        const bottomCount = bottomIslands.length;

        // If Top=2 (Feet) and Bottom=1 (Head), it's upside down.
        if (topCount >= 2 && bottomCount === 1) {
            return true;
        }

        return false;
    }
}