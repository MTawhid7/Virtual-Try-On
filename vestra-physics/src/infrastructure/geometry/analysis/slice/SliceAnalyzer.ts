// src/infrastructure/geometry/analysis/slice/SliceAnalyzer.ts
import * as THREE from 'three';
import { ClusterEngine, type Cluster } from './ClusterEngine';

export type { Cluster };

/**
 * Analyzes cross-sections (slices) of 3D geometry.
 * Used to understand the topology of the body at specific heights (Y-levels).
 */
export class SliceAnalyzer {
    /**
     * Extracts distinct "islands" of geometry within a vertical slice.
     * Useful for detecting arms vs torso or legs vs feet.
     *
     * @param geometry - The input geometry.
     * @param minY - The bottom Y coordinate of the slice.
     * @param maxY - The top Y coordinate of the slice.
     * @param stride - Optimization: Skip every N vertices for speed (default 2).
     */
    static getIslandsInSlice(
        geometry: THREE.BufferGeometry,
        minY: number,
        maxY: number,
        stride: number = 2
    ): Cluster[] {
        const pos = geometry.attributes.position;
        const points: THREE.Vector3[] = [];

        for (let i = 0; i < pos.count; i += stride) {
            const y = pos.getY(i);
            if (y >= minY && y <= maxY) {
                points.push(new THREE.Vector3(pos.getX(i), y, pos.getZ(i)));
            }
        }

        return ClusterEngine.findClusters(points);
    }

    /**
     * Calculates the bounding width (Max X - Min X) of the geometry in a specific Y-slice.
     * Used for orientation heuristics (Shoulders are wider than Ankles).
     */
    static getSliceWidth(
        geometry: THREE.BufferGeometry,
        minY: number,
        maxY: number,
        stride: number = 5
    ): number {
        const pos = geometry.attributes.position;
        let minX = Infinity;
        let maxX = -Infinity;
        let found = false;

        for (let i = 0; i < pos.count; i += stride) {
            const y = pos.getY(i);
            if (y >= minY && y <= maxY) {
                const x = pos.getX(i);
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                found = true;
            }
        }
        return found ? (maxX - minX) : 0;
    }
}