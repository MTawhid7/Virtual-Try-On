// src/infrastructure/geometry/analysis/SpineRegressor.ts

import * as THREE from 'three';
import { SliceAnalyzer } from './slice/SliceAnalyzer';

export interface SpineData {
    neckCenter: THREE.Vector2;
    spineCenter: THREE.Vector2;
    pelvisCenter: THREE.Vector2;
}

export class SpineRegressor {
    /**
     * Estimates the spine position using linear regression on torso slices.
     * This provides a robust "Center Line" for the body, independent of arm position or pose.
     *
     * @param geometry - The body geometry.
     * @param minY - The logical minimum Y (feet/bottom).
     * @param height - The total logic height.
     */
    static analyze(
        geometry: THREE.BufferGeometry,
        minY: number,
        height: number
    ): SpineData {
        // ROI: Ribcage (60% to 85% up the body)
        const startY = minY + (height * 0.60);
        const endY = minY + (height * 0.85);

        const sliceCount = 15;
        const step = (endY - startY) / sliceCount;
        const points: THREE.Vector3[] = [];

        for (let i = 0; i < sliceCount; i++) {
            const y = startY + (i * step);
            const islands = SliceAnalyzer.getIslandsInSlice(geometry, y, y + 0.02);

            if (islands.length > 0) {
                // The largest island is the Torso. Smaller ones are arms.
                islands.sort((a, b) => b.size - a.size);
                points.push(islands[0].centroid);
            }
        }

        // Linear Regression (Least Squares)
        let sumY = 0, sumX = 0, sumZ = 0;
        let sumY2 = 0, sumXY = 0, sumYZ = 0;
        const N = points.length;

        if (N < 5) {
            return {
                neckCenter: new THREE.Vector2(0, 0),
                spineCenter: new THREE.Vector2(0, 0),
                pelvisCenter: new THREE.Vector2(0, 0)
            };
        }

        for (const p of points) {
            sumY += p.y; sumX += p.x; sumZ += p.z;
            sumY2 += p.y * p.y; sumXY += p.x * p.y; sumYZ += p.y * p.z;
        }

        const denom = (N * sumY2 - sumY * sumY);
        const slopeX = (N * sumXY - sumX * sumY) / denom;
        const slopeZ = (N * sumYZ - sumZ * sumY) / denom;
        const offsetX = (sumX - slopeX * sumY) / N;
        const offsetZ = (sumZ - slopeZ * sumY) / N;

        const getPoint = (y: number) => new THREE.Vector2(slopeX * y + offsetX, slopeZ * y + offsetZ);

        return {
            neckCenter: getPoint(minY + height * 0.87),
            spineCenter: getPoint(minY + height * 0.70),
            pelvisCenter: getPoint(minY + height * 0.50)
        };
    }
}