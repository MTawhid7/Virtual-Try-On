// src/infrastructure/geometry/analysis/BodyMeasurer.ts

import * as THREE from 'three';
import { ClusterEngine } from './slice/ClusterEngine';

export class BodyMeasurer {
    /**
     * Calculates the chest circumference of the mesh in cm.
     * Uses an adaptive clustering strategy to robustly isolate the torso
     * even in tight A-Poses where arms are close to the body.
     */
    static getChestCircumference(mesh: THREE.Mesh): number {
        const geometry = mesh.geometry;
        if (!geometry.boundingBox) geometry.computeBoundingBox();
        const box = geometry.boundingBox!;
        const height = box.max.y - box.min.y;

        // Chest is typically at ~73% of total height
        const chestHeight = box.min.y + (height * 0.73);
        const sliceThickness = 0.02;

        // 1. Extract Points
        const points3D: THREE.Vector3[] = [];
        const pos = geometry.attributes.position;
        const v = new THREE.Vector3();

        for (let i = 0; i < pos.count; i++) {
            v.fromBufferAttribute(pos, i);
            if (Math.abs(v.y - chestHeight) < sliceThickness) {
                points3D.push(v.clone());
            }
        }

        if (points3D.length < 3) {
            console.warn("[BodyMeasurer] Not enough points in chest slice.");
            return 100.0; // Fallback
        }

        // 2. Adaptive Clustering Loop
        // We try progressively smaller thresholds.
        // 0.05 (5cm) -> Standard T-Pose
        // 0.015 (1.5cm) -> Tight A-Pose
        // 0.01 (1cm) -> Very tight / High Poly
        const thresholds = [0.05, 0.03, 0.015, 0.01];

        for (const threshold of thresholds) {
            const clusters = ClusterEngine.findClusters(points3D, threshold);

            if (clusters.length === 0) continue;

            // Pick largest cluster (The Torso is always bigger than an Arm)
            clusters.sort((a, b) => b.points.length - a.points.length);
            const torso = clusters[0];

            // Measure
            const perimeter = this.calculatePerimeter(torso.points);

            // 3. Sanity Check
            // A human chest is rarely > 150cm (approx 60 inches).
            // If it's huge, it implies the arms are still merged (Zigzag error).
            if (perimeter < 150) {
                console.log(`[BodyMeasurer] Success at threshold ${threshold}m. Islands: ${clusters.length}. Perimeter: ${perimeter.toFixed(1)}cm`);
                return perimeter;
            }

            console.warn(`[BodyMeasurer] Threshold ${threshold}m yielded suspicious perimeter ${perimeter.toFixed(1)}cm (likely merged arms). Retrying with tighter gap...`);
        }

        console.error("[BodyMeasurer] Failed to isolate torso after all attempts. Returning default.");
        return 100.0; // Safe default (Size M)
    }

    /**
     * Calculates the perimeter of a polygon defined by a set of 3D points.
     * Projects points to 2D (XZ plane), sorts them angularly around the centroid to form a closed loop,
     * and sums the edge lengths.
     */
    private static calculatePerimeter(points: THREE.Vector3[]): number {
        const points2D = points.map(p => new THREE.Vector2(p.x, p.z));

        // Sort by angle around centroid to create a continuous path
        const centroid = new THREE.Vector2();
        for (const p of points2D) centroid.add(p);
        centroid.divideScalar(points2D.length);

        points2D.sort((a, b) => {
            const angA = Math.atan2(a.y - centroid.y, a.x - centroid.x);
            const angB = Math.atan2(b.y - centroid.y, b.x - centroid.x);
            return angA - angB;
        });

        let perimeter = 0;
        for (let i = 0; i < points2D.length; i++) {
            const p1 = points2D[i];
            const p2 = points2D[(i + 1) % points2D.length];
            perimeter += p1.distanceTo(p2);
        }

        return perimeter * 100; // Convert Meters to CM
    }
}