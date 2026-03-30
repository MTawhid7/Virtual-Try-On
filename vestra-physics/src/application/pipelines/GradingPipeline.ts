// src/application/pipelines/GradingPipeline.ts

import * as THREE from 'three';
import { SHIRT_STANDARDS } from '../../core/constants/SizingStandards';
import type { ShirtSize } from '../../core/entities/Garment';
import { MeshMeasurer } from '../../infrastructure/geometry/analysis/MeshMeasurer';
import type { ProcessedMesh } from '../../core/entities/Geometry';

export class GradingPipeline {
    /**
     * Applies sizing logic to a garment mesh, transforming vertices based on standard shirt sizes.
     * This is an isolated, stateless operation that returns new vertex data without modifying the source mesh.
     *
     * @param baseMesh - The original, unscaled garment mesh (ProcessedMesh).
     * @param targetSize - The desired shirt size (XS - XXL) from the standard sizing chart.
     * @returns An object containing the new scaled vertices (Float32Array) and the physics scale factor.
     */
    static execute(baseMesh: ProcessedMesh, targetSize: ShirtSize): { vertices: Float32Array, scaleFactor: number } {
        // 1. Reconstruct a temporary THREE.Mesh to measure dimensions
        // We do this to reuse the robust bounding box logic in MeshMeasurer.
        // Since ProcessedMesh is just arrays (vertices, indices), we wrap it briefly in a BufferGeometry.
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(baseMesh.vertices, 3));
        const tempMesh = new THREE.Mesh(geometry);

        // 2. Measure the Base Mesh
        // This gives us the raw width and length of the mesh in local units.
        const rawDimensions = MeshMeasurer.measure(tempMesh);

        // 3. Calculate Scale Factors
        const target = SHIRT_STANDARDS[targetSize];

        // Scale = Target Dimension / Raw Dimension
        const scaleX = target.chestWidth / rawDimensions.width;
        const scaleY = target.bodyLength / rawDimensions.length;

        // Z (Depth) scales with Width (Chest) to maintain aspect ratio and prevent distortion.
        const scaleZ = scaleX;

        console.log(`[GradingPipeline] Applying Size ${targetSize}: Scale X/Z=${scaleX.toFixed(3)}, Y=${scaleY.toFixed(3)}`);

        // 4. Apply Scaling to Vertices
        const count = baseMesh.vertices.length;
        const scaledVertices = new Float32Array(count);

        // Find Max Y (Neck) to use as the pivot point.
        // We scale length *downwards* from the neck, rather than from the center, so the collar stays at the same height.
        let maxY = -Infinity;
        for (let i = 1; i < count; i += 3) {
            if (baseMesh.vertices[i] > maxY) maxY = baseMesh.vertices[i];
        }

        for (let i = 0; i < count; i += 3) {
            const x = baseMesh.vertices[i];
            const y = baseMesh.vertices[i + 1];
            const z = baseMesh.vertices[i + 2];

            // Scale Width (X) and Depth (Z) from Center (0,0)
            scaledVertices[i] = x * scaleX;
            scaledVertices[i + 2] = z * scaleZ;

            // Scale Height (Y) using the Neck (maxY) as the anchor/pivot.
            // Formula: NewY = Anchor + (CurrentY - Anchor) * ScaleFactor
            scaledVertices[i + 1] = maxY + (y - maxY) * scaleY;
        }

        // 5. Calculate Physics Scale Factor
        // This factor is passed to the physics engine to roughly approximate the change in mass/stiffness.
        // A larger shirt is generally heavier and potentially "floppier"; this factor helps compensate.
        const physicsScale = (scaleX + scaleY) / 2;

        return { vertices: scaledVertices, scaleFactor: physicsScale };
    }
}