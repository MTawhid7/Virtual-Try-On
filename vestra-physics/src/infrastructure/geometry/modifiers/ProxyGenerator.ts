// src/infrastructure/geometry/modifiers/ProxyGenerator.ts

import * as THREE from 'three';
import { MeshoptSimplifier } from 'meshoptimizer';
import {
    GARMENT_RESOLUTION_BUDGET,
    COLLIDER_RESOLUTION_BUDGET
} from '../../../core/constants/SimulationConstants';
import type { ProcessedMesh } from '../../../core/entities/Geometry';

/**
 * Generates optimized proxy meshes for physics simulation.
 * uses `meshoptimizer` to decimate high-poly meshes into efficient low-poly versions
 * suitable for real-time cloth simulation and collision detection.
 */
export class ProxyGenerator {

    /**
     * Generates a collision proxy from the input mesh.
     * Decimates the mesh to `COLLIDER_RESOLUTION_BUDGET` if it's too high-poly.
     * collision meshes need enough detail to capture body shape but must be low-poly enough for performance.
     */
    static async generateCollider(mesh: THREE.Mesh): Promise<ProcessedMesh> {
        const triCount = this.getTriangleCount(mesh);

        if (triCount <= COLLIDER_RESOLUTION_BUDGET) {
            console.log(`[ProxyGenerator] Collider is efficient (${triCount} tris). Skipping decimation.`);
            return this.extractRaw(mesh);
        }

        console.log(`[ProxyGenerator] Collider is High-Poly (${triCount} tris). Decimating to ~${COLLIDER_RESOLUTION_BUDGET}...`);
        return this.process(mesh, COLLIDER_RESOLUTION_BUDGET, 0.01);
    }

    /**
     * Generates a garment simulation proxy.
     * Cloth simulation requires a specific topology balance:
     * - Too high poly: Simulation runs slow.
     * - Too low poly: Cloth looks blocky and lacks detail.
     * Decimates to `GARMENT_RESOLUTION_BUDGET` if needed.
     */
    static async generateGarment(mesh: THREE.Mesh): Promise<ProcessedMesh> {
        const triCount = this.getTriangleCount(mesh);

        if (triCount > GARMENT_RESOLUTION_BUDGET) {
            console.log(`[ProxyGenerator] Garment is High-Poly (${triCount} tris). Decimating to ~${GARMENT_RESOLUTION_BUDGET}...`);
            return this.process(mesh, GARMENT_RESOLUTION_BUDGET, 1e-3);
        } else {
            console.log(`[ProxyGenerator] Garment is Low-Poly (${triCount} tris). Using raw geometry.`);
            return this.extractRaw(mesh);
        }
    }

    private static getTriangleCount(mesh: THREE.Mesh): number {
        return (mesh.geometry.index ? mesh.geometry.index.count : mesh.geometry.attributes.position.count) / 3;
    }

    private static async process(mesh: THREE.Mesh, targetTriangles: number, errorThreshold: number): Promise<ProcessedMesh> {
        await MeshoptSimplifier.ready;

        const srcGeo = mesh.geometry;
        const srcPos = srcGeo.attributes.position.array as Float32Array;

        let srcIndices: Uint32Array;
        if (srcGeo.index) {
            srcIndices = srcGeo.index.array as Uint32Array;
        } else {
            const count = srcPos.length / 3;
            srcIndices = new Uint32Array(count);
            for (let i = 0; i < count; i++) srcIndices[i] = i;
        }


        let targetIndexCount = Math.floor(targetTriangles * 3);
        const requestedIndexCount = targetTriangles * 3;

        if (requestedIndexCount > srcIndices.length) {
            targetIndexCount = Math.floor(srcIndices.length * 0.8);
        }

        const [simplifiedIndices, _error] = MeshoptSimplifier.simplify(
            srcIndices,
            srcPos,
            3,
            targetIndexCount,
            errorThreshold
        );

        const { indices, vertices } = this.compactMesh(simplifiedIndices, srcPos);

        return {
            vertices,
            indices,
            normals: new Float32Array(vertices.length), // Placeholder, recalculated later by GeometryProcessor
            uvs: new Float32Array(0), // Placeholder
            tangents: new Float32Array(0) // Placeholder
        };
    }

    private static extractRaw(mesh: THREE.Mesh): ProcessedMesh {
        const geo = mesh.geometry;
        const positions = geo.attributes.position.array as Float32Array;
        const indices = geo.index ? (geo.index.array as Uint32Array) : new Uint32Array();
        const uvs = geo.attributes.uv ? (geo.attributes.uv.array as Float32Array) : new Float32Array();
        const normals = geo.attributes.normal ? (geo.attributes.normal.array as Float32Array) : new Float32Array();

        // If no indices exist (unindexed mesh), generate them
        if (indices.length === 0) {
            const count = positions.length / 3;
            const newIndices = new Uint32Array(count);
            for (let i = 0; i < count; i++) newIndices[i] = i;

            return {
                vertices: positions,
                indices: newIndices,
                normals: normals,
                uvs: uvs,
                tangents: new Float32Array(0)
            };
        }

        return {
            vertices: positions,
            indices: indices,
            normals: normals,
            uvs: uvs,
            tangents: new Float32Array(0)
        };
    }

    private static compactMesh(indices: Uint32Array, sourcePositions: Float32Array) {
        const vertexCount = sourcePositions.length / 3;
        const map = new Int32Array(vertexCount).fill(-1);
        let nextNewIndex = 0;
        const newIndices = new Uint32Array(indices.length);
        const tempVerts = new Float32Array(indices.length * 3);

        for (let i = 0; i < indices.length; i++) {
            const oldIdx = indices[i];
            let newIdx = map[oldIdx];
            if (newIdx === -1) {
                newIdx = nextNewIndex;
                map[oldIdx] = newIdx;
                tempVerts[newIdx * 3] = sourcePositions[oldIdx * 3];
                tempVerts[newIdx * 3 + 1] = sourcePositions[oldIdx * 3 + 1];
                tempVerts[newIdx * 3 + 2] = sourcePositions[oldIdx * 3 + 2];
                nextNewIndex++;
            }
            newIndices[i] = newIdx;
        }
        return { indices: newIndices, vertices: tempVerts.slice(0, nextNewIndex * 3) };
    }
}