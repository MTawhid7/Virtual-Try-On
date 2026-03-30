// src/infrastructure/geometry/processors/GeometryProcessor.ts

import * as THREE from 'three';
import * as BufferGeometryUtils from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { GEOMETRY_WELD_THRESHOLD } from '../../../core/constants/SimulationConstants';
import type { ProcessedMesh } from '../../../core/entities/Geometry';

export class GeometryProcessor {
    /**
     * Vertices within this threshold are merged.
     * Includes a feature to preserve sharp normals if the dot product is distinct enough.
     */
    static smartWeld(geometry: THREE.BufferGeometry, threshold: number = 0.05): THREE.BufferGeometry {
        let geo = geometry.clone();
        if (geo.attributes.uv) geo.deleteAttribute('uv');
        if (geo.attributes.tangent) geo.deleteAttribute('tangent');
        if (geo.attributes.color) geo.deleteAttribute('color');

        geo.computeVertexNormals();

        const pos = geo.attributes.position;
        const norm = geo.attributes.normal;
        const count = pos.count;

        const newPos: number[] = [];
        const newNorm: number[] = [];
        const map = new Int32Array(count).fill(-1);

        const grid: Record<string, number[]> = {};
        const cellSize = threshold;

        const getKey = (x: number, y: number, z: number) =>
            `${Math.floor(x / cellSize)},${Math.floor(y / cellSize)},${Math.floor(z / cellSize)}`;

        let uniqueCount = 0;

        // Neighbor offsets for 3x3x3 search
        const offsets: number[][] = [];
        for (let x = -1; x <= 1; x++) {
            for (let y = -1; y <= 1; y++) {
                for (let z = -1; z <= 1; z++) {
                    offsets.push([x, y, z]);
                }
            }
        }

        for (let i = 0; i < count; i++) {
            const x = pos.getX(i);
            const y = pos.getY(i);
            const z = pos.getZ(i);

            const nx = norm.getX(i);
            const ny = norm.getY(i);
            const nz = norm.getZ(i);

            // Base cell coordinates
            const cx = Math.floor(x / cellSize);
            const cy = Math.floor(y / cellSize);
            const cz = Math.floor(z / cellSize);

            let found = -1;

            // Check all 27 neighboring cells
            for (const offset of offsets) {
                const key = `${cx + offset[0]},${cy + offset[1]},${cz + offset[2]}`;
                const candidates = grid[key];

                if (candidates) {
                    for (const idx of candidates) {
                        const tx = newPos[idx * 3];
                        const ty = newPos[idx * 3 + 1];
                        const tz = newPos[idx * 3 + 2];

                        const d2 = (x - tx) * (x - tx) + (y - ty) * (y - ty) + (z - tz) * (z - tz);
                        if (d2 < threshold * threshold) {
                            const cnx = newNorm[idx * 3];
                            const cny = newNorm[idx * 3 + 1];
                            const cnz = newNorm[idx * 3 + 2];
                            const dot = nx * cnx + ny * cny + nz * cnz;

                            if (dot > 0.7) {
                                found = idx;
                                break;
                            }
                        }
                    }
                }
                if (found !== -1) break;
            }

            if (found !== -1) {
                map[i] = found;
            } else {
                map[i] = uniqueCount;
                newPos.push(x, y, z);
                newNorm.push(nx, ny, nz);

                const key = getKey(x, y, z);
                if (!grid[key]) grid[key] = [];
                grid[key].push(uniqueCount);

                uniqueCount++;
            }
        }

        const finalPos = new Float32Array(newPos);
        const finalNorm = new Float32Array(newNorm);

        const oldIndices = geo.index ? geo.index.array : null;
        const newIndices: number[] = [];

        if (oldIndices) {
            for (let i = 0; i < oldIndices.length; i++) {
                newIndices.push(map[oldIndices[i]]);
            }
        } else {
            for (let i = 0; i < count; i++) {
                newIndices.push(map[i]);
            }
        }

        const result = new THREE.BufferGeometry();
        result.setAttribute('position', new THREE.BufferAttribute(finalPos, 3));
        result.setAttribute('normal', new THREE.BufferAttribute(finalNorm, 3));
        result.setIndex(newIndices);

        return result;
    }

    /**
     * Prepares a mesh for physics simulation and rendering.
     * - Removes unused attributes (color, etc.)
     * - Merges vertices
     * - Recomputes normals
     * - Restores UVs from original mesh via nearest-neighbor search
     * - Generates tangents for normal mapping
     */
    static process(
        mesh: THREE.Mesh,
        weldThreshold: number = GEOMETRY_WELD_THRESHOLD,
        referenceGeometry?: THREE.BufferGeometry
    ): ProcessedMesh {
        const sourceGeo = referenceGeometry || mesh.geometry;
        let geo = mesh.geometry.clone();

        geo.deleteAttribute('uv');
        geo.deleteAttribute('normal');
        geo.deleteAttribute('tangent');
        geo = BufferGeometryUtils.mergeVertices(geo, weldThreshold);
        geo.computeVertexNormals();

        const newPos = geo.attributes.position;
        const newNorm = geo.attributes.normal;

        const oldPos = sourceGeo.attributes.position;
        const oldNorm = sourceGeo.attributes.normal;
        const oldUV = sourceGeo.attributes.uv;

        const uvs = new Float32Array(newPos.count * 2);

        if (oldUV && oldNorm) {
            const tempVec = new THREE.Vector3();
            const tempOrig = new THREE.Vector3();
            const tempN = new THREE.Vector3();
            const tempOldN = new THREE.Vector3();

            for (let i = 0; i < newPos.count; i++) {
                tempVec.fromBufferAttribute(newPos, i);
                tempN.fromBufferAttribute(newNorm, i);

                let minD2 = Infinity;
                let bestIdx = 0;

                for (let j = 0; j < oldPos.count; j++) {
                    tempOrig.fromBufferAttribute(oldPos, j);
                    const d2 = tempVec.distanceToSquared(tempOrig);

                    if (d2 > 0.01) continue;

                    if (d2 < minD2) {
                        tempOldN.fromBufferAttribute(oldNorm, j);
                        if (tempN.dot(tempOldN) > 0.5) {
                            minD2 = d2;
                            bestIdx = j;
                            if (d2 < 0.000001) break;
                        }
                    }
                }
                uvs[i * 2] = oldUV.getX(bestIdx);
                uvs[i * 2 + 1] = oldUV.getY(bestIdx);
            }
        }

        geo.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));

        if (geo.attributes.uv && geo.attributes.normal) {
            geo.computeTangents();
        }

        const indexAttribute = geo.index;
        if (!indexAttribute) throw new Error("Mesh has no index buffer.");

        const tangents = geo.attributes.tangent
            ? new Float32Array(geo.attributes.tangent.array)
            : new Float32Array(newPos.count * 4);

        return {
            vertices: new Float32Array(geo.attributes.position.array),
            normals: new Float32Array(geo.attributes.normal.array),
            indices: new Uint32Array(indexAttribute.array),
            uvs: uvs,
            tangents: tangents
        };
    }
}