// src/v3/engine/utils/Topology.ts
import * as THREE from 'three';

export class Topology {
    public static buildAdjacency(geo: THREE.BufferGeometry): number[][] {
        const index = geo.index;
        if (!index) throw new Error("Geometry must be indexed");

        const count = geo.attributes.position.count;
        const adj: number[][] = Array.from({ length: count }, () => []);

        for (let i = 0; i < index.count; i += 3) {
            const a = index.getX(i);
            const b = index.getX(i + 1);
            const c = index.getX(i + 2);

            if (!adj[a].includes(b)) adj[a].push(b);
            if (!adj[a].includes(c)) adj[a].push(c);
            if (!adj[b].includes(a)) adj[b].push(a);
            if (!adj[b].includes(c)) adj[b].push(c);
            if (!adj[c].includes(a)) adj[c].push(a);
            if (!adj[c].includes(b)) adj[c].push(b);
        }
        return adj;
    }

    // --- NEW: Pin based on proximity to the highest point (Collar) ---
    public static getNeckIndices(positions: Float32Array, pinRadius: number): Set<number> {
        const indices = new Set<number>();
        const count = positions.length / 3;

        // 1. Find the highest vertex (Max Y)
        let maxY = -Infinity;
        let maxIndex = -1;

        for (let i = 0; i < count; i++) {
            const y = positions[i * 3 + 1];
            if (y > maxY) {
                maxY = y;
                maxIndex = i;
            }
        }

        if (maxIndex === -1) return indices;

        // 2. Pin everything within 'pinRadius' of the highest vertex
        const idxPeak = maxIndex * 3;
        const px = positions[idxPeak];
        const py = positions[idxPeak + 1];
        const pz = positions[idxPeak + 2];
        const radiusSq = pinRadius * pinRadius;

        for (let i = 0; i < count; i++) {
            const idx = i * 3;
            const dx = positions[idx] - px;
            const dy = positions[idx + 1] - py;
            const dz = positions[idx + 2] - pz;

            if (dx * dx + dy * dy + dz * dz < radiusSq) {
                indices.add(i);
            }
        }

        return indices;
    }

    public static computeGeodesicDistances(
        count: number,
        adjacency: number[][],
        positions: Float32Array,
        anchors: Set<number>
    ): Float32Array {
        const distances = new Float32Array(count).fill(Infinity);
        const queue: number[] = [];

        anchors.forEach(idx => {
            distances[idx] = 0;
            queue.push(idx);
        });

        let head = 0;
        while (head < queue.length) {
            const current = queue[head++];
            const currentDist = distances[current];
            const neighbors = adjacency[current];

            const idxA = current * 3;
            const ax = positions[idxA];
            const ay = positions[idxA + 1];
            const az = positions[idxA + 2];

            for (const neighbor of neighbors) {
                const idxB = neighbor * 3;
                const dx = ax - positions[idxB];
                const dy = ay - positions[idxB + 1];
                const dz = az - positions[idxB + 2];
                const edgeLen = Math.sqrt(dx * dx + dy * dy + dz * dz);

                const newDist = currentDist + edgeLen;
                if (newDist < distances[neighbor]) {
                    distances[neighbor] = newDist;
                    queue.push(neighbor);
                }
            }
        }
        return distances;
    }
}