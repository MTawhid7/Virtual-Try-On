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

    // --- NEW: Pin based on Vertical Height (Top 3cm) ---
    // This ensures both left and right sides of the collar are pinned equally.
    public static getNeckIndices(positions: Float32Array, pinDepth: number): Set<number> {
        const indices = new Set<number>();
        const count = positions.length / 3;

        // 1. Find the absolute highest Y value
        let maxY = -Infinity;
        for (let i = 0; i < count; i++) {
            const y = positions[i * 3 + 1];
            if (y > maxY) maxY = y;
        }

        // 2. Pin everything within 'pinDepth' of the top
        // e.g., if Top is 1.55m and depth is 0.03m, pin everything above 1.52m
        const threshold = maxY - pinDepth;

        for (let i = 0; i < count; i++) {
            const y = positions[i * 3 + 1];
            if (y >= threshold) {
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