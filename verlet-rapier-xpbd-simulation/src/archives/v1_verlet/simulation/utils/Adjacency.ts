import * as THREE from 'three';

export class Adjacency {
    static findEdges(geometry: THREE.BufferGeometry): { a: number, b: number }[] {
        const index = geometry.index;
        if (!index) throw new Error("Geometry must be indexed!");

        const edges = new Set<string>();
        const result: { a: number, b: number }[] = [];

        // Helper to store unique edge key "min_max"
        const addEdge = (i1: number, i2: number) => {
            const a = Math.min(i1, i2);
            const b = Math.max(i1, i2);
            const key = `${a}_${b}`;
            if (!edges.has(key)) {
                edges.add(key);
                result.push({ a, b });
            }
        };

        // Iterate over all triangles
        const count = index.count;
        for (let i = 0; i < count; i += 3) {
            const a = index.getX(i);
            const b = index.getX(i + 1);
            const c = index.getX(i + 2);

            // Add the 3 edges of the triangle
            addEdge(a, b);
            addEdge(b, c);
            addEdge(c, a);
        }

        return result;
    }
}