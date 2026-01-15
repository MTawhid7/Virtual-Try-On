import * as THREE from 'three';
import * as BufferGeometryUtils from 'three/examples/jsm/utils/BufferGeometryUtils.js';

export const processShirtGeometry = (mesh: THREE.Mesh) => {
    // 1. Weld vertices
    const geometry = mesh.geometry.clone();
    geometry.deleteAttribute('uv');
    geometry.deleteAttribute('normal');
    const welded = BufferGeometryUtils.mergeVertices(geometry, 0.01);

    // FIX: Center the geometry on its own origin, but DO NOT move it up.
    // This allows us to control its starting Y position precisely in the component.
    welded.computeBoundingBox();
    const center = new THREE.Vector3();
    welded.boundingBox?.getCenter(center);
    welded.translate(-center.x, -center.y, -center.z);

    welded.computeVertexNormals();

    // 2. Extract Vertices & Edges (same as before)
    const positions = welded.attributes.position.array;
    const vertexCount = positions.length / 3;
    const index = welded.index;
    const edges = new Set<string>();
    const constraints: [number, number, number][] = [];

    if (index) {
        for (let i = 0; i < index.count; i += 3) {
            const a = index.getX(i);
            const b = index.getX(i + 1);
            const c = index.getX(i + 2);

            const addEdge = (v1: number, v2: number) => {
                const id = v1 < v2 ? `${v1}_${v2}` : `${v2}_${v1}`;
                if (!edges.has(id)) {
                    edges.add(id);
                    const p1 = new THREE.Vector3(positions[v1 * 3], positions[v1 * 3 + 1], positions[v1 * 3 + 2]);
                    const p2 = new THREE.Vector3(positions[v2 * 3], positions[v2 * 3 + 1], positions[v2 * 3 + 2]);
                    constraints.push([v1, v2, p1.distanceTo(p2)]);
                }
            };

            addEdge(a, b); addEdge(b, c); addEdge(c, a);
        }
    }

    return { geometry: welded, constraints, vertexCount };
};