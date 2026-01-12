// src/v3/engine/constraints/BendingConstraint.ts
import * as THREE from 'three';
import { PhysicsData } from '../core/PhysicsData';
import { PHYSICS_CONSTANTS } from '../../shared/constants';

export class BendingConstraint {
    // Initialize with empty defaults
    private constraints: Int32Array = new Int32Array(0);
    private restLengths: Float32Array = new Float32Array(0);
    private count: number = 0;

    constructor(geo: THREE.BufferGeometry, data: PhysicsData) {
        const index = geo.index;
        if (!index) return;

        // 1. Build Adjacency Map
        const adj = new Array(data.count).fill(0).map(() => new Set<number>());
        for (let i = 0; i < index.count; i += 3) {
            const a = index.getX(i);
            const b = index.getX(i + 1);
            const c = index.getX(i + 2);
            adj[a].add(b); adj[a].add(c);
            adj[b].add(a); adj[b].add(c);
            adj[c].add(a); adj[c].add(b);
        }

        const constraintList: number[] = [];
        const lengthList: number[] = [];
        const processed = new Set<string>();

        // 2. Find "2-hop" neighbors
        for (let i = 0; i < data.count; i++) {
            adj[i].forEach((neighbor) => {
                adj[neighbor].forEach((farNeighbor) => {
                    if (i === farNeighbor) return;
                    if (adj[i].has(farNeighbor)) return; // Skip direct edges

                    const key = i < farNeighbor ? `${i}_${farNeighbor}` : `${farNeighbor}_${i}`;
                    if (processed.has(key)) return;
                    processed.add(key);

                    constraintList.push(i, farNeighbor);

                    const idxA = i * 3;
                    const idxB = farNeighbor * 3;
                    const dx = data.positions[idxA] - data.positions[idxB];
                    const dy = data.positions[idxA + 1] - data.positions[idxB + 1];
                    const dz = data.positions[idxA + 2] - data.positions[idxB + 2];
                    lengthList.push(Math.sqrt(dx * dx + dy * dy + dz * dz));
                });
            });
        }

        this.constraints = new Int32Array(constraintList);
        this.restLengths = new Float32Array(lengthList);
        this.count = lengthList.length;
    }

    public solve(data: PhysicsData, dt: number) {
        const alpha = PHYSICS_CONSTANTS.bendingCompliance / (dt * dt);
        const pos = data.positions;
        const invMass = data.invMass;

        for (let i = 0; i < this.count; i++) {
            const idA = this.constraints[i * 2];
            const idB = this.constraints[i * 2 + 1];

            const wA = invMass[idA];
            const wB = invMass[idB];
            const wSum = wA + wB;
            if (wSum === 0) continue;

            const idxA = idA * 3;
            const idxB = idB * 3;

            const dx = pos[idxA] - pos[idxB];
            const dy = pos[idxA + 1] - pos[idxB + 1];
            const dz = pos[idxA + 2] - pos[idxB + 2];

            const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (dist < 0.000001) continue;

            const rest = this.restLengths[i];
            const C = dist - rest;
            const lambda = -C / (wSum + alpha);

            const nx = dx / dist;
            const ny = dy / dist;
            const nz = dz / dist;

            if (wA > 0) {
                pos[idxA] += nx * lambda * wA;
                pos[idxA + 1] += ny * lambda * wA;
                pos[idxA + 2] += nz * lambda * wA;
            }
            if (wB > 0) {
                pos[idxB] -= nx * lambda * wB;
                pos[idxB + 1] -= ny * lambda * wB;
                pos[idxB + 2] -= nz * lambda * wB;
            }
        }
    }
}