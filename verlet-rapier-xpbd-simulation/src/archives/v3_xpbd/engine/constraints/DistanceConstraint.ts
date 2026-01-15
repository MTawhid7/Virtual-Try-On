// src/v3/engine/constraints/DistanceConstraint.ts
import * as THREE from 'three';
import { PhysicsData } from '../core/PhysicsData';
import { PHYSICS_CONSTANTS } from '../../shared/constants';

export class DistanceConstraint {
    private constraints: Int32Array = new Int32Array(0);
    private restLengths: Float32Array = new Float32Array(0);
    private count: number = 0;

    // New: Specific constraints for the collar
    private collarIndices: Set<number> = new Set();

    constructor(geo: THREE.BufferGeometry, data: PhysicsData) {
        // ... existing constructor code ...
        // (Copy the constructor from previous step)
        const index = geo.index;
        if (!index) throw new Error("Mesh must be indexed");

        const edges = new Set<string>();
        const constraintList: number[] = [];
        const lengthList: number[] = [];

        const addEdge = (a: number, b: number) => {
            const key = a < b ? `${a}_${b}` : `${b}_${a}`;
            if (edges.has(key)) return;
            edges.add(key);
            constraintList.push(a, b);
            const idxA = a * 3;
            const idxB = b * 3;
            const dx = data.positions[idxA] - data.positions[idxB];
            const dy = data.positions[idxA + 1] - data.positions[idxB + 1];
            const dz = data.positions[idxA + 2] - data.positions[idxB + 2];
            lengthList.push(Math.sqrt(dx * dx + dy * dy + dz * dz));
        };

        for (let i = 0; i < index.count; i += 3) {
            addEdge(index.getX(i), index.getX(i + 1));
            addEdge(index.getX(i + 1), index.getX(i + 2));
            addEdge(index.getX(i + 2), index.getX(i));
        }

        this.constraints = new Int32Array(constraintList);
        this.restLengths = new Float32Array(lengthList);
        this.count = lengthList.length;
    }

    public setCollar(indices: Set<number>) {
        this.collarIndices = indices;
    }

    public solve(data: PhysicsData, dt: number) {
        const alpha = PHYSICS_CONSTANTS.compliance / (dt * dt);
        const pos = data.positions;
        const invMass = data.invMass;

        // Pass 1: XPBD (Soft)
        for (let i = 0; i < this.count; i++) {
            this.solveConstraint(i, pos, invMass, alpha);
        }
    }

    public solveHardLimits(data: PhysicsData) {
        const pos = data.positions;
        const invMass = data.invMass;

        // Pass 2: Hard Limits (The "Goldilocks" Pass)
        // We allow 5% stretch for normal cloth, 0% for collar
        const GLOBAL_LIMIT = 1.05;
        const COLLAR_LIMIT = 1.00;

        for (let i = 0; i < this.count; i++) {
            const idA = this.constraints[i * 2];
            const idB = this.constraints[i * 2 + 1];

            // Check if this is a collar edge (both vertices in collar set)
            const isCollar = this.collarIndices.has(idA) && this.collarIndices.has(idB);
            const limit = isCollar ? COLLAR_LIMIT : GLOBAL_LIMIT;

            const idxA = idA * 3;
            const idxB = idB * 3;

            const dx = pos[idxA] - pos[idxB];
            const dy = pos[idxA + 1] - pos[idxB + 1];
            const dz = pos[idxA + 2] - pos[idxB + 2];
            const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            const rest = this.restLengths[i];

            // If stretched beyond limit, HARD RESET
            if (dist > rest * limit) {
                const correction = dist - (rest * limit);
                const wA = invMass[idA];
                const wB = invMass[idB];
                const wSum = wA + wB;
                if (wSum === 0) continue;

                const nx = dx / dist;
                const ny = dy / dist;
                const nz = dz / dist;

                // Direct position modification (No alpha, no forces)
                const s = correction / wSum;

                if (wA > 0) {
                    pos[idxA] -= nx * s * wA;
                    pos[idxA + 1] -= ny * s * wA;
                    pos[idxA + 2] -= nz * s * wA;
                }
                if (wB > 0) {
                    pos[idxB] += nx * s * wB;
                    pos[idxB + 1] += ny * s * wB;
                    pos[idxB + 2] += nz * s * wB;
                }
            }
        }
    }

    private solveConstraint(i: number, pos: Float32Array, invMass: Float32Array, alpha: number) {
        const idA = this.constraints[i * 2];
        const idB = this.constraints[i * 2 + 1];
        const wA = invMass[idA];
        const wB = invMass[idB];
        const wSum = wA + wB;
        if (wSum === 0) return;

        const idxA = idA * 3;
        const idxB = idB * 3;
        const dx = pos[idxA] - pos[idxB];
        const dy = pos[idxA + 1] - pos[idxB + 1];
        const dz = pos[idxA + 2] - pos[idxB + 2];
        const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < 1e-6) return;

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