// src/v3/engine/constraints/TetherConstraint.ts
import * as THREE from 'three';
import { PhysicsData } from '../core/PhysicsData';
import { Topology } from '../utils/Topology';

export class TetherConstraint {
    private indices: Int32Array;
    private maxDistances: Float32Array;
    private count: number = 0;

    constructor(geo: THREE.BufferGeometry, data: PhysicsData, anchorIndices: Set<number>) {
        const adjacency = Topology.buildAdjacency(geo);
        const distances = Topology.computeGeodesicDistances(data.count, adjacency, data.positions, anchorIndices);

        const anchorArray = Array.from(anchorIndices);
        if (anchorArray.length === 0) {
            this.indices = new Int32Array(0);
            this.maxDistances = new Float32Array(0);
            return;
        }

        const constraints: number[] = [];
        const limits: number[] = [];
        const SLACK = 1.05;

        for (let i = 0; i < data.count; i++) {
            if (anchorIndices.has(i)) continue;
            if (distances[i] === Infinity) continue;

            let bestAnchor = anchorArray[0];
            let minEuclidean = Infinity;

            const idxA = i * 3;
            const px = data.positions[idxA];
            const py = data.positions[idxA + 1];
            const pz = data.positions[idxA + 2];

            for (const anchor of anchorArray) {
                const idxB = anchor * 3;
                const dx = px - data.positions[idxB];
                const dy = py - data.positions[idxB + 1];
                const dz = pz - data.positions[idxB + 2];
                const d2 = dx * dx + dy * dy + dz * dz;
                if (d2 < minEuclidean) {
                    minEuclidean = d2;
                    bestAnchor = anchor;
                }
            }

            constraints.push(i, bestAnchor);
            limits.push(distances[i] * SLACK);
        }

        this.indices = new Int32Array(constraints);
        this.maxDistances = new Float32Array(limits);
        this.count = limits.length;
    }

    public solve(data: PhysicsData) {
        const pos = data.positions;
        const invMass = data.invMass;

        // VIBRATION FIX: Soften the tether.
        // 1.0 = Hard Snap (Vibration). 0.8 = Shock Absorber (Stable).
        const STIFFNESS = 0.8;

        for (let i = 0; i < this.count; i++) {
            const particleIdx = this.indices[i * 2];
            const anchorIdx = this.indices[i * 2 + 1];

            const w = invMass[particleIdx];
            if (w === 0) continue;

            const idxA = particleIdx * 3;
            const idxB = anchorIdx * 3;

            const dx = pos[idxA] - pos[idxB];
            const dy = pos[idxA + 1] - pos[idxB + 1];
            const dz = pos[idxA + 2] - pos[idxB + 2];

            const distSq = dx * dx + dy * dy + dz * dz;
            const maxDist = this.maxDistances[i];

            if (distSq > maxDist * maxDist) {
                const dist = Math.sqrt(distSq);
                const correction = (dist - maxDist) / dist;

                // Apply correction with stiffness
                const scalar = correction * STIFFNESS;

                pos[idxA] -= dx * scalar;
                pos[idxA + 1] -= dy * scalar;
                pos[idxA + 2] -= dz * scalar;
            }
        }
    }
}