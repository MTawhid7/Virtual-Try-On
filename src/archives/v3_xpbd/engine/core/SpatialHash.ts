// src/v3/engine/collision/SpatialHash.ts
import { PhysicsData } from '../core/PhysicsData';

export class SpatialHash {
    private cellSize: number;
    private tableSize: number;
    private table: Int32Array;
    private next: Int32Array;
    private queryIds: number[] = [];

    constructor(count: number, cellSize: number = 0.015) {
        this.cellSize = cellSize;
        this.tableSize = this.getNextPrime(count * 2);
        this.table = new Int32Array(this.tableSize);
        this.next = new Int32Array(count);
    }

    private getNextPrime(n: number): number {
        while (true) {
            let isPrime = true;
            for (let i = 2; i * i <= n; i++) {
                if (n % i === 0) {
                    isPrime = false;
                    break;
                }
            }
            if (isPrime) return n;
            n++;
        }
    }

    public solve(data: PhysicsData) {
        const count = data.count;
        const pos = data.positions;
        const invMass = data.invMass;
        const thickness = 0.01; // 1cm thickness
        const stiffness = 0.2;

        // 1. Update Hash Table
        this.table.fill(-1);
        this.next.fill(-1);

        for (let i = 0; i < count; i++) {
            const x = pos[i * 3];
            const y = pos[i * 3 + 1];
            const z = pos[i * 3 + 2];
            const h = this.hash(x, y, z);
            this.next[i] = this.table[h];
            this.table[h] = i;
        }

        // 2. Solve Collisions
        for (let i = 0; i < count; i++) {
            if (invMass[i] === 0) continue;

            const idxA = i * 3;
            const px = pos[idxA];
            const py = pos[idxA + 1];
            const pz = pos[idxA + 2];

            this.query(px, py, pz, thickness);

            for (let k = 0; k < this.queryIds.length; k++) {
                const j = this.queryIds[k];
                if (i === j) continue;

                const idxB = j * 3;
                const dx = px - pos[idxB];
                const dy = py - pos[idxB + 1];
                const dz = pz - pos[idxB + 2];
                const distSq = dx * dx + dy * dy + dz * dz;

                if (distSq < thickness * thickness && distSq > 0.000001) {
                    const dist = Math.sqrt(distSq);
                    const correction = (thickness - dist) * 0.5 * stiffness;

                    const nx = dx / dist;
                    const ny = dy / dist;
                    const nz = dz / dist;

                    const wA = invMass[i];
                    const wB = invMass[j];
                    const wSum = wA + wB;

                    if (wSum > 0) {
                        const s = correction / wSum;
                        if (wA > 0) {
                            pos[idxA] += nx * s * wA;
                            pos[idxA + 1] += ny * s * wA;
                            pos[idxA + 2] += nz * s * wA;
                        }
                        if (wB > 0) {
                            pos[idxB] -= nx * s * wB;
                            pos[idxB + 1] -= ny * s * wB;
                            pos[idxB + 2] -= nz * s * wB;
                        }
                    }
                }
            }
        }
    }

    private query(x: number, y: number, z: number, radius: number) {
        this.queryIds.length = 0;
        const x0 = Math.floor((x - radius) / this.cellSize);
        const y0 = Math.floor((y - radius) / this.cellSize);
        const z0 = Math.floor((z - radius) / this.cellSize);
        const x1 = Math.floor((x + radius) / this.cellSize);
        const y1 = Math.floor((y + radius) / this.cellSize);
        const z1 = Math.floor((z + radius) / this.cellSize);

        for (let xi = x0; xi <= x1; xi++) {
            for (let yi = y0; yi <= y1; yi++) {
                for (let zi = z0; zi <= z1; zi++) {
                    const h = this.hashCoords(xi, yi, zi);
                    let id = this.table[h];
                    while (id !== -1) {
                        this.queryIds.push(id);
                        id = this.next[id];
                    }
                }
            }
        }
    }

    private hash(x: number, y: number, z: number): number {
        const xi = Math.floor(x / this.cellSize);
        const yi = Math.floor(y / this.cellSize);
        const zi = Math.floor(z / this.cellSize);
        return this.hashCoords(xi, yi, zi);
    }

    private hashCoords(xi: number, yi: number, zi: number): number {
        const h = (xi * 73856093) ^ (yi * 19349663) ^ (zi * 83492791);
        return (h & 0x7fffffff) % this.tableSize;
    }
}