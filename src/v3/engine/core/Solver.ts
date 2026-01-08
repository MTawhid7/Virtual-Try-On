// src/v3/engine/core/Solver.ts
import * as THREE from 'three';
import { PhysicsData } from './PhysicsData';
import { DistanceConstraint } from '../constraints/DistanceConstraint';
import { BendingConstraint } from '../constraints/BendingConstraint';
import { TetherConstraint } from '../constraints/TetherConstraint';
import { MouseConstraint } from '../constraints/MouseConstraint';
import { MannequinCollider } from '../MannequinCollider';
import { SpatialHash } from './SpatialHash';
import { Topology } from '../utils/Topology';
import { PHYSICS_CONSTANTS } from '../../shared/constants';

export class Solver {
    public data: PhysicsData;
    private distanceConstraints: DistanceConstraint;
    private bendingConstraints: BendingConstraint;
    private tetherConstraints: TetherConstraint;
    private mouseConstraint: MouseConstraint;
    public collider: MannequinCollider;
    private selfCollider: SpatialHash;
    private tempVec = new THREE.Vector3();

    constructor(proxyMesh: THREE.Mesh, collisionMesh: THREE.Mesh) {
        this.data = new PhysicsData(proxyMesh);

        // 1. Identify Neck Vertices
        const neckIndices = Topology.getNeckIndices(this.data.positions, PHYSICS_CONSTANTS.pinRadius);

        // --- FIX: Apply Pinning (Set Mass to 0) ---
        neckIndices.forEach(index => {
            this.data.invMass[index] = 0;
        });

        this.distanceConstraints = new DistanceConstraint(proxyMesh.geometry, this.data);
        this.bendingConstraints = new BendingConstraint(proxyMesh.geometry, this.data);
        this.tetherConstraints = new TetherConstraint(proxyMesh.geometry, this.data, neckIndices);
        this.mouseConstraint = new MouseConstraint();

        this.collider = new MannequinCollider();
        this.collider.setMesh(collisionMesh);

        this.selfCollider = new SpatialHash(this.data.count);
    }

    // ... rest of the file remains unchanged ...
    public update(dt: number) {
        const substeps = PHYSICS_CONSTANTS.substeps;
        const sdt = dt / substeps;

        this.collider.updateMatrix();

        const collisionFreq = 5;

        for (let step = 0; step < substeps; step++) {
            this.integrate(sdt);

            this.mouseConstraint.solve(this.data, sdt);
            this.distanceConstraints.solve(this.data, sdt);
            this.bendingConstraints.solve(this.data, sdt);
            this.tetherConstraints.solve(this.data);

            if (step % collisionFreq === 0) {
                this.solveCollisions();
            }

            if (step % collisionFreq === 0) {
                this.selfCollider.solve(this.data);
            }
        }
    }

    private integrate(dt: number) {
        const gravity = PHYSICS_CONSTANTS.gravity * dt * dt;
        const drag = PHYSICS_CONSTANTS.drag;

        for (let i = 0; i < this.data.count; i++) {
            if (this.data.invMass[i] === 0) continue;
            const idx = i * 3;

            const x = this.data.positions[idx];
            const y = this.data.positions[idx + 1];
            const z = this.data.positions[idx + 2];

            const px = this.data.prevPositions[idx];
            const py = this.data.prevPositions[idx + 1];
            const pz = this.data.prevPositions[idx + 2];

            const nx = x + (x - px) * drag;
            const ny = y + (y - py) * drag + gravity;
            const nz = z + (z - pz) * drag;

            this.data.prevPositions[idx] = x;
            this.data.prevPositions[idx + 1] = y;
            this.data.prevPositions[idx + 2] = z;

            this.data.positions[idx] = nx;
            this.data.positions[idx + 1] = ny;
            this.data.positions[idx + 2] = nz;
        }
    }

    private solveCollisions() {
        const friction = PHYSICS_CONSTANTS.friction;
        const sleepThresholdSq = 0.001;

        for (let i = 0; i < this.data.count; i++) {
            if (this.data.invMass[i] === 0) continue;

            const idx = i * 3;
            this.tempVec.set(this.data.positions[idx], this.data.positions[idx + 1], this.data.positions[idx + 2]);

            const result = this.collider.collide(this.tempVec);

            if (result.collided) {
                this.data.positions[idx] = this.tempVec.x;
                this.data.positions[idx + 1] = this.tempVec.y;
                this.data.positions[idx + 2] = this.tempVec.z;

                let vx = this.data.positions[idx] - this.data.prevPositions[idx];
                let vy = this.data.positions[idx + 1] - this.data.prevPositions[idx + 1];
                let vz = this.data.positions[idx + 2] - this.data.prevPositions[idx + 2];

                if (result.rescued) {
                    vx = vy = vz = 0;
                } else {
                    const nx = result.normal.x;
                    const ny = result.normal.y;
                    const nz = result.normal.z;

                    const vDotN = vx * nx + vy * ny + vz * nz;

                    if (vDotN < 0) {
                        vx -= nx * vDotN;
                        vy -= ny * vDotN;
                        vz -= nz * vDotN;
                    }

                    vx *= (1 - friction);
                    vy *= (1 - friction);
                    vz *= (1 - friction);

                    const vSq = vx * vx + vy * vy + vz * vz;
                    if (vSq < sleepThresholdSq) {
                        vx = 0;
                        vy = 0;
                        vz = 0;
                    }
                }

                this.data.prevPositions[idx] = this.data.positions[idx] - vx;
                this.data.prevPositions[idx + 1] = this.data.positions[idx + 1] - vy;
                this.data.prevPositions[idx + 2] = this.data.positions[idx + 2] - vz;
            }
        }
    }

    public startInteraction(index: number, point: THREE.Vector3) {
        this.data.interaction.active = true;
        this.data.interaction.particleIndex = index;
        this.data.interaction.target.copy(point);
    }
    public updateInteraction(point: THREE.Vector3) {
        if (this.data.interaction.active) {
            this.data.interaction.target.copy(point);
        }
    }
    public endInteraction(velocity: THREE.Vector3) {
        if (!this.data.interaction.active) return;
        const idx = this.data.interaction.particleIndex * 3;
        const dt = 0.016;
        const damping = PHYSICS_CONSTANTS.interaction.releaseDamping;
        this.data.prevPositions[idx] = this.data.positions[idx] - (velocity.x * dt * damping);
        this.data.prevPositions[idx + 1] = this.data.positions[idx + 1] - (velocity.y * dt * damping);
        this.data.prevPositions[idx + 2] = this.data.positions[idx + 2] - (velocity.z * dt * damping);
        this.data.interaction.active = false;
        this.data.interaction.particleIndex = -1;
    }
}