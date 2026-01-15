import { Vector3 } from 'three';
import type { Particle, Constraint } from './types';
import type { PhysicsConfig } from './IPhysicsEngine';

export class PhysicsState {
    public particles: Particle[] = [];
    public constraints: Constraint[] = [];
    public config: PhysicsConfig;

    constructor() {
        this.config = { gravity: -9.81, iterations: 5 };
    }

    addParticle(x: number, y: number, z: number, mass: number) {
        const p = new Vector3(x, y, z);
        this.particles.push({
            position: p.clone(),
            prevPosition: p.clone(),
            originalPosition: p.clone(),
            acceleration: new Vector3(0, 0, 0),
            mass,
            pinned: false,
        });
    }

    addConstraint(p1: number, p2: number, dist: number) {
        this.constraints.push({ p1, p2, restDistance: dist });
    }

    reset() {
        this.particles = [];
        this.constraints = [];
    }
}