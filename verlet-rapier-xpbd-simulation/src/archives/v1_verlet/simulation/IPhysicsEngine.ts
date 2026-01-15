import { Mesh, Vector3 } from 'three';
// Change to 'import type'
import type { Disposable } from '../types';

export interface PhysicsConfig {
    gravity: number;
    iterations: number;
}

export interface IPhysicsEngine extends Disposable {
    initialize(mesh: Mesh, config: PhysicsConfig): Promise<void>;
    step(deltaTime: number): void;
    applyForce(particleIndex: number, force: Vector3): void;
    pinParticle(particleIndex: number, position: Vector3): void;
    releaseParticle(particleIndex: number): void;
    syncToMesh(mesh: Mesh): void;
}