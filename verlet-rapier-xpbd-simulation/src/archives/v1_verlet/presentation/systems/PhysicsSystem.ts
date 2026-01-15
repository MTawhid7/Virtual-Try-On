import * as THREE from 'three';
import { VerletPhysicsEngine } from '../../simulation/VerletPhysicsEngine';
import type { AppConfig } from '../../types';

export class PhysicsSystem {
    private engine: VerletPhysicsEngine;
    private config: AppConfig;

    constructor(engine: VerletPhysicsEngine, config: AppConfig) {
        this.engine = engine;
        this.config = config;
    }

    // REPLACED: No more hardcoded spheres!
    setBodyCollider(mesh: THREE.Mesh) {
        this.engine.setColliderMesh(mesh);
    }

    update(dt: number, mesh: THREE.Mesh | null) {
        if (this.config.physics.enabled && mesh) {
            this.engine.step(dt, mesh);
            this.engine.syncToMesh(mesh);
        }
    }
}