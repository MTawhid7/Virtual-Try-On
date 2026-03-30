// src/application/use-cases/UpdateGarmentSize.ts

import * as THREE from 'three';
import { GradingPipeline } from '../pipelines/GradingPipeline';
import { WasmAdapter } from '../../infrastructure/physics/adapter/WasmAdapter';
import type { IPhysicsEngine } from '../../core/interfaces/IPhysicsEngine';
import type { SimulationAssets } from '../../core/entities/Assets';
import type { ShirtSize } from '../../core/entities/Garment';

export interface UpdateResult {
    engine: IPhysicsEngine;
    scaledVertices: Float32Array;
}

/**
 * Use Case: Updates the garment size during an active session.
 * Re-runs the grading pipeline and re-initializes the physics engine with the new mesh data.
 */
export class UpdateGarmentSize {
    /**
     * Resizes the garment and re-initializes the physics engine.
     *
     * @param assets - The loaded simulation assets.
     * @param newSize - The new size to apply.
     * @param oldEngine - The current engine instance (to be disposed).
     */
    async execute(
        assets: SimulationAssets<THREE.BufferGeometry>,
        newSize: ShirtSize,
        oldEngine: IPhysicsEngine | null
    ): Promise<UpdateResult> {

        // 1. Dispose the old engine to free WASM memory
        if (oldEngine) {
            oldEngine.dispose();
        }

        // 2. Apply Grading
        const { vertices: scaledGarmentVerts, scaleFactor } = GradingPipeline.execute(
            assets.garment,
            newSize
        );

        // 3. Re-Initialize Physics Engine
        const engine = new WasmAdapter();
        await engine.init(
            scaledGarmentVerts,
            assets.garment.indices,
            assets.garment.uvs,
            assets.collider.vertices,
            assets.collider.normals,
            assets.collider.indices,
            scaleFactor
        );

        return {
            engine,
            scaledVertices: scaledGarmentVerts
        };
    }
}