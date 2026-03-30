// src/application/use-cases/InitializeSimulation.ts

import * as THREE from 'three';
import { AssetPreparationPipeline } from '../pipelines/AssetPreparationPipeline';
import { GradingPipeline } from '../pipelines/GradingPipeline';
import { WasmAdapter } from '../../infrastructure/physics/adapter/WasmAdapter';
import type { IPhysicsEngine } from '../../core/interfaces/IPhysicsEngine';
import type { SimulationAssets } from '../../core/entities/Assets';
import type { ShirtSize } from '../../core/entities/Garment';

export interface InitializationResult {
    engine: IPhysicsEngine;
    assets: SimulationAssets<THREE.BufferGeometry>;
    scaledVertices: Float32Array;
}

/**
 * Use Case: Initializes the entire simulation environment.
 * Orchestrates asset loading, initial grading (sizing), and physics engine initialization.
 */
export class InitializeSimulation {
    private assetPipeline: AssetPreparationPipeline;

    constructor() {
        this.assetPipeline = new AssetPreparationPipeline();
    }

    /**
     * Orchestrates the full initialization flow.
     *
     * @param currentAssets - Existing assets (if already loaded) to avoid reloading.
     * @param size - The shirt size to apply.
     */
    async execute(
        currentAssets: SimulationAssets<THREE.BufferGeometry> | null,
        size: ShirtSize
    ): Promise<InitializationResult> {

        // 1. Load Assets (if needed)
        let assets = currentAssets;
        if (!assets) {
            assets = await this.assetPipeline.execute();
        }

        // 2. Apply Grading (Sizing)
        // This transforms the base mesh vertices to match the requested size.
        const { vertices: scaledGarmentVerts, scaleFactor } = GradingPipeline.execute(
            assets.garment,
            size
        );

        // 3. Initialize Physics Engine
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
            assets,
            scaledVertices: scaledGarmentVerts
        };
    }
}