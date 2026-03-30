// src/core/interfaces/IAssetLoader.ts
import type { SimulationAssets } from '../entities/Assets';

/**
 * Contract for a service that loads and prepares the 3D assets.
 *
 * @template TVisualGeometry - The concrete type of the visual geometry (e.g., THREE.BufferGeometry).
 * We use a generic here to keep the Core layer free of 'three' dependencies.
 */
export interface IAssetLoader<TVisualGeometry = any> {
    /**
     * Loads the default scene assets (Mannequin and Shirt).
     * This includes loading, aligning, scaling, and generating physics proxies.
     */
    loadScene(): Promise<SimulationAssets<TVisualGeometry>>;
}