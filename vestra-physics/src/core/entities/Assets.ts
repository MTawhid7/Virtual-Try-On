// src/core/entities/Assets.ts
import type { ProcessedMesh } from './Geometry';

/**
 * Holds all the assets required to run a simulation.
 *
 * @template TVisualGeometry - The type of the visual geometry object.
 * We use a generic here to allow the Infrastructure layer to plug in
 * `THREE.BufferGeometry` without the Core layer explicitly depending on the 'three' package.
 */
export interface SimulationAssets<TVisualGeometry = any> {
    /** The processed garment mesh (Physics & Rendering data) */
    garment: ProcessedMesh;

    /** The processed body collider (Physics data only) */
    collider: ProcessedMesh;

    /** The high-resolution visual mesh for the mannequin (Rendering only) */
    visualBody: TVisualGeometry;
}