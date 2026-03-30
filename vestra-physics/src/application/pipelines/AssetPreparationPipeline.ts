// src/application/pipelines/AssetPreparationPipeline.ts

import * as THREE from 'three';
import { MeshLoader } from '../../infrastructure/loaders/MeshLoader';
import { OrientationOptimizer } from '../../infrastructure/geometry/analysis/orientation/OrientationOptimizer';
import { MeshScaler } from '../../infrastructure/geometry/modifiers/MeshScaler';
import { AutoAligner } from '../../infrastructure/geometry/modifiers/AutoAligner';
import { BodyMeasurer } from '../../infrastructure/geometry/analysis/BodyMeasurer';
import { ProxyGenerator } from '../../infrastructure/geometry/modifiers/ProxyGenerator';
import { GeometryProcessor } from '../../infrastructure/geometry/processors/GeometryProcessor';
import { GEOMETRY_WELD_THRESHOLD } from '../../core/constants/SimulationConstants';
import type { SimulationAssets } from '../../core/entities/Assets';

/**
 * Pipeline responsible for loading, analyzing, and preparing 3D assets for the simulation.
 * This includes loading GLB files, aligning them, ensuring correct scale, and generating physics proxies.
 */
export class AssetPreparationPipeline {
    private meshLoader: MeshLoader;

    constructor() {
        this.meshLoader = new MeshLoader();
    }

    /**
     * Loads and prepares all assets for the simulation.
     */
    async execute(): Promise<SimulationAssets<THREE.BufferGeometry>> {
        console.log("[AssetPipeline] Starting asset preparation...");

        // 1. Load Raw GLBs
        const [shirtMesh, rawBodyMesh] = await Promise.all([
            this.meshLoader.load('/models/shirt.glb', 'Garment'),
            this.meshLoader.load('/models/mannequin.glb', 'Body')
        ]);

        // 2. PCA Axis Alignment
        // Ensures the body is roughly upright and facing a cardinal direction
        OrientationOptimizer.alignAxes(rawBodyMesh);

        // 3. Smart Scaling
        // Normalizes the body to a standard height (1.75m) so physics units (gravity) make sense
        MeshScaler.normalizeBodyScale(rawBodyMesh);
        this.normalizeShirtScale(shirtMesh);

        // 4. Topology Validation
        // Fixes "Backward" or "Upside Down" meshes using voting heuristics
        OrientationOptimizer.validateOrientation(rawBodyMesh);

        // 5. Bake Transforms
        // Applies all rotations/scales to the vertex data permanently
        this.meshLoader.bakeTransform(rawBodyMesh);
        this.meshLoader.bakeTransform(shirtMesh);

        // 6. Alignment
        // Centers the body on (0,0,0) and snaps the shirt collar to the neck
        console.log("[AssetPipeline] Auto-Aligning meshes...");
        AutoAligner.alignBody(rawBodyMesh.geometry);
        AutoAligner.alignGarmentToBody(shirtMesh.geometry, rawBodyMesh.geometry);

        // Log Body Measurements
        const chestCm = BodyMeasurer.getChestCircumference(rawBodyMesh);
        console.log(`[AssetPipeline] Mannequin Analysis: Height 1.75m, Chest ${chestCm.toFixed(1)}cm`);

        // 7. Physics Proxy Generation
        // Decimates high-poly meshes to optimized counts for physics
        console.log("[AssetPipeline] Generating Physics Proxies...");

        // A. Body Collider
        const colliderProcessed = await ProxyGenerator.generateCollider(rawBodyMesh);

        // B. Garment Proxy
        // Step 1: Smart Weld the High-Poly mesh to close seams.
        // We use a 1.25cm (0.0125) threshold. This is tight enough to preserve folds/shape
        // but wide enough to close standard UV seams, preventing the "Confetti" effect.
        const weldedGarmentGeo = GeometryProcessor.smartWeld(shirtMesh.geometry, 0.0125);
        const weldedGarmentMesh = new THREE.Mesh(weldedGarmentGeo);

        // Step 2: Decimate the Welded Mesh
        // The decimator now sees a continuous hull, so it won't split the seams.
        const garmentProxy = await ProxyGenerator.generateGarment(weldedGarmentMesh);

        // 8. Final Geometry Processing
        // Prepares the garment for simulation (Welding, Tangents, UVs)
        const garmentMeshForProcessing = new THREE.Mesh(new THREE.BufferGeometry());
        garmentMeshForProcessing.geometry.setAttribute('position', new THREE.BufferAttribute(garmentProxy.vertices, 3));
        garmentMeshForProcessing.geometry.setIndex(new THREE.BufferAttribute(garmentProxy.indices, 1));

        // Ensure source has normals for the smart recovery
        shirtMesh.geometry.computeVertexNormals();

        // Step 3: Process & Recover UVs
        // We pass 'shirtMesh.geometry' (Original High-Poly) as the reference to recover UVs
        // that were lost during the pre-weld step.
        const garmentFinal = GeometryProcessor.process(
            garmentMeshForProcessing,
            GEOMETRY_WELD_THRESHOLD, // Keep final cleanup weld tight (2cm default)
            shirtMesh.geometry
        );

        // Restore UVs if they exist (needed for textures)
        if (garmentProxy.uvs.length > 0 && garmentFinal.vertices.length === garmentProxy.vertices.length) {
            garmentFinal.uvs = garmentProxy.uvs;
        }

        console.log(`[AssetPipeline] Assets Ready. Garment: ${garmentFinal.vertices.length / 3} verts.`);

        return {
            garment: garmentFinal,
            collider: colliderProcessed,
            visualBody: rawBodyMesh.geometry // Return the high-res geometry for rendering
        };
    }

    private normalizeShirtScale(mesh: THREE.Mesh) {
        const box = new THREE.Box3().setFromObject(mesh);
        const height = box.max.y - box.min.y;

        // Sanity check: If shirt is microscopic or gigantic, normalize it to ~85cm length
        if (height < 0.5 || height > 2.5) {
            const targetHeight = 0.85;
            const scale = targetHeight / height;
            console.log(`[AssetPipeline] Normalizing Shirt Scale: ${height.toFixed(2)}m -> ${targetHeight}m (Factor: ${scale.toFixed(4)})`);
            mesh.scale.setScalar(scale);
            mesh.updateMatrixWorld(true);
        }
    }
}