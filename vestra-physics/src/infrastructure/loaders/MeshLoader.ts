// src/infrastructure/loaders/MeshLoader.ts

import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';

/**
 * Infrastructure service for loading GLTF/GLB files.
 * Provides functionality to extract meshes from scenes and bake transforms.
 */
export class MeshLoader {
    private loader: GLTFLoader;

    constructor() {
        this.loader = new GLTFLoader();
    }

    /**
     * Loads a GLTF/GLB file and extracts the primary mesh.
     *
     * Strategy:
     * 1. Load the file.
     * 2. Traverse the scene to find all Mesh objects.
     * 3. Select the mesh with the highest vertex count (assumed to be the main body/garment).
     * 4. Clone it to detach from the GLTF scene graph.
     *
     * @param url - Path to the .glb file.
     * @param name - Debug name for logging.
     */
    public load(url: string, name: string): Promise<THREE.Mesh> {
        return new Promise((resolve, reject) => {
            this.loader.load(
                url,
                (gltf) => {
                    const candidates: { mesh: THREE.Mesh, count: number }[] = [];

                    gltf.scene.traverse((child) => {
                        if ((child as THREE.Mesh).isMesh) {
                            const mesh = child as THREE.Mesh;
                            const geometry = mesh.geometry;
                            const count = geometry.attributes.position.count;
                            candidates.push({ mesh, count });
                        }
                    });

                    // Sort by vertex count (descending) to find the main body
                    candidates.sort((a, b) => b.count - a.count);
                    const bestEntry = candidates[0];

                    if (bestEntry) {
                        const bestMesh = bestEntry.mesh;
                        console.log(`[MeshLoader] Loaded '${name}': ${bestMesh.name} (${bestEntry.count} verts)`);

                        // Detach from original scene to ensure clean transform baking
                        const cleanMesh = bestMesh.clone();
                        resolve(cleanMesh);
                    } else {
                        reject(new Error(`No mesh found in ${name} (${url})`));
                    }
                },
                undefined,
                (error) => reject(error)
            );
        });
    }

    /**
     * Permanently applies the Object3D's position, rotation, and scale to its Geometry.
     * Resets the Object3D transform to identity.
     *
     * This is crucial for physics, as the simulation runs in World Space but usually
     * consumes raw vertex data which is in Local Space.
     */
    public bakeTransform(mesh: THREE.Mesh) {
        mesh.updateMatrixWorld(true);
        const geometry = mesh.geometry;

        const bakedGeometry = geometry.clone();
        bakedGeometry.applyMatrix4(mesh.matrixWorld);
        mesh.geometry = bakedGeometry;

        mesh.position.set(0, 0, 0);
        mesh.quaternion.set(0, 0, 0, 1);
        mesh.scale.set(1, 1, 1);
        mesh.updateMatrixWorld();
    }
}