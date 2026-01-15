import * as THREE from 'three';
import * as BufferGeometryUtils from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { assetManager } from '../AssetManager';

export class AssetLoaderSystem {
    // REMOVED: centerScene method (It causes physics desync)

    static async loadMannequin(scene: THREE.Scene): Promise<THREE.Mesh | null> {
        try {
            const gltf = await assetManager.loadGLTF('/mannequin.glb');
            const model = gltf.scene;

            // Trust Blender's coordinates. Do not move.
            model.position.set(0, 0, 0);

            const geometries: THREE.BufferGeometry[] = [];

            model.traverse((child: any) => {
                if (child.isMesh) {
                    const mesh = child as THREE.Mesh;
                    mesh.castShadow = true;
                    mesh.receiveShadow = true;
                    mesh.material = new THREE.MeshStandardMaterial({
                        color: 0xeeeeee, roughness: 0.5, metalness: 0.1
                    });

                    // Bake transforms into geometry for the collider
                    const geo = mesh.geometry.clone();
                    geo.applyMatrix4(mesh.matrixWorld);
                    geometries.push(geo);
                }
            });

            scene.add(model);

            // Merge all parts into one physics collider
            if (geometries.length > 0) {
                const mergedGeo = BufferGeometryUtils.mergeGeometries(geometries);
                const colliderMesh = new THREE.Mesh(mergedGeo);
                colliderMesh.name = "Merged_Mannequin_Collider";
                return colliderMesh;
            } else {
                return null;
            }

        } catch (e) {
            console.error("Error loading mannequin:", e);
            return null;
        }
    }

    static async loadShirt(scene: THREE.Scene): Promise<THREE.Mesh> {
        try {
            const gltf = await assetManager.loadGLTF('/shirt.glb');
            const loadedMesh = this.findFirstMesh(gltf.scene);

            if (!loadedMesh) throw new Error("No mesh found in shirt.glb");

            let geo = loadedMesh.geometry.clone();
            geo.deleteAttribute('uv');
            geo.deleteAttribute('normal');
            geo = BufferGeometryUtils.mergeVertices(geo, 0.01);
            geo.computeVertexNormals();

            const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
                color: 0x44aa88,
                side: THREE.DoubleSide,
                roughness: 0.8
            }));

            mesh.castShadow = true;
            mesh.receiveShadow = true;
            mesh.position.set(0, 0, 0);

            scene.add(mesh);
            return mesh;
        } catch (e) {
            console.error("Error loading shirt:", e);
            throw e;
        }
    }

    private static findFirstMesh(parent: THREE.Object3D): THREE.Mesh | null {
        if ((parent as any).isMesh) return parent as THREE.Mesh;
        if (parent.children.length > 0) {
            for (const child of parent.children) {
                const found = this.findFirstMesh(child);
                if (found) return found;
            }
        }
        return null;
    }
}