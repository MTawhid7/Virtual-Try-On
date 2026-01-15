import * as THREE from 'three';
import { MeshBVHHelper } from 'three-mesh-bvh';
import { VerletPhysicsEngine } from '../../simulation/VerletPhysicsEngine';

export class DebugSystem {
    private bvhHelper: MeshBVHHelper | null = null;
    private particlePoints: THREE.Points | null = null;

    private scene: THREE.Scene;
    private engine: VerletPhysicsEngine;

    constructor(scene: THREE.Scene, engine: VerletPhysicsEngine) {
        this.scene = scene;
        this.engine = engine;
    }

    // 1. Visualize the Mannequin Collider
    setupBVHDebug(colliderMesh: THREE.Mesh) {
        if (this.bvhHelper) {
            this.scene.remove(this.bvhHelper);
            this.disposeHelper(this.bvhHelper);
        }

        // Create helper with depth 10
        this.bvhHelper = new MeshBVHHelper(colliderMesh, 10);

        // Set visual properties
        this.bvhHelper.color.set(0x00ff00);
        this.bvhHelper.opacity = 0.5;

        // Access internal materials via casting to apply X-Ray effect
        const helperAny = this.bvhHelper as any;
        if (helperAny.meshMaterial) {
            helperAny.meshMaterial.transparent = true;
            helperAny.meshMaterial.depthTest = false;
            helperAny.meshMaterial.opacity = 0.3;
        }
        if (helperAny.edgeMaterial) {
            helperAny.edgeMaterial.depthTest = false;
            helperAny.edgeMaterial.color.set(0x00ff00);
        }

        this.bvhHelper.renderOrder = 9999;
        this.scene.add(this.bvhHelper);
    }

    // 2. Visualize Particles
    setupParticleDebug(particleCount: number) {
        if (this.particlePoints) {
            this.scene.remove(this.particlePoints);
            this.particlePoints.geometry.dispose();
            (this.particlePoints.material as THREE.Material).dispose();
        }

        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(particleCount * 3);
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

        const material = new THREE.PointsMaterial({
            color: 0xff0000,
            size: 0.03,
            sizeAttenuation: true,
            depthTest: false,
            transparent: true
        });

        this.particlePoints = new THREE.Points(geometry, material);
        this.particlePoints.renderOrder = 1000;
        this.scene.add(this.particlePoints);
    }

    update() {
        if (this.bvhHelper) {
            this.bvhHelper.update();
        }

        if (this.particlePoints) {
            // Access particles via getter
            const particles = (this.engine as any).getParticles ? (this.engine as any).getParticles() : null;

            if (particles) {
                const positions = this.particlePoints.geometry.attributes.position.array as Float32Array;

                for (let i = 0; i < particles.length; i++) {
                    positions[i * 3] = particles[i].position.x;
                    positions[i * 3 + 1] = particles[i].position.y;
                    positions[i * 3 + 2] = particles[i].position.z;
                }
                this.particlePoints.geometry.attributes.position.needsUpdate = true;
            }
        }
    }

    dispose() {
        if (this.bvhHelper) {
            this.scene.remove(this.bvhHelper);
            this.disposeHelper(this.bvhHelper);
            this.bvhHelper = null;
        }
        if (this.particlePoints) {
            this.scene.remove(this.particlePoints);
            this.particlePoints.geometry.dispose();
            (this.particlePoints.material as THREE.Material).dispose();
            this.particlePoints = null;
        }
    }

    // Helper to dispose MeshBVHHelper resources
    private disposeHelper(helper: MeshBVHHelper) {
        helper.traverse((child) => {
            if ((child as THREE.Mesh).isMesh) {
                const m = child as THREE.Mesh;
                m.geometry.dispose();
                if (Array.isArray(m.material)) {
                    m.material.forEach(mat => mat.dispose());
                } else {
                    (m.material as THREE.Material).dispose();
                }
            }
        });
    }
}