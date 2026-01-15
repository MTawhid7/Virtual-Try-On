import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { AppConfig, Disposable } from '../types';
import { VerletPhysicsEngine } from '../simulation/VerletPhysicsEngine';
import { InteractionController } from './InteractionController';
import { PerformanceMonitor } from '../infrastructure/PerformanceMonitor';
import { RendererSystem } from './systems/RendererSystem';
import { PhysicsSystem } from './systems/PhysicsSystem';
import { DebugSystem } from './systems/DebugSystem';
import { AssetLoaderSystem } from './systems/AssetLoaderSystem';

export class SceneManager implements Disposable {
    private renderer: RendererSystem;
    private physics: VerletPhysicsEngine;
    private physicsSystem: PhysicsSystem;
    private debug: DebugSystem;
    private perf: PerformanceMonitor;
    private controls: OrbitControls;
    private interaction: InteractionController | null = null;

    private clock = new THREE.Clock();
    private requestID: number | null = null;
    private garmentMesh: THREE.Mesh | null = null;

    constructor(config: AppConfig) {
        this.renderer = new RendererSystem(config);
        this.physics = new VerletPhysicsEngine();
        this.physicsSystem = new PhysicsSystem(this.physics, config);
        this.debug = new DebugSystem(this.renderer.scene, this.physics);
        this.perf = new PerformanceMonitor();

        this.controls = new OrbitControls(this.renderer.camera, this.renderer.renderer.domElement);
        this.controls.enableDamping = true;
    }

    async initialize(container: HTMLElement): Promise<void> {
        this.renderer.mount(container);
        this.setupLights();

        try {
            const mannequinMesh = await AssetLoaderSystem.loadMannequin(this.renderer.scene);
            this.garmentMesh = await AssetLoaderSystem.loadShirt(this.renderer.scene);

            // FIX: Do not move scene. Instead, point camera at the mannequin's center.
            if (mannequinMesh) {
                // Calculate center of the collider
                mannequinMesh.geometry.computeBoundingBox();
                const center = new THREE.Vector3();
                mannequinMesh.geometry.boundingBox?.getCenter(center);

                // Look at the chest/head area (Center Y + offset)
                // If center.y is waist, we add ~0.5 to look at chest
                const targetY = center.y + 0.4;

                this.controls.target.set(center.x, targetY, center.z);
                this.renderer.camera.position.set(center.x, targetY, center.z + 2.5);
                this.controls.update();
            }

            await this.physics.initialize(this.garmentMesh, { gravity: -9.81, iterations: 3 });

            if (mannequinMesh) {
                console.log("Setting up collider on:", mannequinMesh.name);
                this.physicsSystem.setBodyCollider(mannequinMesh);
                this.debug.setupBVHDebug(mannequinMesh);
            }

            this.interaction = new InteractionController(
                this.renderer.renderer.domElement,
                this.renderer.camera,
                this.garmentMesh,
                this.physics
            );
        } catch (e) {
            console.error("Simulation failed to start:", e);
        }

        this.startLoop();
    }

    private setupLights() {
        const ambient = new THREE.AmbientLight(0xffffff, 0.6);
        const dir = new THREE.DirectionalLight(0xffffff, 1.0);
        dir.position.set(5, 10, 10);
        this.renderer.scene.add(ambient, dir);
    }

    private startLoop() {
        const animate = () => {
            this.requestID = requestAnimationFrame(animate);
            const dt = this.clock.getDelta();

            this.perf.update();
            this.controls.update();
            this.physicsSystem.update(dt, this.garmentMesh);
            this.debug.update();

            this.renderer.renderer.render(this.renderer.scene, this.renderer.camera);
        };
        animate();
    }

    dispose(): void {
        if (this.requestID) cancelAnimationFrame(this.requestID);
        this.renderer.dispose();
        this.physics.dispose();
        this.debug.dispose();
        this.interaction?.dispose();
        this.perf.dispose();
        this.controls.dispose();
    }
}