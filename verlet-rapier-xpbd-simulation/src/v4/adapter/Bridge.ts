// src/v4/adapter/Bridge.ts
import init, { Simulation, init_hooks } from '../../../physics-core/pkg/physics_core';
import * as THREE from 'three';
import type { SkinningData } from '../utils/skinning'; // Import type

let wasmInitPromise: Promise<WebAssembly.Memory> | null = null;

export class Bridge {
    private sim: Simulation | null = null;
    private memory: WebAssembly.Memory | null = null;
    public positions: Float32Array | null = null;
    public debugData: Float32Array | null = null;

    async init(geometry: THREE.BufferGeometry) {
        if (!wasmInitPromise) {
            wasmInitPromise = (async () => {
                const wasm = await init();
                init_hooks();
                console.log("[Bridge] WASM Initialized Globally.");
                return wasm.memory;
            })();
        }
        this.memory = await wasmInitPromise;

        const posAttr = geometry.attributes.position;
        const vertexArray = new Float32Array(posAttr.array);

        const indexAttribute = geometry.index;
        if (!indexAttribute) {
            throw new Error("Geometry must be indexed!");
        }
        const indexArray = new Uint32Array(indexAttribute.array);

        this.sim = new Simulation(vertexArray, indexArray);

        this.sim.pin_collar(0.05);
        this.sim.init_tethers();

        const count = this.sim.get_count();
        const posPtr = this.sim.get_pos_ptr();
        const debugPtr = this.sim.get_debug_ptr();

        this.positions = new Float32Array(this.memory.buffer, posPtr, count * 3);
        this.debugData = new Float32Array(this.memory.buffer, debugPtr, count);

        console.log(`[Bridge] Loaded ${count} vertices into WASM.`);
    }

    update(dt: number) {
        if (this.sim) {
            this.sim.step(dt);
        }
    }

    getSDFConfig(): number[] | null {
        if (!this.sim) return null;
        return Array.from(this.sim.get_sdf_config());
    }

    // Interaction Helpers
    public startInteraction(index: number, x: number, y: number, z: number) {
        if (this.sim) this.sim.grab_particle(index, x, y, z);
    }

    public updateInteraction(x: number, y: number, z: number) {
        if (this.sim) this.sim.move_grab(x, y, z);
    }

    public endInteraction() {
        if (this.sim) this.sim.release_grab();
    }

    // Unused params removed to fix lint errors
    public getPhysicsParticleIndex(_visualFaceIndex: number, _skinning: SkinningData, _physicsIndex: THREE.BufferAttribute): number {
        return -1;
    }

    public findNearestParticle(point: THREE.Vector3): number {
        if (!this.positions) return -1;

        let minDist = Infinity;
        let bestIdx = -1;

        for (let i = 0; i < this.positions.length / 3; i++) {
            const x = this.positions[i * 3];
            const y = this.positions[i * 3 + 1];
            const z = this.positions[i * 3 + 2];

            const dx = x - point.x;
            const dy = y - point.y;
            const dz = z - point.z;
            const d2 = dx * dx + dy * dy + dz * dz;

            if (d2 < minDist) {
                minDist = d2;
                bestIdx = i;
            }
        }

        if (minDist > 0.01) return -1;
        return bestIdx;
    }
}