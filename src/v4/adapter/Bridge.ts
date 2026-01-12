// src/v4/adapter/Bridge.ts
import init, { Simulation, init_hooks } from '../../../physics-core/pkg/physics_core';
import * as THREE from 'three';

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

        // --- CHANGE: REMOVED WELDING ---
        // The geometry passed here must ALREADY be welded.

        const posAttr = geometry.attributes.position;
        const vertexArray = new Float32Array(posAttr.array);

        const indexAttribute = geometry.index;
        if (!indexAttribute) {
            throw new Error("Geometry must be indexed!");
        }
        const indexArray = new Uint32Array(indexAttribute.array);

        this.sim = new Simulation(vertexArray, indexArray);

        // Pinning & Tethers
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
}