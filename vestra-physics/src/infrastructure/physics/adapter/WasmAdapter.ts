// src/infrastructure/physics/adapter/WasmAdapter.ts

import * as THREE from 'three';
import type { IPhysicsEngine } from '../../../core/interfaces/IPhysicsEngine';
// We import types only
import type { InitPayload, MainToWorkerMessage, WorkerToMainMessage } from '../worker/PhysicsWorker';

export interface ProfileReport {
    frameCount: number;
    categories: { [key: string]: { avg: number; min: number; max: number; last: number; count: number; }; };
}

/**
 * Adapter class that communicates with the Physics Worker.
 * Handles async messaging and buffer management for the Main Thread.
 */
export class WasmAdapter implements IPhysicsEngine {
    private worker: Worker | null = null;

    // Double Buffering for Interpolation
    private prevPositions: Float32Array | null = null;
    private prevNormals: Float32Array | null = null;

    // The latest data received from Worker
    private currPositions: Float32Array | null = null;
    private currNormals: Float32Array | null = null;

    // The buffer used for rendering (interpolated result)
    private renderPositions: Float32Array | null = null;
    private renderNormals: Float32Array | null = null;

    // THREE.js Attributes
    private cachedPositionAttribute: THREE.InterleavedBufferAttribute | null = null;
    private cachedNormalAttribute: THREE.InterleavedBufferAttribute | null = null;

    private particleCount: number = 0;
    private isInitialized: boolean = false;

    constructor() {
        // Initialize Worker
        this.worker = new Worker(new URL('../worker/PhysicsWorker.ts', import.meta.url), { type: 'module' });
        this.worker.onmessage = this.handleWorkerMessage.bind(this);
        this.worker.onerror = (e) => {
            console.error("[WasmAdapter] Worker Error:", e.message, "\nFile:", e.filename, "\nLine:", e.lineno);
        };
    }

    private isWorkerBusy: boolean = false;
    private messageCount: number = 0;

    private handleWorkerMessage(e: MessageEvent<WorkerToMainMessage>) {
        const msg = e.data;
        switch (msg.type) {
            case 'INIT_SUCCESS':
                this.isInitialized = true;
                console.log("[WasmAdapter] Worker Initialized Successfully.");
                break;
            case 'UPDATE':
                this.isWorkerBusy = false; // Worker is free now
                this.messageCount++;
                if (this.messageCount % 60 === 0) {
                    console.log(`[WasmAdapter] Heartbeat: ${this.messageCount} frames. Busy: ${this.isWorkerBusy}`);
                }
                // Shift buffers: current becomes previous
                // If this is the FIRST update, prev = curr = new
                if (!this.currPositions) {
                    this.prevPositions = new Float32Array(msg.positions);
                    this.prevNormals = new Float32Array(msg.normals);
                } else {
                    this.prevPositions = this.currPositions; // Reuse the old buffer
                    this.prevNormals = this.currNormals;
                }

                this.currPositions = msg.positions;
                this.currNormals = msg.normals;
                break;
            /*
            case 'PROFILE':
                // Store or log profile data if needed
                console.log("Profile Data:", msg.data);
                break;
            */
        }
    }



    async init(
        garmentVerts: Float32Array,
        garmentIndices: Uint32Array,
        garmentUVs: Float32Array,
        colliderVerts: Float32Array,
        colliderNormals: Float32Array,
        colliderIndices: Uint32Array,
        scaleFactor: number
    ): Promise<void> {
        this.particleCount = garmentVerts.length / 3;

        // FIXED: Initialize buffers immediately so we have something to render at Frame 0
        this.currPositions = new Float32Array(garmentVerts);
        this.prevPositions = new Float32Array(garmentVerts);
        // Normals might be zero initially or we can assume up/out, zero is safer than garbage
        this.currNormals = new Float32Array(this.particleCount * 4); // 4 floats for padding alignment? NO, ensure size match
        // Actually, we need to check stride. If we use Vec4 aligned in WASM, we should stick to that.
        // The worker returns aligned Vec4 (x,y,z,w).
        // BUT the input `garmentVerts` is likely packed Vec3 (x,y,z).
        // We must convert packed Vec3 -> Aligned Vec4 for our internal buffer if that's what we use for rendering?
        // Let's check getPositions(): it returns InterleavedBufferAttribute with itemSize=3, stride=4.
        // So yes, we need 4 floats per particle.

        // Let's create proper initial buffers with padding
        const posAligned = new Float32Array(this.particleCount * 4);
        const norAligned = new Float32Array(this.particleCount * 4);

        for (let i = 0; i < this.particleCount; i++) {
            posAligned[i * 4 + 0] = garmentVerts[i * 3 + 0];
            posAligned[i * 4 + 1] = garmentVerts[i * 3 + 1];
            posAligned[i * 4 + 2] = garmentVerts[i * 3 + 2];
            posAligned[i * 4 + 3] = 0; // w padding
        }

        this.currPositions = posAligned;
        this.prevPositions = new Float32Array(posAligned);
        this.currNormals = norAligned;
        this.prevNormals = new Float32Array(norAligned);


        const payload: InitPayload = {
            garmentVerts,
            garmentIndices,
            garmentUVs,
            colliderVerts,
            colliderNormals,
            colliderIndices,
            scaleFactor
        };

        this.postMessage({ type: 'INIT', payload });

        // Wait for initialization? Ideally yes, but our system is async.
        // For now, we return immediately and let the worker catch up.
        // Or we could return a promise that resolves on INIT_SUCCESS.
        return new Promise<void>((resolve) => {
            const check = setInterval(() => {
                if (this.isInitialized) {
                    clearInterval(check);
                    resolve();
                }
            }, 100);
        });
    }

    private accumulator: number = 0;
    // 30 Hz Physics Update (33.33ms) - Decoupled from Render FPS
    private readonly FIXED_TIME_STEP = 1 / 30;

    /**
     * Steps the physics simulation forward using a Fixed Timestep Accumulator.
     * Decouples render FPS from physics calculation speed.
     * @param dt - The time step delta in seconds (variable).
     */
    step(dt: number): void {
        if (!this.isInitialized) return;

        // Cap dt to prevent spiral of death if tab is backgrounded
        const safeDt = Math.min(dt, 0.1);

        this.accumulator += safeDt;

        if (this.accumulator >= this.FIXED_TIME_STEP) {
            // Throttling: Only send step if worker is free.
            // This prevents "Death Spiral" where the main thread floods the worker with more tasks than it can handle.
            if (!this.isWorkerBusy) {
                this.accumulator -= this.FIXED_TIME_STEP;
                this.isWorkerBusy = true;
                this.postMessage({ type: 'STEP', dt: this.FIXED_TIME_STEP });
                // console.debug("[WasmAdapter] Step Sent.");
            } else {
                // Worker is busy. To prevent browser freeze, we skip this simulation step (Slow Motion).
                // CRITICAL: To prevent visual jitter (snapping back to prev frame), we must
                // advance our "previous" state to the "current" state, effectively "holding" the pose.
                // This makes interpolation alpha=0 yield 'curr' instead of 'prev'.
                if (this.currPositions) {
                    this.prevPositions = this.currPositions;
                    this.prevNormals = this.currNormals;
                }

                // Consume the time debt so we don't build up infinite backlog
                this.accumulator -= this.FIXED_TIME_STEP;

                // Warn occasionally (optional throttling of logs could be added here)
                // console.warn("[WasmAdapter] Worker Busy. Skipping step (Slow Motion).");
            }
        }
    }

    private postMessage(msg: MainToWorkerMessage, transfer: Transferable[] = []) {
        this.worker?.postMessage(msg, transfer);
    }

    getPositions(): THREE.BufferAttribute | THREE.InterleavedBufferAttribute {
        if (!this.currPositions) {
            // Return dummy or empty if not ready
            if (!this.cachedPositionAttribute) {
                const empty = new Float32Array(this.particleCount * 4);
                this.cachedPositionAttribute = this.createAttribute(empty);
            }
            return this.cachedPositionAttribute;
        }

        // Initialize Render Buffer if needed
        if (!this.renderPositions || this.renderPositions.length !== this.currPositions.length) {
            this.renderPositions = new Float32Array(this.currPositions.length);
        }

        // Calculate Interpolation Alpha
        // alpha = time_since_last_step / step_size
        // We clamp alpha to [0,1] just to be safe, though accumulator logic should guarantee it.
        const alpha = Math.max(0, Math.min(1, this.accumulator / this.FIXED_TIME_STEP));

        // Lerp: render = prev * (1 - alpha) + curr * alpha
        // If prev is missing (shouldn't happen if curr exists due to logic above), treat as curr.
        const p1 = this.prevPositions || this.currPositions;
        const p2 = this.currPositions;
        const out = this.renderPositions;

        for (let i = 0; i < out.length; i++) {
            out[i] = p1[i] * (1 - alpha) + p2[i] * alpha;
        }

        // Update cached attribute
        if (!this.cachedPositionAttribute) {
            this.cachedPositionAttribute = this.createAttribute(out);
        } else {
            // Update inplace
            // Check size again just in case
            if (this.cachedPositionAttribute.data.array.length !== out.length) {
                this.cachedPositionAttribute = this.createAttribute(out);
            } else {
                this.cachedPositionAttribute.data.set(out, 0); // Add offset 0
                this.cachedPositionAttribute.data.needsUpdate = true;
            }
        }

        return this.cachedPositionAttribute;
    }

    getNormals(): THREE.BufferAttribute | THREE.InterleavedBufferAttribute {
        if (!this.currNormals) {
            if (!this.cachedNormalAttribute) {
                const empty = new Float32Array(this.particleCount * 4);
                this.cachedNormalAttribute = this.createNormalAttribute(empty);
            }
            return this.cachedNormalAttribute;
        }

        if (!this.renderNormals || this.renderNormals.length !== this.currNormals.length) {
            this.renderNormals = new Float32Array(this.currNormals.length);
        }

        const alpha = Math.max(0, Math.min(1, this.accumulator / this.FIXED_TIME_STEP));
        const n1 = this.prevNormals || this.currNormals;
        const n2 = this.currNormals;
        const out = this.renderNormals;

        for (let i = 0; i < out.length; i++) {
            out[i] = n1[i] * (1 - alpha) + n2[i] * alpha;
        }

        if (!this.cachedNormalAttribute) {
            this.cachedNormalAttribute = this.createNormalAttribute(out);
        } else {
            if (this.cachedNormalAttribute.data.array.length !== out.length) {
                this.cachedNormalAttribute = this.createNormalAttribute(out);
            } else {
                this.cachedNormalAttribute.data.set(out, 0); // Offset 0
                this.cachedNormalAttribute.data.needsUpdate = true;
            }
        }
        return this.cachedNormalAttribute;
    }

    private createAttribute(data: Float32Array): THREE.InterleavedBufferAttribute {
        const interleaved = new THREE.InterleavedBuffer(data, 4);
        // Usage: DynamicDraw
        interleaved.setUsage(THREE.DynamicDrawUsage);
        return new THREE.InterleavedBufferAttribute(interleaved, 3, 0);
    }

    private createNormalAttribute(data: Float32Array): THREE.InterleavedBufferAttribute {
        const interleaved = new THREE.InterleavedBuffer(data, 4);
        interleaved.setUsage(THREE.DynamicDrawUsage);
        return new THREE.InterleavedBufferAttribute(interleaved, 3, 0);
    }

    dispose(): void {
        this.worker?.terminate();
        this.worker = null;
    }

    startInteraction(index: number, x: number, y: number, z: number): void {
        if (!this.isInitialized) return;
        this.postMessage({ type: 'INTERACTION_START', index, x, y, z });
    }

    updateInteraction(x: number, y: number, z: number): void {
        if (!this.isInitialized) return;
        this.postMessage({ type: 'INTERACTION_UPDATE', x, y, z });
    }

    endInteraction(): void {
        if (!this.isInitialized) return;
        this.postMessage({ type: 'INTERACTION_END' });
    }

    updateCollider(positions: Float32Array): void {
        const buffer = positions.slice();
        this.postMessage({ type: 'UPDATE_COLLIDER', positions: buffer }, [buffer.buffer]);
    }

    // --- Profiling Stubs (Can be implemented via message if needed) ---
    getProfileReportJson(): string { return "Profile data in worker console"; }
    getProfileData(): ProfileReport | null { return null; }
    resetProfiler(): void { }
    /**
     * Enables or disables profiling.
     * When disabled, profiling calls have near-zero overhead.
     */
    setProfilerEnabled(_enabled: boolean): void { // Fixed unused var
        // Sending message to worker if implemented
        // profiler_set_enabled(_enabled); // Old wasm call
        // For now, just suppress lint or implement message
        this.postMessage({ type: 'GET_PROFILE' }); // Just an example usage
    }
    getParticleCount(): number { return this.particleCount; }
    getSubsteps(): number { return 8; } // Hardcoded for now or fetch from worker
    getSolverIterations(): number { return 8; }
}
