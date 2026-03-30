
import init, { PhysicsEngine, initThreadPool } from '../wasm'; // profiler_get_report removed
// Constants must be hardcoded or imported. Importing is better if possible, but might add complexity to worker bundle.
// Let's hardcode for simplicity and robustness in worker environment first, or use relative import if Vite supports it.
// Given strict TS, relative import is best.
import { COLLIDER_INFLATION, COLLIDER_SMOOTHING_ITERATIONS } from '../../../core/constants/SimulationConstants';

// --- Worker Message Protocol ---

export type MainToWorkerMessage =
    | { type: 'INIT', payload: InitPayload }
    | { type: 'STEP', dt: number }
    | { type: 'INTERACTION_START', index: number, x: number, y: number, z: number }
    | { type: 'INTERACTION_UPDATE', x: number, y: number, z: number }
    | { type: 'INTERACTION_END' }
    | { type: 'UPDATE_COLLIDER', positions: Float32Array }
    | { type: 'GET_PROFILE' };

export type WorkerToMainMessage =
    | { type: 'INIT_SUCCESS' }
    | { type: 'UPDATE', positions: Float32Array, normals: Float32Array } // Transferable
    | { type: 'PROFILE', data: string }; // JSON string

export interface InitPayload {
    garmentVerts: Float32Array;
    garmentIndices: Uint32Array;
    garmentUVs: Float32Array;
    colliderVerts: Float32Array;
    colliderNormals: Float32Array;
    colliderIndices: Uint32Array;
    scaleFactor: number;
}

// --- Worker State ---

let engine: PhysicsEngine | null = null;
let wasmMemory: WebAssembly.Memory | null = null;
let isThreadPoolInitialized = false;

// --- Event Listener ---

self.onmessage = async (e: MessageEvent<MainToWorkerMessage>) => {
    const msg = e.data;

    try {
        switch (msg.type) {
            case 'INIT':
                await initialize(msg.payload);
                break;
            case 'STEP':
                step(msg.dt);
                break;
            case 'INTERACTION_START':
                engine?.set_interaction(msg.index, msg.x, msg.y, msg.z);
                break;
            case 'INTERACTION_UPDATE':
                engine?.update_interaction(msg.x, msg.y, msg.z);
                break;
            case 'INTERACTION_END':
                engine?.end_interaction();
                break;
            case 'UPDATE_COLLIDER':
                engine?.update_collider(msg.positions);
                break;
            /*
            case 'GET_PROFILE':
                if (engine) {
                    const report = profiler_get_report();
                    self.postMessage({ type: 'PROFILE', data: report } as WorkerToMainMessage);
                }
                break;
            */
        }
    } catch (err) {
        console.error("[PhysicsWorker] Error handling message:", err);
    }
};

// --- Core Logic ---

async function initialize(payload: InitPayload) {
    // 1. Initialize WASM module
    const wasm = await init();
    wasmMemory = wasm.memory;

    // 2. Initialize Thread Pool (Rayon)
    if (!isThreadPoolInitialized) {
        try {
            // Using navigator.hardwareConcurrency inside worker is standard
            // Clamp to 4 to prevent OOM on high-core machines during debugging
            const threads = Math.min(navigator.hardwareConcurrency || 4, 4);
            await initThreadPool(threads);
            isThreadPoolInitialized = true;
            console.log(`[PhysicsWorker] Thread pool initialized (${threads} threads).`);
        } catch (e) {
            // Might be already initialized if worker is reused or hot-reloaded
            console.warn("[PhysicsWorker] Thread pool init warning:", e);
        }
    }

    // 3. Create Physics Engine Instance
    // Note: We reconstruct TypedArrays from buffer transfer if needed, but payload comes as structured clone (copy).
    engine = new PhysicsEngine(
        payload.garmentVerts,
        payload.garmentIndices,
        payload.garmentUVs,
        payload.colliderVerts,
        payload.colliderNormals,
        payload.colliderIndices,
        COLLIDER_SMOOTHING_ITERATIONS,
        COLLIDER_INFLATION,
        payload.scaleFactor
    );

    console.log("[PhysicsWorker] Engine initialized.");

    // Dynamic Quality Adjustment
    // Simple heuristic: If concurrency <= 4 or User Agent smells like mobile, lower settings.
    const isMobile = navigator.userAgent.match(/Android|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i) || (navigator.hardwareConcurrency || 4) <= 4;

    if (isMobile) {
        console.log("[PhysicsWorker] Mobile device detected. Lowering simulation fidelity.");
        engine.set_substeps(6);
        engine.set_solver_iterations(8);
        // engine.set_self_collision_enabled(false); // Optional: Disable completely if totally broken
    } else {
        console.log("[PhysicsWorker] Desktop/High-End device detected. Using full fidelity (8x8).");
    }

    self.postMessage({ type: 'INIT_SUCCESS' } as WorkerToMainMessage);
}

function step(dt: number) {
    if (!engine || !wasmMemory) return;

    // 1. Run Physics Step
    engine.step(dt);

    // 2. Extract Data (Zero-Copy Read, but Copy-for-Transfer)
    const particleCount = engine.get_particle_count();
    const posPtr = engine.get_positions_ptr();
    const norPtr = engine.get_normals_ptr();

    // Create views into WASM memory
    // positions are aligned Vec4 (x, y, z, w), so stride is 4 floats per particle
    const posView = new Float32Array(wasmMemory.buffer, posPtr, particleCount * 4);
    const norView = new Float32Array(wasmMemory.buffer, norPtr, particleCount * 4);

    // Create COPIES to transfer ownership to Main Thread
    // We CANNOT transfer the view directly because it points to Shared WASM Memory (or at least owned by WASM).
    // .slice() creates a deep copy of the segment.
    const posBuffer = posView.slice();
    const norBuffer = norView.slice();

    // 3. Send Data back to Main Thread
    // Transfer the buffers to avoid another copy.
    (self as any).postMessage(
        {
            type: 'UPDATE',
            positions: posBuffer,
            normals: norBuffer
        } as WorkerToMainMessage,
        [posBuffer.buffer, norBuffer.buffer] // Transfer list
    );
}
