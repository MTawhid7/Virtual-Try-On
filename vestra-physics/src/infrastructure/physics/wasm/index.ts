
// src/infrastructure/physics/wasm/index.ts

// Point to the generated JS/Wasm bindings in the same directory
import init, {
    PhysicsEngine,
    initThreadPool,
    type InitOutput
} from './vestra_physics.js';

// Re-export as named and default for compatibility
export default init;
export { init, PhysicsEngine, initThreadPool };
export type { InitOutput };