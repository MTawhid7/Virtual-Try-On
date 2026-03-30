// src/core/constants/SimulationConstants.ts
/**
 * Global configuration for the Physics Engine and Asset Pipeline.
 * Centralizing these values allows for easier tuning of the "Goldilocks" zone.
 */

// --- ASSET PIPELINE ---

/**
 * The target height (in meters) for the mannequin.
 * Used by MeshScaler to normalize arbitrary inputs to a standard physics scale.
 */
export const TARGET_BODY_HEIGHT = 1.75;

/**
 * The maximum triangle count for the Garment visual mesh/proxy.
 * Meshes larger than this are decimated to preserve FPS.
 */
export const GARMENT_RESOLUTION_BUDGET = 10000;

/**
 * The maximum triangle count for the Body collider.
 * Meshes larger than this are decimated to ensure fast spatial hashing.
 */
export const COLLIDER_RESOLUTION_BUDGET = 5000;

/**
 * The distance (in meters) to weld vertices during geometry processing.
 * 0.02 = 2cm. This fixes detached collars or hems in raw GLB exports.
 */
export const GEOMETRY_WELD_THRESHOLD = 0.02;


// --- PHYSICS ENGINE ---

/**
 * The invisible buffer (in meters) around the body collider.
 * 0.002 = 2mm.
 * This prevents z-fighting/clipping without creating a visible "Space Suit" gap.
 */
export const COLLIDER_INFLATION = 0.002;

/**
 * Number of Laplacian smoothing passes applied to the collider.
 * 0 = Raw Geometry (Most accurate for low-poly proxies).
 */
export const COLLIDER_SMOOTHING_ITERATIONS = 0;

/**
 * The maximum time step (in seconds) passed to the physics engine.
 * Prevents simulation explosion when the browser tab is inactive or lagging.
 */
export const MAX_PHYSICS_STEP = 0.05;