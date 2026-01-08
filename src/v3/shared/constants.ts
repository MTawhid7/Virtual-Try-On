// src/v3/shared/constants.ts
export const PHYSICS_CONSTANTS = {
    gravity: -9.81,
    // OPTIMIZATION: Reduced from 20 to 10.
    // We will rely on "Soft Tethers" for stability instead of raw iteration count.
    substeps: 10,

    // VIBRATION FIX: Increased air resistance (Lower value = more drag).
    // 0.90 acts like "thick air", damping out micro-jitters.
    drag: 0.90,
    friction: 0.3,

    // Material Properties
    compliance: 0.00001,
    bendingCompliance: 2.0,

    // Interaction
    interaction: {
        stiffness: 0.0,
        maxForce: 1000,
        releaseDamping: 0.2
    },

    // HANGER FIX: Radius in meters around the neck to pin.
    // 0.07 = 7cm radius.
    pinRadius: 0.07,

    debug: {
        showProxy: false,
        showBounds: false,
        showActiveConstraint: true
    }
};