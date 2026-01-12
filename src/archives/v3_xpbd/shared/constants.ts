// src/v3/shared/constants.ts
export const PHYSICS_CONSTANTS = {
    gravity: -9.81,
    substeps: 10,
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

    // NEW: Vertical depth from the top to pin.
    // 0.03 = 3cm. This captures the collar ring but frees the shoulders.
    pinDepth: 0.03,

    debug: {
        showProxy: false,
        showBounds: false,
        showActiveConstraint: true
    }
};