// src/v2/config/physics.config.ts

import type { PhysicsConfig } from '../types/physics.types';

export const PHYSICS_CONFIG: PhysicsConfig = {
    gravity: [0, -9.81, 0],
    timeStep: 1 / 60, // Fixed 60 FPS timestep
    solverIterations: 12, // Increased for stability
    frictionIterations: 8, // Increased for stability
    collisionGroups: {
        mannequin: 0x0001, // Group 1: Interacts with everything
        garment: 0x0002,   // Group 2: Only interacts with Group 1
    }
};

export const INTERACTION_CONFIG = {
    grabRadius: 0.2,           // How close mouse needs to be (meters)
    springStiffness: 50,       // Mouse spring force
    springDamping: 5,          // Mouse spring damping
    maxDragDistance: 2.0,      // Max distance from camera
    dragPlaneOffset: -1.5,     // Distance of drag plane from camera
};

export const SETTLING_CONFIG = {
    duration: 2000,            // Settling period (ms)
    impulseForce: -0.002,      // Downward impulse per frame
    enabled: true,
};

export const VISUAL_CONFIG = {
    proxyVisible: false,       // Show low-poly physics mesh
    showSprings: false,        // Draw spring connections
    showColliders: false,      // Draw collision spheres
    showForces: false,         // Draw force vectors
    showNormals: false,        // Draw vertex normals
};
