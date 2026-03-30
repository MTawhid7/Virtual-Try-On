// src/core/entities/Geometry.ts
/**
 * A simple 2D vector interface to avoid hard dependencies on Three.js
 * in the Core layer.
 */
export interface Vector2 {
    x: number;
    y: number;
}

/**
 * A simple 3D vector interface.
 */
export interface Vector3 {
    x: number;
    y: number;
    z: number;
}

/**
 * Represents a mesh that has been processed and is ready for simulation.
 * This is the "Data Transfer Object" (DTO) between the Asset Pipeline and the Physics Engine.
 * It uses TypedArrays for high-performance memory sharing with WASM.
 */
export interface ProcessedMesh {
    /** Flattened vertex positions [x, y, z, x, y, z...] */
    vertices: Float32Array;

    /** Triangle indices */
    indices: Uint32Array;

    /** Flattened vertex normals */
    normals: Float32Array;

    /** Flattened UV coordinates [u, v, u, v...] */
    uvs: Float32Array;

    /** Flattened tangents [x, y, z, w...] (Required for Anisotropic shading) */
    tangents: Float32Array;
}

/**
 * The result of the anatomical analysis performed on the body mesh.
 * Used to align garments correctly regardless of the body's pose or origin.
 */
export interface AnatomicalAnchors {
    /** The Y-coordinate (height) of the neck in logical space */
    neckY: number;

    /** The 2D center (XZ) of the neck */
    neckCenter: Vector2;

    /** The 2D center (XZ) of the spine/torso */
    spineCenter: Vector2;

    /** The 2D center (XZ) of the pelvis */
    pelvisCenter: Vector2;

    /** Whether the mesh was detected as being upside down */
    isUpsideDown: boolean;
}