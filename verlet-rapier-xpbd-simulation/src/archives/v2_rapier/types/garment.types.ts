// src/v2/types/garment.types.ts

import * as THREE from 'three';

export interface GarmentData {
    physicsGeometry: THREE.BufferGeometry;
    visualGeometry: THREE.BufferGeometry;
    constraints: [number, number, number][];
    vertexCount: number;
    skinning: SkinningData;
}

export interface SkinningData {
    indices: Int32Array;
    weights: Float32Array;
}

export interface PhysicsDebugInfo {
    bodyCount: number;
    activeBodyCount: number;
    constraintCount: number;
    collisionCount: number;
    averageVelocity: number;
    maxVelocity: number;
    isStable: boolean;
}

export interface PerformanceMetrics {
    fps: number;
    frameTime: number;
    physicsTime: number;
    skinningTime: number;
    renderTime: number;
}

export type MaterialType = 'silk' | 'cotton' | 'denim' | 'leather';