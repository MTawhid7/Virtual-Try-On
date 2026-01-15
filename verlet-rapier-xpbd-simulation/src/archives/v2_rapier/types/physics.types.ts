// src/v2/types/physics.types.ts

import { Vector3 } from 'three';

export interface ClothMaterialParams {
    name: string;
    stiffness: number;
    damping: number;
    mass: number;
    gravityScale: number;
    linearDamping: number;
    angularDamping: number;
    colliderRadius: number;
    restLengthScale: number;
    friction: number;
    restitution: number;
    description: string;
}

export interface PhysicsConfig {
    gravity: [number, number, number];
    timeStep: number;
    solverIterations: number;
    frictionIterations: number;
    collisionGroups: {
        mannequin: number;
        garment: number;
    };
}

export interface InteractionState {
    isDragging: boolean;
    draggedBodyIndex: number | null;
    mousePosition: Vector3 | null;
    dragStartTime: number;
}

export interface SettlingConfig {
    duration: number;
    forceMultiplier: number;
    enabled: boolean;
}

export interface ValidationResult {
    isValid: boolean;
    errors: string[];
    warnings: string[];
}