import { Vector3 } from 'three';

export interface Particle {
    position: Vector3;
    prevPosition: Vector3;
    originalPosition: Vector3;
    acceleration: Vector3;
    mass: number;
    pinned: boolean;
}

export interface Constraint {
    p1: number;
    p2: number;
    restDistance: number;
}

export interface CollisionSphere {
    position: Vector3;
    radius: number;
}