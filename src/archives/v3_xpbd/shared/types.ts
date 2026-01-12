// src/v3/shared/types.ts
export interface CapsuleData {
    radius: number;
    // Allow number[] to be compatible with JSON arrays
    start: number[] | [number, number, number];
    end: number[] | [number, number, number];
}

export type ColliderMap = Record<string, CapsuleData>;

export interface SimulationParams {
    gravity: number;
    substeps: number;
}