// src/v2/utils/validation.ts

import * as THREE from 'three';
import { RapierRigidBody } from '@react-three/rapier';
import { DEBUG_FLAGS } from '../config/debug.config';
import type { ValidationResult } from '../types/physics.types';

/**
 * Validates a Vector3 for NaN or Infinity values
 */
export function validateVector3(
    vec: { x: number; y: number; z: number },
    context: string
): boolean {
    const hasNaN = isNaN(vec.x) || isNaN(vec.y) || isNaN(vec.z);
    const hasInfinity =
        !isFinite(vec.x) || !isFinite(vec.y) || !isFinite(vec.z);

    if ((hasNaN || hasInfinity) && DEBUG_FLAGS.logNaNErrors) {
        console.error(`❌ [Validation] Invalid vector at ${context}:`, {
            x: vec.x,
            y: vec.y,
            z: vec.z,
            hasNaN,
            hasInfinity,
        });
    }

    return !(hasNaN || hasInfinity);
}

/**
 * Validates a RigidBody is valid and accessible
 */
export function validateRigidBody(
    body: RapierRigidBody | null | undefined,
    index: number
): body is RapierRigidBody {
    if (!body) {
        if (DEBUG_FLAGS.logNaNErrors) {
            console.warn(`⚠️  [Validation] Body at index ${index} is null/undefined`);
        }
        return false;
    }

    try {
        const translation = body.translation();
        return validateVector3(translation, `Body[${index}].translation`);
    } catch (error) {
        if (DEBUG_FLAGS.logNaNErrors) {
            console.error(`❌ [Validation] Error accessing body ${index}:`, error);
        }
        return false;
    }
}

/**
 * Validates skinning data integrity
 */
export function validateSkinningData(
    indices: Int32Array,
    weights: Float32Array,
    visualVertexCount: number,
    physicsVertexCount: number
): ValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Check array sizes
    if (indices.length !== visualVertexCount) {
        errors.push(
            `Indices length (${indices.length}) doesn't match visual vertex count (${visualVertexCount})`
        );
    }

    if (weights.length !== visualVertexCount * 3) {
        errors.push(
            `Weights length (${weights.length}) doesn't match expected (${visualVertexCount * 3})`
        );
    }

    // Check for invalid indices
    for (let i = 0; i < indices.length; i++) {
        const idx = indices[i];
        if (idx < 0 || idx >= physicsVertexCount) {
            errors.push(`Invalid face index at vertex ${i}: ${idx}`);
            if (errors.length > 10) {
                errors.push('... (more errors suppressed)');
                break;
            }
        }
    }

    // Check for invalid weights
    for (let i = 0; i < weights.length; i++) {
        if (isNaN(weights[i]) || !isFinite(weights[i])) {
            errors.push(`Invalid weight at index ${i}: ${weights[i]}`);
            if (errors.length > 10) {
                errors.push('... (more errors suppressed)');
                break;
            }
        }
    }

    // Check weight normalization (should sum to ~1.0)
    for (let i = 0; i < visualVertexCount; i++) {
        const w1 = weights[i * 3];
        const w2 = weights[i * 3 + 1];
        const w3 = weights[i * 3 + 2];
        const sum = w1 + w2 + w3;

        if (Math.abs(sum - 1.0) > 0.01) {
            warnings.push(
                `Weights for vertex ${i} don't sum to 1.0: ${sum.toFixed(3)}`
            );
            if (warnings.length > 5) {
                warnings.push('... (more warnings suppressed)');
                break;
            }
        }
    }

    return {
        isValid: errors.length === 0,
        errors,
        warnings,
    };
}

/**
 * Validates geometry data
 */
export function validateGeometry(
    geometry: THREE.BufferGeometry,
    context: string
): ValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];

    if (!geometry.attributes.position) {
        errors.push(`${context}: Missing position attribute`);
    } else {
        const positions = geometry.attributes.position.array;
        for (let i = 0; i < positions.length; i++) {
            if (isNaN(positions[i]) || !isFinite(positions[i])) {
                errors.push(`${context}: Invalid position at index ${i}`);
                break;
            }
        }
    }

    if (!geometry.index) {
        warnings.push(`${context}: Missing index buffer (non-indexed geometry)`);
    }

    if (!geometry.attributes.normal) {
        warnings.push(`${context}: Missing normals`);
    }

    return {
        isValid: errors.length === 0,
        errors,
        warnings,
    };
}

/**
 * Logs validation results with appropriate formatting
 */
export function logValidationResult(result: ValidationResult, context: string) {
    if (!DEBUG_FLAGS.enableValidation) return;

    if (!result.isValid) {
        console.group(`❌ Validation Failed: ${context}`);
        result.errors.forEach(err => console.error(`  • ${err}`));
        console.groupEnd();
    }

    if (result.warnings.length > 0) {
        console.group(`⚠️  Validation Warnings: ${context}`);
        result.warnings.forEach(warn => console.warn(`  • ${warn}`));
        console.groupEnd();
    }

    if (result.isValid && result.warnings.length === 0) {
        console.log(`✅ Validation Passed: ${context}`);
    }
}

/**
 * Performance timer utility
 */
export class PerformanceTimer {
    private startTime: number = 0;
    private measurements: Map<string, number[]> = new Map();

    start() {
        this.startTime = performance.now();
    }

    end(label: string) {
        const duration = performance.now() - this.startTime;

        if (!this.measurements.has(label)) {
            this.measurements.set(label, []);
        }

        const measurements = this.measurements.get(label)!;
        measurements.push(duration);

        // Keep only last 60 measurements (1 second at 60fps)
        if (measurements.length > 60) {
            measurements.shift();
        }
    }

    getAverage(label: string): number {
        const measurements = this.measurements.get(label);
        if (!measurements || measurements.length === 0) return 0;

        const sum = measurements.reduce((a, b) => a + b, 0);
        return sum / measurements.length;
    }

    getStats(label: string) {
        const measurements = this.measurements.get(label);
        if (!measurements || measurements.length === 0) {
            return { avg: 0, min: 0, max: 0, count: 0 };
        }

        return {
            avg: this.getAverage(label),
            min: Math.min(...measurements),
            max: Math.max(...measurements),
            count: measurements.length,
        };
    }
}