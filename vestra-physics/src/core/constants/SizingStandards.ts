// src/core/constants/SizingStandards.ts
import type { ShirtSize } from '../entities/Garment';

/**
 * Dimensions in Centimeters.
 * Based on standard retail sizing charts.
 */
export interface SizeDimensions {
    chestWidth: number; // cm
    bodyLength: number; // cm
}

/**
 * The Reference Size (Anchor) from which all other sizes are graded.
 * The original GLB assets should match these dimensions.
 */
export const ANCHOR_SIZE: ShirtSize = 'M';

/**
 * Standard grading table.
 * Used to calculate the scale factors for the physics engine.
 */
export const SHIRT_STANDARDS: Record<ShirtSize, SizeDimensions> = {
    'XS': { chestWidth: 45.0, bodyLength: 67.0 },
    'S': { chestWidth: 47.0, bodyLength: 70.0 },
    'M': { chestWidth: 51.0, bodyLength: 73.0 }, // ANCHOR
    'L': { chestWidth: 55.0, bodyLength: 75.0 },
    'XL': { chestWidth: 59.5, bodyLength: 77.5 },
    'XXL': { chestWidth: 63.0, bodyLength: 80.0 },
};