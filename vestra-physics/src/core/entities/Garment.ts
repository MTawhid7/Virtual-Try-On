// src/core/entities/Garment.ts
/**
 * Represents the available standard sizes for garments.
 * These correspond to the keys in the sizing standards table.
 */
export type ShirtSize = 'XS' | 'S' | 'M' | 'L' | 'XL' | 'XXL';

/**
 * Metadata associated with a garment mesh.
 * Useful for future features where we load garments with specific properties.
 */
export interface GarmentMetadata {
    id: string;
    name: string;
    size: ShirtSize;
    fabricType?: 'cotton' | 'denim' | 'silk';
}