// src/v2/utils/materials.ts

import type { ClothMaterialParams } from '../types/physics.types';
import type { MaterialType } from '../types/garment.types';

export const CLOTH_MATERIALS: Record<MaterialType, ClothMaterialParams> = {
    silk: {
        name: 'Silk',
        description: 'Lightweight, flows gracefully, smooth drape',
        stiffness: 300,
        damping: 15,
        mass: 0.005,
        gravityScale: 1.2,
        linearDamping: 0.4,
        angularDamping: 0.6,
        colliderRadius: 0.012,
        restLengthScale: 0.96,
        friction: 0.3,
        restitution: 0.05,
    },

    cotton: {
        name: 'Cotton',
        description: 'Medium weight, natural drape, versatile',
        stiffness: 500,
        damping: 25,
        mass: 0.01,
        gravityScale: 1.5,
        linearDamping: 0.6,
        angularDamping: 0.7,
        colliderRadius: 0.015,
        restLengthScale: 0.98,
        friction: 0.5,
        restitution: 0.1,
    },

    denim: {
        name: 'Denim',
        description: 'Heavy, stiff, holds shape',
        stiffness: 1200,
        damping: 40,
        mass: 0.02,
        gravityScale: 2.0,
        linearDamping: 0.8,
        angularDamping: 0.9,
        colliderRadius: 0.018,
        restLengthScale: 1.0,
        friction: 0.7,
        restitution: 0.15,
    },

    leather: {
        name: 'Leather',
        description: 'Very stiff, rigid, minimal deformation',
        stiffness: 2000,
        damping: 60,
        mass: 0.03,
        gravityScale: 2.5,
        linearDamping: 0.9,
        angularDamping: 0.95,
        colliderRadius: 0.02,
        restLengthScale: 1.01,
        friction: 0.9,
        restitution: 0.2,
    },
};

export const MATERIAL_VISUAL_PROPERTIES = {
    silk: {
        color: '#e8d5f2',
        roughness: 0.2,
        metalness: 0.1,
        sheen: 0.5,
    },
    cotton: {
        color: '#44aa88',
        roughness: 0.8,
        metalness: 0.0,
        sheen: 0.0,
    },
    denim: {
        color: '#2a5f8f',
        roughness: 0.9,
        metalness: 0.0,
        sheen: 0.0,
    },
    leather: {
        color: '#3d2817',
        roughness: 0.4,
        metalness: 0.2,
        sheen: 0.3,
    },
};

export function getMaterialParams(type: MaterialType): ClothMaterialParams {
    return CLOTH_MATERIALS[type];
}

export function getAllMaterialTypes(): MaterialType[] {
    return Object.keys(CLOTH_MATERIALS) as MaterialType[];
}