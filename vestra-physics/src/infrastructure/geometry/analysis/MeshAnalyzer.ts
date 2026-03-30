// src/infrastructure/geometry/analysis/MeshAnalyzer.ts

import * as THREE from 'three';
import { OrientationDetector } from './orientation/OrientationDetector';
import { SpineRegressor } from './SpineRegressor';
import type { AnatomicalAnchors } from '../../../core/entities/Geometry';

/**
 * Orchestrates the anatomical analysis of a body mesh.
 * It combines orientation detection, bounds analysis, and spine regression
 * to identify key landmarks (Neck, Spine, Pelvis) required for garment fitting.
 */
export class MeshAnalyzer {
    /**
     * Analyze a body geometry to find its anatomical structure.
     * Handles upside-down meshes and normalizes the data into a standard "Anchors" format.
     */
    static analyzeBody(geometry: THREE.BufferGeometry): AnatomicalAnchors {
        geometry.computeBoundingBox();
        const box = geometry.boundingBox!;
        const height = box.max.y - box.min.y;

        // 1. Detect Orientation
        const isUpsideDown = OrientationDetector.isUpsideDown(geometry, box);

        // 2. Define Logical Coordinates
        // If upside down, we analyze "backwards" relative to world space
        const logicalMinY = isUpsideDown ? box.max.y : box.min.y;
        const logicalHeight = isUpsideDown ? -height : height;

        // 3. Perform Regression
        const spineData = SpineRegressor.analyze(geometry, logicalMinY, logicalHeight);

        return {
            neckY: logicalMinY + (logicalHeight * 0.87),
            neckCenter: spineData.neckCenter,
            spineCenter: spineData.spineCenter,
            pelvisCenter: spineData.pelvisCenter,
            isUpsideDown
        };
    }
}