// src/infrastructure/rendering/TextureGenerator.ts

import * as THREE from 'three';

/**
 * Generates procedural textures for materials using HTML5 Canvas.
 * Useful for creating lightweight assets without external image files.
 */
export class TextureGenerator {
    /**
     * Generates a "Cotton Weave" Normal Map using HTML5 Canvas.
     *
     * Technique:
     * Draws a grid of semi-transparent lines to simulate threads (warp and weft).
     * Adds per-pixel noise to simulate fiber irregularity.
     */
    static generateFabricNormal(): THREE.CanvasTexture {
        const size = 1024;
        const canvas = document.createElement('canvas');
        canvas.width = size;
        canvas.height = size;
        const ctx = canvas.getContext('2d')!;

        // 1. Fill Neutral Blue (0, 0, 1) -> RGB(128, 128, 255)
        // This represents a "flat" normal.
        ctx.fillStyle = '#8080ff';
        ctx.fillRect(0, 0, size, size);

        // 2. Draw Weave Pattern
        const threads = 200;
        const step = size / threads;

        for (let i = 0; i < threads; i++) {
            const pos = i * step;

            // Vertical Threads
            ctx.fillStyle = i % 2 === 0 ? 'rgba(255, 128, 255, 0.1)' : 'rgba(0, 128, 255, 0.1)';
            ctx.fillRect(pos, 0, step, size);

            // Horizontal Threads
            ctx.fillStyle = i % 2 === 0 ? 'rgba(128, 255, 255, 0.1)' : 'rgba(128, 0, 255, 0.1)';
            ctx.fillRect(0, pos, size, step);
        }

        // 3. Add Noise
        const imageData = ctx.getImageData(0, 0, size, size);
        const data = imageData.data;
        for (let i = 0; i < data.length; i += 4) {
            const noise = (Math.random() - 0.5) * 20;
            data[i] = Math.max(0, Math.min(255, data[i] + noise));
            data[i + 1] = Math.max(0, Math.min(255, data[i + 1] + noise));
        }
        ctx.putImageData(imageData, 0, 0);

        const texture = new THREE.CanvasTexture(canvas);
        texture.wrapS = THREE.RepeatWrapping;
        texture.wrapT = THREE.RepeatWrapping;
        texture.repeat.set(4, 4);

        // CRITICAL: Disable Color Management for Normal Maps
        // Normal maps store vector data (XYZ), not color data (RGB).
        // Gamma correction would distort the vectors, causing lighting artifacts.
        texture.colorSpace = THREE.NoColorSpace;

        return texture;
    }
}