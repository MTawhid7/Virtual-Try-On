import type { AppConfig, DeviceCapability } from '../types';

// Simple detection mock for Phase 0
export const detectCapability = (): DeviceCapability => {
    const concurrency = navigator.hardwareConcurrency || 4;
    if (concurrency >= 8) return 'high';
    if (concurrency >= 4) return 'medium';
    return 'low';
};

export const getRuntimeConfig = (capability: DeviceCapability): AppConfig => {
    const isHigh = capability === 'high';

    return {
        rendering: {
            targetFPS: 60,
            pixelRatioMax: isHigh ? 2.0 : 1.0,
            shadowMapSize: isHigh ? 1024 : 512,
            antialias: isHigh, // Conditional antialias (Page 4)
        },
        physics: {
            enabled: capability !== 'low',
            gravity: -9.81,
            constraintIterations: isHigh ? 10 : 5,
        },
    };
};