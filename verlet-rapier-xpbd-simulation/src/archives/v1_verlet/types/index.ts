export type DeviceCapability = 'high' | 'medium' | 'low';

export interface AppConfig {
    rendering: {
        targetFPS: number;
        pixelRatioMax: number;
        shadowMapSize: number;
        antialias: boolean;
    };
    physics: {
        enabled: boolean;
        gravity: number;
        constraintIterations: number;
    };
}

// Disposable interface (Reference: Page 7 - Memory Management)
export interface Disposable {
    dispose(): void;
}