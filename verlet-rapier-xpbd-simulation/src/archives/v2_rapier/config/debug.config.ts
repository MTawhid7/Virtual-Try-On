// src/v2/config/debug.config.ts

export const DEBUG_FLAGS = {
    enableLogging: true,
    enablePerformanceMetrics: true,
    enablePhysicsDebug: true,
    enableValidation: true,
    enableVisualDebug: false,
    logFrameData: false,
    logCollisions: false,
    logNaNErrors: true,
};

export const PERFORMANCE_THRESHOLDS = {
    minFPS: 30,
    maxFrameTime: 33.33, // ms
    maxPhysicsTime: 16.67, // ms
    warningVelocity: 10.0, // m/s
    criticalVelocity: 50.0, // m/s
};

export const DEBUG_COLORS = {
    physicsBody: 0xff0000,
    spring: 0x00ff00,
    collider: 0xffff00,
    force: 0xff00ff,
    normal: 0x0000ff,
    error: 0xff0000,
    warning: 0xffaa00,
    info: 0x00aaff,
};