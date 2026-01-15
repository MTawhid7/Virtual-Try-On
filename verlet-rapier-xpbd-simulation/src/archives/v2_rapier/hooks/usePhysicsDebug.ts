// src/v2/hooks/usePhysicsDebug.ts

import { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { RapierRigidBody } from '@react-three/rapier';
import type { PhysicsDebugInfo, PerformanceMetrics } from '../types/garment.types';
import { DEBUG_FLAGS, PERFORMANCE_THRESHOLDS } from '../config/debug.config';
import { validateRigidBody } from '../utils/validation';
import { PerformanceTimer } from '../utils/validation';

export function usePhysicsDebug(
    bodies: React.MutableRefObject<RapierRigidBody[]>,
    initialized: React.MutableRefObject<boolean>
) {
    const debugInfo = useRef<PhysicsDebugInfo>({
        bodyCount: 0,
        activeBodyCount: 0,
        constraintCount: 0,
        collisionCount: 0,
        averageVelocity: 0,
        maxVelocity: 0,
        isStable: false,
    });

    const performanceMetrics = useRef<PerformanceMetrics>({
        fps: 60,
        frameTime: 0,
        physicsTime: 0,
        skinningTime: 0,
        renderTime: 0,
    });

    const timer = useRef(new PerformanceTimer());
    const frameCount = useRef(0);
    const lastReportTime = useRef(performance.now());

    useFrame(() => {
        if (!initialized.current || !DEBUG_FLAGS.enablePhysicsDebug) return;

        timer.current.start();

        // Collect physics metrics
        let activeCount = 0;
        let totalVelocity = 0;
        let maxVel = 0;
        let validBodyCount = 0;

        bodies.current.forEach((body, idx) => {
            if (!validateRigidBody(body, idx)) return;

            validBodyCount++;

            try {
                // Check if body is sleeping (inactive)
                if (!body.isSleeping()) {
                    activeCount++;
                }

                // Calculate velocity magnitude
                const vel = body.linvel();
                const velMag = Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
                totalVelocity += velMag;
                maxVel = Math.max(maxVel, velMag);
            } catch (e) {
                // Body may have been removed
            }
        });

        const avgVel = validBodyCount > 0 ? totalVelocity / validBodyCount : 0;

        // Update debug info
        debugInfo.current = {
            bodyCount: bodies.current.length,
            activeBodyCount: activeCount,
            constraintCount: debugInfo.current.constraintCount, // Static
            collisionCount: 0, // Would need collision event listener
            averageVelocity: avgVel,
            maxVelocity: maxVel,
            isStable: avgVel < 0.1 && maxVel < 1.0, // Heuristic for stability
        };

        timer.current.end('physics_update');

        // Calculate FPS
        frameCount.current++;
        const now = performance.now();
        const elapsed = now - lastReportTime.current;

        if (elapsed >= 1000) { // Update every second
            performanceMetrics.current.fps = (frameCount.current / elapsed) * 1000;
            frameCount.current = 0;
            lastReportTime.current = now;

            // Log warnings for performance issues
            if (DEBUG_FLAGS.enableLogging) {
                if (performanceMetrics.current.fps < PERFORMANCE_THRESHOLDS.minFPS) {
                    console.warn(
                        `⚠️  [Performance] Low FPS: ${performanceMetrics.current.fps.toFixed(1)}`
                    );
                }

                if (maxVel > PERFORMANCE_THRESHOLDS.warningVelocity) {
                    console.warn(
                        `⚠️  [Physics] High velocity detected: ${maxVel.toFixed(2)} m/s`
                    );
                }

                if (maxVel > PERFORMANCE_THRESHOLDS.criticalVelocity) {
                    console.error(
                        `❌ [Physics] CRITICAL velocity: ${maxVel.toFixed(2)} m/s - Simulation may be unstable!`
                    );
                }
            }
        }

        // Update performance metrics
        const stats = timer.current.getStats('physics_update');
        performanceMetrics.current.physicsTime = stats.avg;
        performanceMetrics.current.frameTime = stats.avg; // Approximation
    });

    return {
        debugInfo: debugInfo.current,
        performanceMetrics: performanceMetrics.current,
    };
}