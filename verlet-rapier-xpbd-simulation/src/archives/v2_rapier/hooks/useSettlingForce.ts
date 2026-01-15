// src/v2/hooks/useSettlingForce.ts

import { useEffect, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { useRapier, RapierRigidBody } from '@react-three/rapier';
import { SETTLING_CONFIG } from '../config/physics.config';
import { DEBUG_FLAGS } from '../config/debug.config';
import { validateRigidBody } from '../utils/validation';

/**
 * Applies gentle downward forces to help cloth settle naturally
 * Active only during the first few seconds after initialization
 */
export function useSettlingForce(
    bodies: React.MutableRefObject<RapierRigidBody[]>,
    initialized: React.MutableRefObject<boolean>,
    enabled: boolean = SETTLING_CONFIG.enabled
) {
    const { rapier } = useRapier();
    const startTimeRef = useRef<number>(0);
    const isActiveRef = useRef(false);
    const appliedForceCount = useRef(0);

    // Reset settling timer when physics is initialized
    useEffect(() => {
        if (initialized.current && enabled) {
            startTimeRef.current = performance.now();
            isActiveRef.current = true;
            appliedForceCount.current = 0;

            if (DEBUG_FLAGS.enableLogging) {
                console.log('🌊 [Settling] Starting auto-draping phase');
            }
        }
    }, [initialized.current, enabled]);

    useFrame(() => {
        if (!enabled || !isActiveRef.current || !initialized.current) return;

        const elapsed = performance.now() - startTimeRef.current;

        // Check if settling period is over
        if (elapsed > SETTLING_CONFIG.duration) {
            if (isActiveRef.current) {
                isActiveRef.current = false;

                if (DEBUG_FLAGS.enableLogging) {
                    console.log(
                        `✅ [Settling] Auto-draping complete (applied ${appliedForceCount.current} forces)`
                    );
                }
            }
            return;
        }

        // Calculate force multiplier (fade out over time)
        const progress = elapsed / SETTLING_CONFIG.duration;
        const fadeOutCurve = 1 - Math.pow(progress, 2); // Quadratic fade
        const currentForce = SETTLING_CONFIG.impulseForce * fadeOutCurve;

        // Apply gentle downward impulse to dynamic bodies
        let appliedThisFrame = 0;
        bodies.current.forEach((body, idx) => {
            if (!validateRigidBody(body, idx)) return;

            // Only affect dynamic bodies (not pinned)
            if (body.bodyType() === rapier.RigidBodyType.Dynamic) {
                try {
                    body.applyImpulse({ x: 0, y: currentForce, z: 0 }, true);
                    appliedThisFrame++;
                } catch (e) {
                    // Silent fail - body may have been removed
                }
            }
        });

        appliedForceCount.current += appliedThisFrame;

        // Log progress every 500ms
        if (DEBUG_FLAGS.enableLogging && Math.floor(elapsed / 500) !== Math.floor((elapsed - 16) / 500)) {
            const progressPercent = (progress * 100).toFixed(0);
            console.log(
                `🌊 [Settling] ${progressPercent}% complete ` +
                `(force: ${(currentForce * 1000).toFixed(2)}mN)`
            );
        }
    });

    return {
        isActive: isActiveRef.current,
        progress: isActiveRef.current
            ? (performance.now() - startTimeRef.current) / SETTLING_CONFIG.duration
            : 1.0,
    };
}