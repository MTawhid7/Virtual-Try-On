// src/presentation/canvas/CameraRig.tsx

import React, { useEffect } from 'react';
import { useThree } from '@react-three/fiber';
import { useSimulationStore } from '../state/useSimulationStore';

/**
 * A logical component that manages the camera's position and target.
 * specific behaviors:
 * - Resets camera to "Hero Shot" position when the simulation becomes ready.
 * - Updates OrbitControls target to focus on the mannequin's chest.
 * - This component does not render any visible UI.
 */
export const CameraRig: React.FC = () => {
    const { camera, controls } = useThree();
    const { isReady } = useSimulationStore();

    useEffect(() => {
        if (isReady) {
            // Reset camera to the "Hero Shot" position
            camera.position.set(0, 1.4, 2.5);
            camera.lookAt(0, 1.0, 0);

            // Update OrbitControls target to center on the chest/torso
            if (controls) {
                // @ts-ignore - OrbitControls type definition sometimes misses 'target'
                controls.target.set(0, 1.0, 0);
                // @ts-ignore
                controls.update();
            }
        }
    }, [isReady, camera, controls]);

    return null; // This component has no visual representation
};