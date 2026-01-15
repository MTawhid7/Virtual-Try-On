import React from 'react';
// import { Debug } from '@react-three/rapier';

interface PhysicsVisualizerProps {
    visible?: boolean;
}

/**
 * Visualizes the physics bodies and colliders using @react-three/rapier's built-in Debug component.
 * This is useful for verifying collider shapes and positions against the visual meshes.
 */
export const PhysicsVisualizer: React.FC<PhysicsVisualizerProps> = ({ visible = true }) => {
    if (!visible) return null;

    // <Debug /> component is not exported in the current @react-three/rapier version.
    // Debugging should be enabled via the <Physics debug /> prop in the parent component.
    return null;
};
