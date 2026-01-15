import React, { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { useRapier } from '@react-three/rapier';
import { LineSegments, BufferGeometry, BufferAttribute } from 'three';

interface VelocityVisualizerProps {
    color?: string;
    scale?: number; // Scale factor for velocity vector length
    visible?: boolean;
}

/**
 * Visualizes the velocity vectors of all dynamic rigid bodies.
 * Draws a line from the body's center of mass to (center + velocity * scale).
 */
export const VelocityVisualizer: React.FC<VelocityVisualizerProps> = ({
    color = '#00ff00',
    scale = 0.5,
    visible = true,
}) => {
    const { world } = useRapier();
    const linesRef = useRef<LineSegments>(null);
    const geometryRef = useRef<BufferGeometry>(null);

    // Maximum number of bodies to visualize to prevent memory issues
    const MAX_BODIES = 200;

    useFrame(() => {
        if (!visible || !linesRef.current || !geometryRef.current) return;

        const positions: number[] = [];

        // Iterate active bodies (or all bodies)
        let count = 0;

        // Note: forEachBody might iterate ALL bodies (static, dynamic).
        // We usually only care about dynamic bodies for velocity.
        // eslint-disable-next-line @typescript-eslint/no-explicit-any
        const rapierWorld = world as any;

        if (rapierWorld.forEachBody) {
            // eslint-disable-next-line @typescript-eslint/no-explicit-any
            rapierWorld.forEachBody((body: any) => {
                if (count >= MAX_BODIES) return;

                // Filter for dynamic bodies only
                if (body.isDynamic() && !body.isSleeping()) {
                    const vel = body.linvel();
                    const msg = Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);

                    // Only draw if moving significantly
                    if (msg > 0.01) {
                        const t = body.translation();

                        // Start point: Center of body
                        positions.push(t.x, t.y, t.z);

                        // End point: Center + Velocity * Scale
                        positions.push(
                            t.x + vel.x * scale,
                            t.y + vel.y * scale,
                            t.z + vel.z * scale
                        );

                        count++;
                    }
                }
            });
        }

        // Update geometry
        geometryRef.current.setAttribute(
            'position',
            new BufferAttribute(new Float32Array(positions), 3)
        );
        // Important: set draw range to actual line count (2 vertices per line)
        geometryRef.current.setDrawRange(0, count * 2);
    });

    return (
        <lineSegments ref={linesRef} frustumCulled={false}>
            <bufferGeometry ref={geometryRef}>
                <bufferAttribute
                    attach="attributes-position"
                    args={[new Float32Array(MAX_BODIES * 2 * 3), 3]}
                />
            </bufferGeometry>
            <lineBasicMaterial color={color} />
        </lineSegments>
    );
};
