// src/v3/presentation/components/DebugOverlay.tsx
import { useState } from 'react';
import { useFrame } from '@react-three/fiber';
import { Solver } from '../../engine/core/Solver';

// We need a way to access the solver from the UI.
// Since the solver is in React state/ref, we can pass it via a global store or context,
// but for simplicity in this architecture, we'll attach it to the window or use a prop if possible.
// Assuming this component is inside the Canvas, we can't easily render HTML DOM.
// So we will make this a standard React component that sits OUTSIDE the Canvas,
// but that requires lifting state.
//
// ALTERNATIVE: Render HTML inside Canvas using @react-three/drei/Html

import { Html } from '@react-three/drei';

export const DebugOverlay = ({ engine }: { engine: Solver | null }) => {
    const [stats, setStats] = useState({ inside: 0, contact: 0, fps: 0 });

    useFrame((state) => {
        if (!engine) return;
        // Throttle updates to 10fps to save React render cycles
        if (state.clock.elapsedTime % 0.1 < 0.02) {
            setStats({
                inside: engine.collider.debugData.stats.insideCount,
                contact: engine.collider.debugData.stats.contactCount,
                fps: Math.round(1 / state.clock.getDelta())
            });
        }
    });

    if (!engine) return null;

    return (
        <Html position={[-0.8, 1.8, 0]} style={{ pointerEvents: 'none' }}>
            <div style={{
                background: 'rgba(0,0,0,0.8)',
                color: '#0f0',
                padding: '12px',
                fontFamily: 'monospace',
                fontSize: '12px',
                borderRadius: '4px',
                width: '200px',
                border: '1px solid #333'
            }}>
                <div style={{ borderBottom: '1px solid #555', marginBottom: '4px', paddingBottom: '4px', fontWeight: 'bold' }}>
                    PHYSICS DIAGNOSTICS
                </div>
                <div>Particles: {engine.data.count}</div>
                <div>Substeps: 10</div>
                <div style={{ marginTop: '8px', color: stats.inside > 0 ? '#f55' : '#888' }}>
                    ⚠ Deep Rescue: {stats.inside}
                </div>
                <div style={{ color: '#ff0' }}>
                    ⚡ Surface Contact: {stats.contact}
                </div>
            </div>
        </Html>
    );
};