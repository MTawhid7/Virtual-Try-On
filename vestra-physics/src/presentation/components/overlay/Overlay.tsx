// src/presentation/components/overlay/Overlay.tsx

import React from 'react'; // { useState, useEffect, useCallback } removed
import { Panel } from '../common/Panel';
import { StatusPanel } from './StatusPanel';
// import { ProfilerOverlay } from './ProfilerOverlay';
import { SimulationControls } from '../controls/SimulationControls';
import { SizeSelector } from '../controls/SizeSelector';
import { useSimulationStore } from '../../state/useSimulationStore';
import { useMediaQuery } from '../../hooks'; // useProfiler removed

/**
 * The main UI overlay layer.
 * Positions the control panels and legends on top of the 3D canvas.
 * Manages pointer events to ensure clicks pass through to the canvas where appropriate.
 */
export const Overlay: React.FC = () => {
    const { error } = useSimulationStore(); // Removed isReady, substeps, solverIterations
    const isMobile = useMediaQuery('(max-width: 600px)');
    // const [showProfiler, setShowProfiler] = useState(!isMobile);

    /*
    const {
        getProfileData,
        resetProfiler,
    } = useProfiler(500);
    */

    /*
    // Toggle profiler with keyboard shortcut (P key)
    const handleKeyDown = useCallback((e: KeyboardEvent) => {
        if (e.key === 'p' || e.key === 'P') {
            setShowProfiler(prev => !prev);
        }
    }, []);

    useEffect(() => {
        window.addEventListener('keydown', handleKeyDown);
        return () => window.removeEventListener('keydown', handleKeyDown);
    }, [handleKeyDown]);
    */

    return (
        <>
            <div style={{
                position: 'absolute',
                top: isMobile ? 10 : 20,
                left: isMobile ? 10 : 20,
                zIndex: 10, // Ensure it sits above the Canvas
                pointerEvents: 'none', // Let clicks pass through to Canvas...
                display: 'flex',
                flexDirection: 'column',
                gap: '10px',
                width: isMobile ? 'min(300px, calc(100vw - 20px))' : '280px'
            }}>
                {/* ...but re-enable pointer events for the panel itself */}
                <div style={{ pointerEvents: 'auto' }}>
                    <Panel style={{ padding: isMobile ? '12px' : '15px' }}>
                        <StatusPanel />

                        {error && (
                            <div style={{
                                marginTop: 10,
                                padding: 8,
                                background: 'rgba(255, 0, 0, 0.2)',
                                border: '1px solid #ff4444',
                                borderRadius: 4,
                                fontSize: '0.8em',
                                color: '#ffcccc'
                            }}>
                                <strong>Error:</strong> {error}
                            </div>
                        )}

                        <SimulationControls />
                        <SizeSelector />

                        {/* Profiler Toggle - COMMENTED OUT FOR PERFORMANCE
                        <div style={{
                            marginTop: '10px',
                            paddingTop: '10px',
                            borderTop: '1px solid rgba(255,255,255,0.1)',
                            display: 'flex',
                            alignItems: 'center',
                            gap: '8px'
                        }}>
                            <button
                                onClick={() => setShowProfiler(!showProfiler)}
                                style={{
                                    background: showProfiler ? 'rgba(68, 255, 68, 0.2)' : 'rgba(255, 255, 255, 0.1)',
                                    border: showProfiler ? '1px solid rgba(68, 255, 68, 0.3)' : '1px solid rgba(255, 255, 255, 0.2)',
                                    color: showProfiler ? '#44ff44' : '#888',
                                    padding: '6px 12px',
                                    borderRadius: '4px',
                                    cursor: 'pointer',
                                    fontSize: '0.8em',
                                    flex: 1,
                                    display: 'flex',
                                    alignItems: 'center',
                                    gap: '6px'
                                }}
                            >
                                <span>⚡</span>
                                <span>Profiler</span>
                                <span style={{
                                    marginLeft: 'auto',
                                    fontSize: '0.8em',
                                    color: '#666'
                                }}>
                                    [P]
                                </span>
                            </button>
                        </div>
                        */}
                    </Panel>
                </div>

                {!isMobile && (
                    <div style={{ pointerEvents: 'auto' }}>
                        <Panel style={{ fontSize: '0.75em', color: '#888' }}>
                            <p style={{ margin: 0 }}>
                                <strong>Controls:</strong><br />
                                • Left Click + Drag to Rotate<br />
                                • Right Click to Pan<br />
                                • Scroll to Zoom<br />
                                • Click & Drag Cloth to Interact
                            </p>
                        </Panel>
                    </div>
                )}
            </div>

            {/* Profiler Overlay (Right Side) - COMMENTED OUT FOR PERFORMANCE
            {showProfiler && isReady && (
                <div style={{ pointerEvents: 'auto' }}>
                    <ProfilerOverlay
                        getProfileData={getProfileData}
                        resetProfiler={resetProfiler}
                        particleCount={0}
                        substeps={substeps}
                        solverIterations={solverIterations}
                    />
                </div>
            )}
            */}
        </>
    );
};