// src/presentation/components/overlay/ProfilerOverlay.tsx

import React, { useState, useEffect, useCallback } from 'react';
import type { ProfileReport } from '../../../infrastructure/physics/adapter/WasmAdapter';

interface ProfilerOverlayProps {
    getProfileData: () => ProfileReport | null;
    resetProfiler: () => void;
    particleCount: number;
    substeps: number;
    solverIterations: number;
}

/**
 * Profiler display category with color coding for performance.
 */
interface CategoryDisplay {
    name: string;
    key: string;
    indent?: number;
}

// Define display order and hierarchy
const CATEGORY_ORDER: CategoryDisplay[] = [
    { name: 'Frame Total', key: 'Frame', indent: 0 },
    { name: 'Broad Phase', key: 'BroadPhase', indent: 1 },
    { name: 'Aerodynamics', key: 'Aerodynamics', indent: 1 },
    { name: 'Integration', key: 'Integration', indent: 1 },
    { name: 'Mouse', key: 'MouseConstraint', indent: 1 },
    { name: 'Narrow Phase', key: 'NarrowPhase', indent: 1 },
    { name: 'Constraints', key: 'Constraints', indent: 1 },
    { name: '├ Distance', key: 'DistanceConstraint', indent: 2 },
    { name: '├ Bending', key: 'BendingConstraint', indent: 2 },
    { name: '├ Tether', key: 'TetherConstraint', indent: 2 },
    { name: '├ Area', key: 'AreaConstraint', indent: 2 },
    { name: '└ Collision', key: 'CollisionResolve', indent: 2 },
    { name: 'Self-Collision', key: 'SelfCollision', indent: 1 },
    { name: '├ Detect', key: 'SelfCollisionDetect', indent: 2 },
    { name: '├ Color', key: 'SelfCollisionColor', indent: 2 },
    { name: '└ Resolve', key: 'SelfCollisionResolve', indent: 2 },
    { name: 'Normals', key: 'Normals', indent: 1 },
];

/**
 * Returns a color based on time spent (green = fast, red = slow).
 * @param ms - Time in milliseconds
 * @param threshold - Time threshold for "slow" (default 2ms)
 */
function getPerformanceColor(ms: number, threshold = 2): string {
    if (ms < threshold * 0.5) return '#44ff44'; // Green - fast
    if (ms < threshold) return '#ffbb44'; // Yellow - moderate
    return '#ff4444'; // Red - slow
}

/**
 * Format milliseconds to a fixed-width string.
 */
function formatMs(ms: number): string {
    if (ms < 0.001) return '0.000';
    if (ms < 10) return ms.toFixed(3);
    if (ms < 100) return ms.toFixed(2);
    return ms.toFixed(1);
}

/**
 * Calculate percentage of frame time.
 */
function calcPercentage(ms: number, frameMs: number): number {
    if (frameMs <= 0) return 0;
    return Math.round((ms / frameMs) * 100);
}

/**
 * Real-time profiler overlay for physics engine performance analysis.
 * Displays timing breakdown for all pipeline stages.
 */
export const ProfilerOverlay: React.FC<ProfilerOverlayProps> = ({
    getProfileData,
    resetProfiler,
    particleCount,
    substeps,
    solverIterations
}) => {
    const [isExpanded, setIsExpanded] = useState(true);
    const [profileData, setProfileData] = useState<ProfileReport | null>(null);
    const [updateInterval, setUpdateInterval] = useState(500); // ms between updates

    // Update profile data periodically
    useEffect(() => {
        const interval = setInterval(() => {
            const data = getProfileData();
            setProfileData(data);
        }, updateInterval);

        return () => clearInterval(interval);
    }, [getProfileData, updateInterval]);

    const handleReset = useCallback(() => {
        resetProfiler();
        setProfileData(null);
    }, [resetProfiler]);

    const frameTime = profileData?.categories?.Frame?.avg ?? 0;
    const targetFrameTime = 16.67; // 60 FPS target
    const budget = Math.round((frameTime / targetFrameTime) * 100);

    return (
        <div style={{
            position: 'absolute',
            top: '10px',
            right: '10px',
            backgroundColor: 'rgba(20, 20, 30, 0.95)',
            color: '#fff',
            padding: '12px',
            borderRadius: '8px',
            fontFamily: 'ui-monospace, monospace',
            fontSize: '12px',
            minWidth: '280px',
            maxWidth: '340px',
            boxShadow: '0 4px 20px rgba(0, 0, 0, 0.5)',
            backdropFilter: 'blur(10px)',
            border: '1px solid rgba(255, 255, 255, 0.1)',
            zIndex: 1000,
        }}>
            {/* Header */}
            <div style={{
                display: 'flex',
                justifyContent: 'space-between',
                alignItems: 'center',
                marginBottom: '10px',
                borderBottom: '1px solid rgba(255, 255, 255, 0.1)',
                paddingBottom: '8px'
            }}>
                <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                    <span style={{
                        fontSize: '14px',
                        fontWeight: 'bold',
                        letterSpacing: '1px'
                    }}>
                        ⚡ PROFILER
                    </span>
                    <span style={{
                        fontSize: '10px',
                        color: '#888',
                        background: 'rgba(0,0,0,0.3)',
                        padding: '2px 6px',
                        borderRadius: '4px'
                    }}>
                        {particleCount.toLocaleString()} verts
                    </span>
                </div>
                <div style={{ display: 'flex', gap: '6px' }}>
                    <button
                        onClick={handleReset}
                        style={{
                            background: 'rgba(255, 100, 100, 0.2)',
                            border: '1px solid rgba(255, 100, 100, 0.3)',
                            color: '#ff8888',
                            padding: '2px 8px',
                            borderRadius: '4px',
                            cursor: 'pointer',
                            fontSize: '10px',
                        }}
                    >
                        Reset
                    </button>
                    <button
                        onClick={() => setIsExpanded(!isExpanded)}
                        style={{
                            background: 'rgba(255, 255, 255, 0.1)',
                            border: 'none',
                            color: '#fff',
                            padding: '2px 6px',
                            borderRadius: '4px',
                            cursor: 'pointer',
                            fontSize: '10px',
                        }}
                    >
                        {isExpanded ? '▼' : '▶'}
                    </button>
                </div>
            </div>

            {/* Budget Bar */}
            <div style={{ marginBottom: '10px' }}>
                <div style={{
                    display: 'flex',
                    justifyContent: 'space-between',
                    marginBottom: '4px'
                }}>
                    <span style={{ color: '#aaa', fontSize: '10px' }}>
                        Frame Budget (16.67ms @ 60fps)
                    </span>
                    <span style={{
                        fontWeight: 'bold',
                        color: budget > 100 ? '#ff4444' : budget > 75 ? '#ffbb44' : '#44ff44'
                    }}>
                        {budget}%
                    </span>
                </div>
                <div style={{
                    height: '6px',
                    background: 'rgba(255, 255, 255, 0.1)',
                    borderRadius: '3px',
                    overflow: 'hidden'
                }}>
                    <div style={{
                        height: '100%',
                        width: `${Math.min(budget, 100)}%`,
                        background: budget > 100
                            ? 'linear-gradient(90deg, #ff4444, #ff6666)'
                            : budget > 75
                                ? 'linear-gradient(90deg, #ffbb44, #ffdd66)'
                                : 'linear-gradient(90deg, #44ff44, #66ff66)',
                        transition: 'width 0.2s ease'
                    }} />
                </div>
            </div>

            {isExpanded && (
                <>
                    {/* Config Info */}
                    <div style={{
                        display: 'flex',
                        gap: '12px',
                        marginBottom: '10px',
                        padding: '6px 8px',
                        background: 'rgba(0, 0, 0, 0.3)',
                        borderRadius: '4px',
                        fontSize: '10px',
                        color: '#aaa'
                    }}>
                        <span>Substeps: <b style={{ color: '#fff' }}>{substeps}</b></span>
                        <span>Iterations: <b style={{ color: '#fff' }}>{solverIterations}</b></span>
                        <span>Total: <b style={{ color: '#fff' }}>{substeps * solverIterations}</b></span>
                    </div>

                    {/* Update Interval Control */}
                    <div style={{
                        display: 'flex',
                        alignItems: 'center',
                        gap: '8px',
                        marginBottom: '10px',
                        fontSize: '10px',
                        color: '#aaa'
                    }}>
                        <span>Update:</span>
                        {[100, 250, 500, 1000].map(ms => (
                            <button
                                key={ms}
                                onClick={() => setUpdateInterval(ms)}
                                style={{
                                    background: updateInterval === ms
                                        ? 'rgba(68, 255, 68, 0.2)'
                                        : 'rgba(255, 255, 255, 0.05)',
                                    border: updateInterval === ms
                                        ? '1px solid rgba(68, 255, 68, 0.3)'
                                        : '1px solid rgba(255, 255, 255, 0.1)',
                                    color: updateInterval === ms ? '#44ff44' : '#888',
                                    padding: '2px 6px',
                                    borderRadius: '3px',
                                    cursor: 'pointer',
                                    fontSize: '10px'
                                }}
                            >
                                {ms}ms
                            </button>
                        ))}
                    </div>

                    {/* Timing Table */}
                    <div style={{
                        display: 'grid',
                        gridTemplateColumns: '1fr auto auto auto',
                        gap: '2px 8px',
                        fontSize: '11px'
                    }}>
                        {/* Header */}
                        <div style={{ color: '#666', fontSize: '9px', paddingBottom: '4px' }}>PHASE</div>
                        <div style={{ color: '#666', fontSize: '9px', textAlign: 'right', paddingBottom: '4px' }}>AVG</div>
                        <div style={{ color: '#666', fontSize: '9px', textAlign: 'right', paddingBottom: '4px' }}>LAST</div>
                        <div style={{ color: '#666', fontSize: '9px', textAlign: 'right', paddingBottom: '4px' }}>%</div>

                        {/* Data Rows */}
                        {CATEGORY_ORDER.map(cat => {
                            const data = profileData?.categories?.[cat.key];
                            if (!data || data.count === 0) return null;

                            const indent = (cat.indent ?? 0) * 12;
                            const isTopLevel = cat.indent === 0;
                            const threshold = isTopLevel ? 16.67 : 2;

                            return (
                                <React.Fragment key={cat.key}>
                                    <div style={{
                                        paddingLeft: `${indent}px`,
                                        color: isTopLevel ? '#fff' : '#bbb',
                                        fontWeight: isTopLevel ? 'bold' : 'normal'
                                    }}>
                                        {cat.name}
                                    </div>
                                    <div style={{
                                        textAlign: 'right',
                                        color: getPerformanceColor(data.avg, threshold),
                                        fontWeight: isTopLevel ? 'bold' : 'normal'
                                    }}>
                                        {formatMs(data.avg)}
                                    </div>
                                    <div style={{
                                        textAlign: 'right',
                                        color: '#888'
                                    }}>
                                        {formatMs(data.last)}
                                    </div>
                                    <div style={{
                                        textAlign: 'right',
                                        color: '#666',
                                        fontSize: '10px'
                                    }}>
                                        {isTopLevel ? '—' : `${calcPercentage(data.avg, frameTime)}%`}
                                    </div>
                                </React.Fragment>
                            );
                        })}
                    </div>

                    {/* Summary */}
                    {profileData && (
                        <div style={{
                            marginTop: '10px',
                            paddingTop: '8px',
                            borderTop: '1px solid rgba(255, 255, 255, 0.1)',
                            fontSize: '10px',
                            color: '#888'
                        }}>
                            Frames: {profileData.frameCount?.toLocaleString() ?? 0}
                            <span style={{ marginLeft: '12px' }}>
                                Target FPS: <b style={{
                                    color: frameTime < targetFrameTime ? '#44ff44' : '#ff4444'
                                }}>
                                    {frameTime > 0 ? Math.round(1000 / frameTime) : '—'}
                                </b>
                            </span>
                        </div>
                    )}
                </>
            )}
        </div>
    );
};

export default ProfilerOverlay;
