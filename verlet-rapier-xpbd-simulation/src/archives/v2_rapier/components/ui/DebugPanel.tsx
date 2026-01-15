// src/v2/components/ui/DebugPanel.tsx

import type { PhysicsDebugInfo, PerformanceMetrics } from '../../types/garment.types';

interface DebugPanelProps {
    debugInfo: PhysicsDebugInfo;
    performanceMetrics: PerformanceMetrics;
    settlingProgress?: number;
    isSettling?: boolean;
}

export const DebugPanel = ({
    debugInfo,
    performanceMetrics,
    settlingProgress = 0,
    isSettling = false
}: DebugPanelProps) => {
    const fpsColor = performanceMetrics.fps < 30 ? '#ff4444' :
        performanceMetrics.fps < 50 ? '#ffaa44' : '#44ff44';

    const velColor = debugInfo.maxVelocity > 10 ? '#ff4444' :
        debugInfo.maxVelocity > 5 ? '#ffaa44' : '#44ff44';

    return (
        <div style={{
            position: 'absolute',
            top: 10,
            right: 10,
            background: 'rgba(0, 0, 0, 0.85)',
            color: '#fff',
            padding: '12px',
            borderRadius: '8px',
            fontFamily: 'monospace',
            fontSize: '11px',
            minWidth: '220px',
            pointerEvents: 'none',
            userSelect: 'none',
            boxShadow: '0 4px 12px rgba(0,0,0,0.3)',
        }}>
            <div style={{ marginBottom: '10px', fontWeight: 'bold', fontSize: '12px', borderBottom: '1px solid #444', paddingBottom: '5px' }}>
                📊 DEBUG METRICS
            </div>

            {/* Performance */}
            <div style={{ marginBottom: '8px' }}>
                <div style={{ color: '#aaa', marginBottom: '3px' }}>PERFORMANCE</div>
                <div style={{ color: fpsColor }}>
                    FPS: {performanceMetrics.fps.toFixed(1)}
                </div>
                <div style={{ color: '#ddd' }}>
                    Frame: {performanceMetrics.frameTime.toFixed(2)}ms
                </div>
            </div>

            {/* Physics */}
            <div style={{ marginBottom: '8px' }}>
                <div style={{ color: '#aaa', marginBottom: '3px' }}>PHYSICS</div>
                <div style={{ color: '#ddd' }}>
                    Bodies: {debugInfo.bodyCount} ({debugInfo.activeBodyCount} active)
                </div>
                <div style={{ color: velColor }}>
                    Max Vel: {debugInfo.maxVelocity.toFixed(2)} m/s
                </div>
                <div style={{ color: '#ddd' }}>
                    Avg Vel: {debugInfo.averageVelocity.toFixed(3)} m/s
                </div>
                <div style={{ color: debugInfo.isStable ? '#44ff44' : '#ffaa44' }}>
                    Status: {debugInfo.isStable ? '✓ Stable' : '⚠ Settling'}
                </div>
            </div>

            {/* Settling */}
            {isSettling && (
                <div style={{ marginTop: '8px' }}>
                    <div style={{ color: '#aaa', marginBottom: '3px' }}>AUTO-DRAPING</div>
                    <div style={{
                        width: '100%',
                        height: '6px',
                        background: '#333',
                        borderRadius: '3px',
                        overflow: 'hidden',
                        marginTop: '4px',
                    }}>
                        <div style={{
                            width: `${settlingProgress * 100}%`,
                            height: '100%',
                            background: 'linear-gradient(90deg, #4a9eff, #44ff88)',
                            transition: 'width 0.3s ease',
                        }} />
                    </div>
                    <div style={{ color: '#ddd', marginTop: '2px', fontSize: '10px' }}>
                        {(settlingProgress * 100).toFixed(0)}% complete
                    </div>
                </div>
            )}
        </div>
    );
};
