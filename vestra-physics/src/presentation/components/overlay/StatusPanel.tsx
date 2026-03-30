// src/presentation/components/overlay/StatusPanel.tsx

import React from 'react';
import { useSimulationStore } from '../../state/useSimulationStore';

/**
 * Displays the current status of the simulation (Loading, Running, Paused).
 * Uses a visual indicator light and text label.
 */
export const StatusPanel: React.FC = () => {
    const { isReady, isRunning, isLoading, fps } = useSimulationStore();

    let statusText = "WAITING";
    let statusColor = "#aaa";

    if (isLoading) {
        statusText = "LOADING ASSETS...";
        statusColor = "#fbbf24"; // Amber
    } else if (isReady) {
        if (isRunning) {
            statusText = "RUNNING";
            statusColor = "#44ff44"; // Green
        } else {
            statusText = "PAUSED";
            statusColor = "#aaa";
        }
    }

    return (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '8px' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                <h3 style={{ margin: 0, fontSize: '1.1em', letterSpacing: '1px' }}>
                    VESTRA PHYSICS
                </h3>
                <span style={{
                    fontSize: '0.7em',
                    color: fps < 30 ? '#ff4444' : '#888',
                    fontWeight: 'bold',
                    background: 'rgba(0,0,0,0.3)',
                    padding: '2px 6px',
                    borderRadius: '4px'
                }}>
                    {fps} FPS
                </span>
            </div>

            <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                <div style={{
                    width: 8,
                    height: 8,
                    borderRadius: '50%',
                    backgroundColor: statusColor,
                    boxShadow: isRunning ? `0 0 8px ${statusColor}` : 'none',
                    transition: 'all 0.3s ease'
                }} />
                <span style={{ fontSize: '0.85em', color: '#ccc', fontWeight: 500 }}>
                    {statusText}
                </span>
            </div>
        </div>
    );
};