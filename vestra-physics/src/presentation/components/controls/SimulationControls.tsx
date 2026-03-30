// src/presentation/components/controls/SimulationControls.tsx

import React from 'react';
import { useSimulationStore } from '../../state/useSimulationStore';
import { Button } from '../common/Button';
import { useMediaQuery } from '../../hooks';

/**
 * The main control panel for the physics simulation.
 * Allows the user to Start/Stop the simulation and displays current state.
 */
export const SimulationControls: React.FC = () => {
    const { isReady, isRunning, toggleSimulation } = useSimulationStore();
    const isMobile = useMediaQuery('(max-width: 600px)');

    return (
        <div style={{ display: 'flex', gap: '10px', marginTop: isMobile ? '8px' : '10px' }}>
            <Button
                onClick={toggleSimulation}
                disabled={!isReady}
                variant={isRunning ? 'danger' : 'success'}
                style={{
                    width: '100%',
                    padding: isMobile ? '8px' : '12px',
                    fontSize: isMobile ? '0.85em' : '1em'
                }}
            >
                {isRunning ? "STOP SIMULATION" : "START SIMULATION"}
            </Button>
        </div>
    );
};