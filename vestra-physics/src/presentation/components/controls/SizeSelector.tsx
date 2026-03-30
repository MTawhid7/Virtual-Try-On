// src/presentation/components/controls/SizeSelector.tsx

import React from 'react';
import { useSimulationStore } from '../../state/useSimulationStore';
import { Button } from '../common/Button';
import { useMediaQuery } from '../../hooks';
import type { ShirtSize } from '../../../core/entities/Garment';

const SIZES: ShirtSize[] = ['XS', 'S', 'M', 'L', 'XL', 'XXL'];

/**
 * UI Component for selecting the garment size (XS - XXL).
 * Interacts with the `SimulationStore` to trigger a garment reload when changed.
 * Automatically disables itself while the simulation is loading or initializing.
 */
export const SizeSelector: React.FC = () => {
    const { shirtSize, setShirtSize, isReady } = useSimulationStore();
    const isMobile = useMediaQuery('(max-width: 600px)');

    return (
        <div style={{ marginTop: isMobile ? '10px' : '15px' }}>
            <p style={{
                margin: '0 0 8px 0',
                fontSize: '0.75em',
                textTransform: 'uppercase',
                color: '#aaa',
                letterSpacing: '1px'
            }}>
                Select Size
            </p>
            <div style={{ display: 'flex', gap: '5px', flexWrap: 'wrap' }}>
                {SIZES.map((size) => (
                    <Button
                        key={size}
                        onClick={() => setShirtSize(size)}
                        disabled={!isReady}
                        isActive={shirtSize === size}
                        style={{
                            flex: '1 0 auto',
                            textAlign: 'center',
                            padding: isMobile ? '4px 8px' : '8px 12px',
                            fontSize: isMobile ? '0.8em' : '1em',
                            minWidth: isMobile ? '40px' : '50px'
                        }}
                    >
                        {size}
                    </Button>
                ))}
            </div>
        </div>
    );
};