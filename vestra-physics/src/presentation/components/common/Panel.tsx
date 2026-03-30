// src/presentation/components/common/Panel.tsx

import React from 'react';

interface PanelProps {
    children: React.ReactNode;
    style?: React.CSSProperties;
}

/**
 * A reusable UI container with a distinct "Glassmorphism" look.
 * Used for floating panels like the Status Panel and Controls.
 */
export const Panel: React.FC<PanelProps> = ({ children, style }) => {
    return (
        <div style={{
            padding: 15,
            background: 'rgba(0, 0, 0, 0.8)',
            color: 'white',
            borderRadius: 8,
            fontFamily: 'monospace',
            backdropFilter: 'blur(4px)',
            boxShadow: '0 4px 6px rgba(0,0,0,0.1)',
            ...style
        }}>
            {children}
        </div>
    );
};