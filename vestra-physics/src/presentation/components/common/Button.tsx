// src/presentation/components/common/Button.tsx

import React from 'react';

/**
 * Props for the Button component.
 */
interface ButtonProps extends React.ButtonHTMLAttributes<HTMLButtonElement> {
    /** The visual style variant of the button. Defaults to 'neutral'. */
    variant?: 'primary' | 'danger' | 'success' | 'neutral';

    /** Whether the button is currently in an active/toggled state. */
    isActive?: boolean;
}

/**
 * A reusable button component with standard styling variants.
 * Supports primary, danger, success, and neutral themes.
 */
export const Button: React.FC<ButtonProps> = ({
    children,
    variant = 'neutral',
    isActive = false,
    style,
    disabled,
    ...props
}) => {
    let backgroundColor = '#333';
    let color = 'white';
    let border = '1px solid #555';

    // Style Logic
    if (variant === 'success') {
        backgroundColor = '#44ff44';
        color = '#000';
        border = 'none';
    } else if (variant === 'danger') {
        backgroundColor = '#ff4444';
        color = '#000';
        border = 'none';
    } else if (isActive) {
        backgroundColor = '#4488ff';
        border = '1px solid #4488ff';
    }

    // Opacity for disabled state
    const opacity = disabled ? 0.5 : 1;
    const cursor = disabled ? 'not-allowed' : 'pointer';

    return (
        <button
            style={{
                padding: '6px 12px',
                borderRadius: 4,
                fontSize: '0.85em',
                fontWeight: isActive || variant !== 'neutral' ? 'bold' : 'normal',
                cursor,
                backgroundColor,
                color,
                border,
                opacity,
                transition: 'all 0.2s ease',
                ...style
            }}
            disabled={disabled}
            {...props}
        >
            {children}
        </button>
    );
};