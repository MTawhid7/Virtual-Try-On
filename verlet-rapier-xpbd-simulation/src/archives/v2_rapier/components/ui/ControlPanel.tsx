// src/v2/components/ui/ControlPanel.tsx

interface ControlPanelProps {
    showProxy: boolean;
    onToggleProxy: () => void;
    onReset: () => void;
}

export const ControlPanel = ({ showProxy, onToggleProxy, onReset }: ControlPanelProps) => {
    return (
        <div style={{
            position: 'absolute',
            top: 20,
            left: 20,
            background: 'rgba(255, 255, 255, 0.9)',
            padding: '15px',
            borderRadius: '10px',
            boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
            fontFamily: 'system-ui, sans-serif',
        }}>
            <h1 style={{
                margin: '0 0 5px 0',
                fontSize: '1.5rem',
                color: '#333',
                fontWeight: '600',
            }}>
                Garment Viewer V2
            </h1>
            <p style={{
                margin: '0 0 12px 0',
                color: '#666',
                fontSize: '0.9rem',
            }}>
                Engine: R3F + Rapier (WASM)
            </p>

            <div style={{ display: 'flex', gap: '8px', flexDirection: 'column' }}>
                <button
                    onClick={onToggleProxy}
                    style={{
                        padding: '8px 12px',
                        background: showProxy ? '#ff4444' : '#4a9eff',
                        color: '#fff',
                        border: 'none',
                        borderRadius: '6px',
                        cursor: 'pointer',
                        fontSize: '13px',
                        fontWeight: '500',
                        transition: 'all 0.2s',
                    }}
                    onMouseEnter={(e) => {
                        e.currentTarget.style.opacity = '0.9';
                    }}
                    onMouseLeave={(e) => {
                        e.currentTarget.style.opacity = '1';
                    }}
                >
                    {showProxy ? '👁️ Hide Physics Proxy' : '🔍 Show Physics Proxy'}
                </button>

                <button
                    onClick={onReset}
                    style={{
                        padding: '8px 12px',
                        background: '#fff',
                        color: '#333',
                        border: '1px solid #ddd',
                        borderRadius: '6px',
                        cursor: 'pointer',
                        fontSize: '13px',
                        fontWeight: '500',
                        transition: 'all 0.2s',
                    }}
                    onMouseEnter={(e) => {
                        e.currentTarget.style.background = '#f5f5f5';
                    }}
                    onMouseLeave={(e) => {
                        e.currentTarget.style.background = '#fff';
                    }}
                >
                    🔄 Reset Simulation
                </button>
            </div>
        </div>
    );
};