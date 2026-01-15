// src/v2/components/ui/MaterialSelector.tsx

import type { MaterialType } from '../../types/garment.types';
import { getAllMaterialTypes, CLOTH_MATERIALS } from '../../utils/materials';

interface MaterialSelectorProps {
    currentMaterial: MaterialType;
    onMaterialChange: (material: MaterialType) => void;
}

export const MaterialSelector = ({ currentMaterial, onMaterialChange }: MaterialSelectorProps) => {
    const materials = getAllMaterialTypes();

    return (
        <div style={{
            position: 'absolute',
            bottom: 20,
            left: 20,
            background: 'rgba(255, 255, 255, 0.95)',
            padding: '15px',
            borderRadius: '10px',
            boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
            fontFamily: 'system-ui, sans-serif',
            minWidth: '200px',
        }}>
            <div style={{
                fontSize: '13px',
                fontWeight: '600',
                marginBottom: '10px',
                color: '#333',
            }}>
                🧵 Fabric Material
            </div>

            <div style={{ display: 'flex', flexDirection: 'column', gap: '8px' }}>
                {materials.map(mat => {
                    const isSelected = mat === currentMaterial;
                    const matInfo = CLOTH_MATERIALS[mat];

                    return (
                        <button
                            key={mat}
                            onClick={() => onMaterialChange(mat)}
                            style={{
                                padding: '10px 12px',
                                border: isSelected ? '2px solid #4a9eff' : '1px solid #ddd',
                                borderRadius: '6px',
                                background: isSelected ? '#e8f4ff' : '#fff',
                                cursor: 'pointer',
                                textAlign: 'left',
                                transition: 'all 0.2s',
                                outline: 'none',
                            }}
                            onMouseEnter={(e) => {
                                if (!isSelected) e.currentTarget.style.background = '#f5f5f5';
                            }}
                            onMouseLeave={(e) => {
                                if (!isSelected) e.currentTarget.style.background = '#fff';
                            }}
                        >
                            <div style={{
                                fontWeight: isSelected ? '600' : '500',
                                color: isSelected ? '#4a9eff' : '#333',
                                marginBottom: '2px',
                            }}>
                                {matInfo.name}
                            </div>
                            <div style={{
                                fontSize: '11px',
                                color: '#666',
                                lineHeight: '1.3',
                            }}>
                                {matInfo.description}
                            </div>
                        </button>
                    );
                })}
            </div>
        </div>
    );
};
