// src/v4/components/SDFVisualizer.tsx
import { useEffect, useState } from 'react';
import { Bridge } from '../adapter/Bridge';

export const SDFVisualizer = ({ bridge }: { bridge: Bridge }) => {
    const [config, setConfig] = useState<number[] | null>(null);

    useEffect(() => {
        const interval = setInterval(() => {
            const params = bridge.getSDFConfig();
            if (params && params.length > 0) {
                setConfig(params);
                clearInterval(interval);
            }
        }, 100);
        return () => clearInterval(interval);
    }, [bridge]);

    if (!config) return null;

    // Unpack
    const spineR = config[0];
    const spineBase = [config[1], config[2], config[3]];
    const spineTop = [config[4], config[5], config[6]];

    const shoulderR = config[7];
    const shoulderL = [config[8], config[9], config[10]];
    const shoulderR_Pos = [config[11], config[12], config[13]];

    // Helper to calc mid and height for vertical capsule
    const sHeight = spineTop[1] - spineBase[1];
    const sMid = spineBase[1] + sHeight / 2;

    // Helper for horizontal capsule
    const hLen = shoulderR_Pos[0] - shoulderL[0];
    const hMidX = shoulderL[0] + hLen / 2;

    return (
        <group>
            {/* Spine */}
            <mesh position={[0, sMid, 0]}>
                <capsuleGeometry args={[spineR, sHeight, 4, 8]} />
                <meshBasicMaterial color="red" wireframe transparent opacity={0.3} />
            </mesh>

            {/* Shoulders (Rotated 90 deg Z) */}
            <mesh position={[hMidX, shoulderL[1], 0]} rotation={[0, 0, Math.PI / 2]}>
                <capsuleGeometry args={[shoulderR, hLen, 4, 8]} />
                <meshBasicMaterial color="orange" wireframe transparent opacity={0.3} />
            </mesh>

            <gridHelper args={[10, 10, 0xff0000, 0x444444]} position={[0, 0.01, 0]} />
        </group>
    );
};