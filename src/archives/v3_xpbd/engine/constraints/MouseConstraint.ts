// src/v3/engine/constraints/MouseConstraint.ts
import { PhysicsData } from '../core/PhysicsData';
import { PHYSICS_CONSTANTS } from '../../shared/constants';

export class MouseConstraint {
    public solve(data: PhysicsData, dt: number) {
        if (!data.interaction.active) return;

        const { particleIndex, target } = data.interaction;
        const idx = particleIndex * 3;
        const w = data.invMass[particleIndex];

        if (w === 0) return;

        const x = data.positions[idx];
        const y = data.positions[idx + 1];
        const z = data.positions[idx + 2];

        const dx = target.x - x;
        const dy = target.y - y;
        const dz = target.z - z;

        const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < 0.000001) return;

        // Calculate Alpha
        const stiffness = PHYSICS_CONSTANTS.interaction.stiffness;
        let alpha = 0;

        if (stiffness > 0) {
            alpha = stiffness / (dt * dt);
        } else {
            alpha = 0; // Hard constraint
        }

        // XPBD Correction
        const correction = dist / (w + alpha);
        const scale = correction / dist;

        data.positions[idx] += dx * scale * w;
        data.positions[idx + 1] += dy * scale * w;
        data.positions[idx + 2] += dz * scale * w;
    }
}