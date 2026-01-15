import type { Particle, Constraint } from '../types';

export class ConstraintSolver {
    static solve(particles: Particle[], constraints: Constraint[]) {
        const diff = { x: 0, y: 0, z: 0 }; // Avoid creating new Vector3 per loop

        for (let i = 0; i < constraints.length; i++) {
            const c = constraints[i];
            const p1 = particles[c.p1];
            const p2 = particles[c.p2];

            diff.x = p2.position.x - p1.position.x;
            diff.y = p2.position.y - p1.position.y;
            diff.z = p2.position.z - p1.position.z;

            const currentDist = Math.sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);

            if (currentDist === 0) continue;

            const correction = (1 - c.restDistance / currentDist) * 0.5;
            const offX = diff.x * correction;
            const offY = diff.y * correction;
            const offZ = diff.z * correction;

            if (!p1.pinned) {
                p1.position.x += offX;
                p1.position.y += offY;
                p1.position.z += offZ;
            }
            if (!p2.pinned) {
                p2.position.x -= offX;
                p2.position.y -= offY;
                p2.position.z -= offZ;
            }
        }
    }
}