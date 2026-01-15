import type { Particle, CollisionSphere } from '../types';

export class CollisionSolver {
    static solve(particles: Particle[], spheres: CollisionSphere[]) {
        for (const sphere of spheres) {
            const radiusSq = sphere.radius * sphere.radius;

            for (const p of particles) {
                const dx = p.position.x - sphere.position.x;
                const dy = p.position.y - sphere.position.y;
                const dz = p.position.z - sphere.position.z;
                const distSq = dx * dx + dy * dy + dz * dz;

                if (distSq < radiusSq) {
                    const dist = Math.sqrt(distSq);
                    const correction = (sphere.radius - dist) / dist; // Normalize and scale

                    p.position.x += dx * correction;
                    p.position.y += dy * correction;
                    p.position.z += dz * correction;
                }
            }
        }
    }
}