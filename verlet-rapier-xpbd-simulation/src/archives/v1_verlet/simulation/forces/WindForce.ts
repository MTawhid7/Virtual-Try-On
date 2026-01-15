import { Vector3, BufferAttribute, Mesh } from 'three';
import type { Particle } from '../types';

export class WindForce {
    private time = 0;
    private windVector = new Vector3();
    private tmpForce = new Vector3();
    private normal = new Vector3();

    update(dt: number) {
        this.time += dt;
        const windStrength = Math.cos(this.time / 2.0) * 5 + 10;
        this.windVector.set(
            Math.sin(this.time),
            Math.cos(this.time * 0.8),
            Math.sin(this.time * 0.5)
        );
        this.windVector.normalize().multiplyScalar(windStrength * 0.05);
    }

    apply(particles: Particle[], mesh?: Mesh) {
        if (!mesh) return;

        const normals = mesh.geometry.attributes.normal;
        for (let i = 0; i < particles.length; i++) {
            this.normal.fromBufferAttribute(normals as BufferAttribute, i);
            this.tmpForce.copy(this.normal).normalize().multiplyScalar(this.normal.dot(this.windVector));
            particles[i].acceleration.add(this.tmpForce.multiplyScalar(2.0));
        }
    }
}