import { Mesh, Vector3 } from 'three';
import type { IPhysicsEngine, PhysicsConfig } from './IPhysicsEngine';
import { PhysicsState } from './PhysicsState';
import { WindForce } from './forces/WindForce';
import { BVHCollider } from './solvers/BVHCollider';
import { ConstraintSolver } from './solvers/ConstraintSolver';

export class VerletPhysicsEngine implements IPhysicsEngine {
    private state = new PhysicsState();
    private wind = new WindForce();
    private collider = new BVHCollider();

    setColliderMesh(mesh: Mesh) {
        this.collider.setCollider(mesh);
    }

    async initialize(mesh: Mesh, config: PhysicsConfig): Promise<void> {
        this.state.config = config;
        this.state.reset();

        const positions = mesh.geometry.attributes.position.array;
        for (let i = 0; i < positions.length; i += 3) {
            this.state.addParticle(positions[i], positions[i + 1], positions[i + 2], 0.1);
        }

        // Adjacency Logic (Simplified for brevity, assuming indexed geometry)
        if (mesh.geometry.index) {
            const index = mesh.geometry.index;
            const edges = new Set<string>();
            const addEdge = (a: number, b: number) => {
                const key = a < b ? `${a}_${b}` : `${b}_${a}`;
                if (!edges.has(key)) {
                    edges.add(key);
                    const p1 = this.state.particles[a];
                    const p2 = this.state.particles[b];
                    this.state.addConstraint(a, b, p1.position.distanceTo(p2.position));
                }
            };
            for (let i = 0; i < index.count; i += 3) {
                addEdge(index.getX(i), index.getX(i + 1));
                addEdge(index.getX(i + 1), index.getX(i + 2));
                addEdge(index.getX(i + 2), index.getX(i));
            }
        }
    }

    step(_deltaTime: number, mesh?: Mesh): void {
        const dt = 18 / 1000;
        const dtSq = dt * dt;

        this.wind.update(dt);
        this.wind.apply(this.state.particles, mesh);

        const gravity = new Vector3(0, -9.81, 0).multiplyScalar(0.1);
        const drag = 0.97;

        for (const p of this.state.particles) {
            if (p.pinned) continue;
            p.acceleration.add(gravity);

            const velocity = p.position.clone().sub(p.prevPosition).multiplyScalar(drag);
            const nextPos = p.position.clone().add(velocity).add(p.acceleration.multiplyScalar(dtSq));

            p.prevPosition.copy(p.position);
            p.position.copy(nextPos);
            p.acceleration.set(0, 0, 0);
        }

        for (let i = 0; i < this.state.config.iterations; i++) {
            ConstraintSolver.solve(this.state.particles, this.state.constraints);
            this.collider.solve(this.state.particles);
        }
    }

    syncToMesh(mesh: Mesh): void {
        const positions = mesh.geometry.attributes.position.array as Float32Array;
        for (let i = 0; i < this.state.particles.length; i++) {
            const p = this.state.particles[i];
            positions[i * 3] = p.position.x;
            positions[i * 3 + 1] = p.position.y;
            positions[i * 3 + 2] = p.position.z;
        }
        mesh.geometry.attributes.position.needsUpdate = true;
        mesh.geometry.computeVertexNormals();
    }

    // Pass-through methods
    pinIndices(indices: number[]) {
        indices.forEach(i => { if (this.state.particles[i]) this.state.particles[i].pinned = true; });
    }

    pinParticle(index: number, pos: Vector3) {
        if (this.state.particles[index]) {
            this.state.particles[index].pinned = true;
            this.state.particles[index].position.copy(pos);
        }
    }

    releaseParticle(index: number) {
        if (this.state.particles[index]) this.state.particles[index].pinned = false;
    }

    applyForce(_i: number, _f: Vector3) { }
    dispose() { this.state.reset(); }

    // Legacy support for spheres if needed (empty now as we use BVH)
    addCollisionSphere(_p: Vector3, _r: number) { }
    updateSpherePosition(_i: number, _p: Vector3) { }
    get collisionSpheres(): { position: Vector3, radius: number }[] { return []; }

    public getParticles() {
        return this.state.particles;
      }
}