import * as THREE from 'three';
import { VerletPhysicsEngine } from '../simulation/VerletPhysicsEngine';

export class BodyManager {
    private scene: THREE.Scene;
    private physics: VerletPhysicsEngine;
    private group: THREE.Group;

    constructor(scene: THREE.Scene, physics: VerletPhysicsEngine) {
        this.scene = scene;
        this.physics = physics;
        this.group = new THREE.Group();
        this.scene.add(this.group);
    }

    createMannequin() {
        // 1. Head
        this.addBodyPart(
            new THREE.Vector3(0, 1.7, 0), // Position
            0.15,                         // Radius
            0xcccccc                      // Color
        );

        // 2. Neck (Visual only, usually too thin for collision)
        const neckGeo = new THREE.CylinderGeometry(0.08, 0.08, 0.2);
        const neckMat = new THREE.MeshStandardMaterial({ color: 0xcccccc });
        const neck = new THREE.Mesh(neckGeo, neckMat);
        neck.position.set(0, 1.55, 0);
        neck.castShadow = true;
        this.group.add(neck);

        // 3. Chest / Torso
        this.addBodyPart(
            new THREE.Vector3(0, 1.35, 0),
            0.25,
            0x888888
        );

        // 4. Left Shoulder
        this.addBodyPart(
            new THREE.Vector3(-0.25, 1.45, 0),
            0.12,
            0xcccccc
        );

        // 5. Right Shoulder
        this.addBodyPart(
            new THREE.Vector3(0.25, 1.45, 0),
            0.12,
            0xcccccc
        );
    }

    private addBodyPart(position: THREE.Vector3, radius: number, color: number) {
        // A. Visual Mesh
        const geometry = new THREE.SphereGeometry(radius, 32, 32);
        const material = new THREE.MeshStandardMaterial({
            color: color,
            roughness: 0.5,
            metalness: 0.1
        });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.copy(position);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        this.group.add(mesh);

        // B. Physics Collider
        // We must convert the world position to the cloth's local space if the cloth moves.
        // But for now, assuming cloth is at (0,0,0) world, this works fine.
        this.physics.addCollisionSphere(position, radius + 0.02); // Add slightly padding
    }

    dispose() {
        this.scene.remove(this.group);
        // Traverse and dispose geometries/materials...
    }
}