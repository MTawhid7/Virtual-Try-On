import * as THREE from 'three';
import type { IPhysicsEngine } from '../simulation/IPhysicsEngine';
import type { Disposable } from '../types';

export class InteractionController implements Disposable {
    private raycaster = new THREE.Raycaster();
    private mouse = new THREE.Vector2();
    private plane = new THREE.Plane();
    private planeIntersect = new THREE.Vector3();

    private isDragging = false;
    private draggedParticleIndex: number | null = null;
    public enabled = true;

    private canvas: HTMLElement;
    private camera: THREE.Camera;
    private physics: IPhysicsEngine;
    private mesh: THREE.Mesh;

    constructor(
        canvas: HTMLElement,
        camera: THREE.Camera,
        mesh: THREE.Mesh,
        physics: IPhysicsEngine
    ) {
        this.canvas = canvas;
        this.camera = camera;
        this.mesh = mesh;
        this.physics = physics;

        this.canvas.addEventListener('mousedown', this.onMouseDown);
        this.canvas.addEventListener('mousemove', this.onMouseMove);
        window.addEventListener('mouseup', this.onMouseUp);
        this.canvas.addEventListener('touchstart', this.onTouchStart, { passive: false });
        this.canvas.addEventListener('touchmove', this.onTouchMove, { passive: false });
        window.addEventListener('touchend', this.onMouseUp);
    }

    public setEnabled(isEnabled: boolean) {
        this.enabled = isEnabled;
        if (!isEnabled) {
            this.isDragging = false;
            this.draggedParticleIndex = null;
        }
    }

    private updateMouse(clientX: number, clientY: number) {
        const rect = this.canvas.getBoundingClientRect();
        this.mouse.x = ((clientX - rect.left) / rect.width) * 2 - 1;
        this.mouse.y = -((clientY - rect.top) / rect.height) * 2 + 1;
    }

    private findNearestParticle(point: THREE.Vector3): number | null {
        const positions = this.mesh.geometry.attributes.position.array;
        let minDist = Infinity;
        let index = -1;

        for (let i = 0; i < positions.length; i += 3) {
            const dx = point.x - positions[i];
            const dy = point.y - positions[i + 1];
            const dz = point.z - positions[i + 2];
            const distSq = dx * dx + dy * dy + dz * dz;

            if (distSq < minDist) {
                minDist = distSq;
                index = i / 3;
            }
        }
        // Threshold 0.2 units
        return minDist < 0.2 ? index : null;
    }

    private handleStart(x: number, y: number) {
        this.updateMouse(x, y);
        this.raycaster.setFromCamera(this.mouse, this.camera);

        const intersects = this.raycaster.intersectObject(this.mesh);
        if (intersects.length > 0) {
            const hitPoint = intersects[0].point;

            // FIX: Convert World Hit Point -> Local Mesh Space
            // The physics engine works in local space (relative to mesh position)
            const localPoint = this.mesh.worldToLocal(hitPoint.clone());

            this.draggedParticleIndex = this.findNearestParticle(localPoint);

            if (this.draggedParticleIndex !== null) {
                this.isDragging = true;

                // Plane stays in World Space for dragging calculation
                this.plane.setFromNormalAndCoplanarPoint(
                    this.camera.getWorldDirection(new THREE.Vector3()),
                    hitPoint
                );
            }
        }
    }

    private handleMove(x: number, y: number) {
        if (!this.enabled || !this.isDragging || this.draggedParticleIndex === null) return;

        this.updateMouse(x, y);
        this.raycaster.setFromCamera(this.mouse, this.camera);

        if (this.raycaster.ray.intersectPlane(this.plane, this.planeIntersect)) {
            // FIX: Convert the dragged point from World -> Local before sending to physics
            const localTarget = this.mesh.worldToLocal(this.planeIntersect.clone());
            this.physics.pinParticle(this.draggedParticleIndex, localTarget);
        }
    }

    // ... (Rest of file: onMouseDown, onTouchStart, onMouseMove, etc. remain the same)
    private onMouseDown = (e: MouseEvent) => {
        if (!this.enabled || e.button !== 0) return;
        this.handleStart(e.clientX, e.clientY);
    };

    private onTouchStart = (e: TouchEvent) => {
        if (!this.enabled || e.touches.length === 0) return;
        e.preventDefault();
        this.handleStart(e.touches[0].clientX, e.touches[0].clientY);
    };

    private onMouseMove = (e: MouseEvent) => {
        this.handleMove(e.clientX, e.clientY);
    };

    private onTouchMove = (e: TouchEvent) => {
        if (e.touches.length > 0) {
            e.preventDefault();
            this.handleMove(e.touches[0].clientX, e.touches[0].clientY);
        }
    };

    private onMouseUp = () => {
        if (this.isDragging && this.draggedParticleIndex !== null) {
            this.physics.releaseParticle(this.draggedParticleIndex);
        }
        this.isDragging = false;
        this.draggedParticleIndex = null;
    };

    dispose() {
        this.canvas.removeEventListener('mousedown', this.onMouseDown);
        this.canvas.removeEventListener('mousemove', this.onMouseMove);
        window.removeEventListener('mouseup', this.onMouseUp);
        this.canvas.removeEventListener('touchstart', this.onTouchStart);
        this.canvas.removeEventListener('touchmove', this.onTouchMove);
        window.removeEventListener('touchend', this.onMouseUp);
    }
}