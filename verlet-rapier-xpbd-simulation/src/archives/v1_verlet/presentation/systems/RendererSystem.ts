import * as THREE from 'three';
import type { AppConfig, Disposable } from '../../types';

export class RendererSystem implements Disposable {
    public renderer: THREE.WebGLRenderer;
    public camera: THREE.PerspectiveCamera;
    public scene: THREE.Scene;

    constructor(config: AppConfig) {
        // 1. Setup Scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0xf0f0f0);

        // 2. Setup Camera
        this.camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 100);
        this.camera.position.set(0, 0, 2);

        // 3. Setup Renderer
        this.renderer = new THREE.WebGLRenderer({
            antialias: config.rendering.antialias,
            powerPreference: 'high-performance'
        });
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.outputColorSpace = THREE.SRGBColorSpace;
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, config.rendering.pixelRatioMax));
    }

    mount(container: HTMLElement) {
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        container.appendChild(this.renderer.domElement);
        window.addEventListener('resize', this.onResize);
    }

    private onResize = () => {
        const canvas = this.renderer.domElement;
        const parent = canvas.parentElement;
        if (parent) {
            const width = parent.clientWidth;
            const height = parent.clientHeight;
            this.camera.aspect = width / height;
            this.camera.updateProjectionMatrix();
            this.renderer.setSize(width, height);
        }
    };

    dispose() {
        window.removeEventListener('resize', this.onResize);
        this.renderer.dispose();
        this.scene.traverse((obj) => {
            if (obj instanceof THREE.Mesh) {
                obj.geometry.dispose();
                if (obj.material instanceof THREE.Material) obj.material.dispose();
            }
        });
    }
}