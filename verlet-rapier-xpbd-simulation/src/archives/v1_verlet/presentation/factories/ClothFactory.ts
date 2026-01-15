import * as THREE from 'three';

export class ClothFactory {
    static async createCloth(width: number, height: number, segments: number): Promise<THREE.Mesh> {
        const geometry = new THREE.PlaneGeometry(width, height, segments, segments);
        geometry.translate(0, 0.5, 0); // Center it vertically

        const loader = new THREE.TextureLoader();
        // Using a Promise to handle texture loading properly
        const texture = await new Promise<THREE.Texture>((resolve) => {
            loader.load('https://threejs.org/examples/textures/uv_grid_opengl.jpg', resolve);
        });

        texture.wrapS = THREE.RepeatWrapping;
        texture.wrapT = THREE.RepeatWrapping;

        const material = new THREE.MeshStandardMaterial({
            color: 0xffffff,
            map: texture,
            side: THREE.DoubleSide,
            roughness: 0.6,
            metalness: 0.1
        });

        const mesh = new THREE.Mesh(geometry, material);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        return mesh;
    }
}