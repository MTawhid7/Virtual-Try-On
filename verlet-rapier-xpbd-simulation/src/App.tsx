// src/App.tsx
import { Canvas } from '@react-three/fiber';
import { Environment, OrbitControls } from '@react-three/drei';
import { GarmentV4 } from './v4/components/GarmentV4';

export const App = () => {
  return (
    <div style={{ width: '100vw', height: '100vh', background: '#1a1a1a' }}>
      {/* 1. Move Camera up to Y=1.5 (Chest height) */}
      <Canvas shadows camera={{ position: [0, 1.5, 2.5], fov: 45 }}>
        <OrbitControls target={[0, 1.4, 0]} makeDefault />

        {/* Stronger Main Light */}
        <ambientLight intensity={0.4} />
        <directionalLight
          position={[5, 5, 5]}
          intensity={1.5}
          castShadow
          shadow-mapSize={[1024, 1024]}
        />

        {/* Backlight (Rim Light) to show silhouette */}
        <spotLight position={[-5, 5, -5]} intensity={1.0} color="#ffccaa" />

        {/* Studio Environment for reflections */}
        <Environment preset="city" />

        <GarmentV4 />
      </Canvas>
    </div>
  );
};

export default App;