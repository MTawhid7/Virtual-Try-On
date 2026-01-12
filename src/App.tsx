// src/App.tsx
import { Canvas } from '@react-three/fiber';
import { Environment, OrbitControls } from '@react-three/drei';
import { GarmentV4 } from './v4/components/GarmentV4';

export const App = () => {
  return (
    <div style={{ width: '100vw', height: '100vh', background: '#1a1a1a' }}>
      <Canvas shadows camera={{ position: [0, 0, 5], fov: 45 }}>
        <OrbitControls />
        <ambientLight intensity={0.5} />
        <directionalLight position={[5, 5, 5]} intensity={1} />
        <Environment preset="city" />
        <gridHelper args={[10, 10]} />

        {/* The V4 Verification Component */}
        <GarmentV4 />

      </Canvas>
    </div>
  );
};

export default App;