"use client";

import { Suspense, useEffect, useMemo, useRef } from "react";
import { Canvas, useThree, useFrame } from "@react-three/fiber";
import {
  OrbitControls,
  Environment,
  useGLTF,
} from "@react-three/drei";
import * as THREE from "three";
import type { BgMode } from "@/app/viewer/page";

// Playback FPS must match the fps arg passed to write_glb_animated().
const ANIMATION_FPS = 6;

// ---------------------------------------------------------------------------
// Scene background controller
// ---------------------------------------------------------------------------

function SceneBackground({ bgMode }: { bgMode: BgMode }) {
  const { scene } = useThree();
  useEffect(() => {
    scene.background = new THREE.Color(
      bgMode === "light" ? "#f2f2f2" : "#1a1a1a"
    );
  }, [scene, bgMode]);
  return null;
}

// ---------------------------------------------------------------------------
// GLB model loader + animation player
// ---------------------------------------------------------------------------

interface ModelProps {
  url: string;
  wireframe: boolean;
  clothColor: string;
  isPlaying: boolean;
  playbackSpeed: number;
  seekFrame: number | null;
  onAnimationUpdate: (frame: number, total: number, hasAnim: boolean) => void;
}

function Model({
  url,
  wireframe,
  clothColor,
  isPlaying,
  playbackSpeed,
  seekFrame,
  onAnimationUpdate,
}: ModelProps) {
  const { scene, animations } = useGLTF(url);
  const hasAnimation = animations.length > 0;

  const clothColorObj = useMemo(
    () => new THREE.Color(clothColor),
    [clothColor]
  );

  const bodyMaterial = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        roughness: 0.65,
        metalness: 0.0,
        color: new THREE.Color("#c8956c"),
        side: THREE.FrontSide,
      }),
    []
  );

  const cloned = useMemo(() => {
    const c = scene.clone(true);
    c.traverse((node) => {
      if (!(node instanceof THREE.Mesh)) return;
      const name = node.name.toLowerCase();
      if (name.includes("body")) {
        node.material = bodyMaterial;
      } else {
        node.material = new THREE.MeshStandardMaterial({
          roughness: 0.82,
          metalness: 0.0,
          color: clothColorObj,
          side: THREE.DoubleSide,
          wireframe,
        });
      }
    });
    return c;
  }, [scene, clothColorObj, wireframe, bodyMaterial]);

  // AnimationMixer — recreated whenever the cloned scene changes (new model load).
  const mixerRef = useRef<THREE.AnimationMixer | null>(null);
  const clipDurationRef = useRef(0);

  useEffect(() => {
    if (!hasAnimation) {
      mixerRef.current = null;
      return;
    }
    const mixer = new THREE.AnimationMixer(cloned);
    const clip = animations[0];
    clipDurationRef.current = clip.duration;
    // Start the action so morph influences are bound; pause immediately.
    const action = mixer.clipAction(clip);
    action.play();
    mixer.setTime(0);
    mixerRef.current = mixer;

    return () => {
      mixer.stopAllAction();
      mixerRef.current = null;
    };
  }, [cloned, animations, hasAnimation]);

  // Track last-reported frame to avoid flooding React with state updates.
  const prevFrameRef = useRef(-1);

  useFrame((_, delta) => {
    const mixer = mixerRef.current;
    const dur = clipDurationRef.current;

    if (!hasAnimation) {
      if (prevFrameRef.current !== -2) {
        prevFrameRef.current = -2;
        onAnimationUpdate(0, 0, false);
      }
      return;
    }

    if (!mixer) return;

    if (isPlaying) {
      mixer.update(delta * playbackSpeed);
    } else if (seekFrame !== null) {
      // User is scrubbing — seek without advancing.
      const t = dur > 0 ? (seekFrame / Math.max(Math.floor(dur * ANIMATION_FPS), 1)) * dur : 0;
      mixer.setTime(Math.min(t, dur));
    }

    // Report current frame (throttled — only when frame index changes).
    const currentTime = dur > 0 ? mixer.time % dur : 0;
    const frame = Math.floor(currentTime * ANIMATION_FPS);
    const total = Math.floor(dur * ANIMATION_FPS);
    if (frame !== prevFrameRef.current) {
      prevFrameRef.current = frame;
      onAnimationUpdate(frame, total, true);
    }
  });

  return <primitive object={cloned} />;
}

// ---------------------------------------------------------------------------
// Studio lighting rig
// ---------------------------------------------------------------------------

function StudioLighting() {
  return (
    <>
      {/* HDR environment for reflections + ambient fill */}
      <Environment preset="studio" />

      {/* Key light — upper-right-front */}
      <directionalLight position={[2.5, 4, 3]} intensity={2.2} />

      {/* Fill light — upper-left */}
      <directionalLight position={[-2, 3, 1]} intensity={0.9} />

      {/* Rim light — back */}
      <directionalLight position={[0.5, 2, -3]} intensity={0.6} />

      {/* Soft ambient floor bounce */}
      <ambientLight intensity={0.3} />
    </>
  );
}

// ---------------------------------------------------------------------------
// Loading fallback rendered inside Canvas
// ---------------------------------------------------------------------------

function LoadingFallback() {
  return null;
}

// ---------------------------------------------------------------------------
// Main export
// ---------------------------------------------------------------------------

interface GarmentViewerProps {
  modelUrl: string;
  bgMode: BgMode;
  wireframe: boolean;
  clothColor: string;
  isPlaying: boolean;
  playbackSpeed: number;
  seekFrame: number | null;
  onAnimationUpdate: (frame: number, total: number, hasAnim: boolean) => void;
}

export default function GarmentViewer({
  modelUrl,
  bgMode,
  wireframe,
  clothColor,
  isPlaying,
  playbackSpeed,
  seekFrame,
  onAnimationUpdate,
}: GarmentViewerProps) {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const controlsRef = useRef<any>(null);

  // Re-center orbit target when model changes
  useEffect(() => {
    if (controlsRef.current) {
      controlsRef.current.target.set(0, 0.875, 0);
      controlsRef.current.update();
    }
  }, [modelUrl]);

  return (
    <Canvas
      camera={{ position: [0, 1.1, 2.8], fov: 40, near: 0.01, far: 50 }}
      gl={{
        antialias: true,
        toneMapping: THREE.ACESFilmicToneMapping,
        toneMappingExposure: 1.1,
        outputColorSpace: THREE.SRGBColorSpace,
      }}
      style={{ width: "100%", height: "100%" }}
    >
      <SceneBackground bgMode={bgMode} />

      <StudioLighting />

      <OrbitControls
        ref={controlsRef}
        target={[0, 0.875, 0]}
        enableDamping
        dampingFactor={0.08}
        minDistance={0.4}
        maxDistance={6}
        maxPolarAngle={Math.PI / 2 + 0.1}
        makeDefault
      />

      <Suspense fallback={<LoadingFallback />}>
        <Model
          url={modelUrl}
          wireframe={wireframe}
          clothColor={clothColor}
          isPlaying={isPlaying}
          playbackSpeed={playbackSpeed}
          seekFrame={seekFrame}
          onAnimationUpdate={onAnimationUpdate}
        />
      </Suspense>
    </Canvas>
  );
}
