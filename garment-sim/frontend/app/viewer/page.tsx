"use client";

import dynamic from "next/dynamic";
import { useState, useEffect, useCallback } from "react";
import ViewerControls from "@/components/ViewerControls";

const GarmentViewer = dynamic(() => import("@/components/GarmentViewer"), {
  ssr: false,
  loading: () => (
    <div className="flex h-full w-full items-center justify-center bg-neutral-900">
      <div className="text-neutral-400 text-sm">Loading viewer…</div>
    </div>
  ),
});

export type BgMode = "dark" | "light";

export default function ViewerPage() {
  const [models, setModels] = useState<string[]>([]);
  const [selectedModel, setSelectedModel] = useState("garment_drape.glb");
  const [bgMode, setBgMode] = useState<BgMode>("light");
  const [wireframe, setWireframe] = useState(false);
  const [clothColor, setClothColor] = useState("#4ac8ea");

  // Animation state
  const [hasAnimation, setHasAnimation] = useState(false);
  const [isPlaying, setIsPlaying] = useState(false);
  const [playbackSpeed, setPlaybackSpeed] = useState(0.5);
  const [currentFrame, setCurrentFrame] = useState(0);
  const [totalFrames, setTotalFrames] = useState(0);
  const [seekFrame, setSeekFrame] = useState<number | null>(null);

  useEffect(() => {
    fetch("/api/models")
      .then((r) => r.json())
      .then(({ models }: { models: string[] }) => {
        if (models.length > 0) {
          setModels(models);
          if (!models.includes(selectedModel)) setSelectedModel(models[0]);
        }
      })
      .catch(() => {});
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // Reset animation state when model changes
  useEffect(() => {
    setIsPlaying(false);
    setCurrentFrame(0);
    setTotalFrames(0);
    setHasAnimation(false);
    setSeekFrame(null);
  }, [selectedModel]);

  const handleAnimationUpdate = useCallback(
    (frame: number, total: number, hasAnim: boolean) => {
      setCurrentFrame(frame);
      setTotalFrames(total);
      setHasAnimation(hasAnim);
    },
    []
  );

  const handleSeek = useCallback((frame: number) => {
    setSeekFrame(frame);
    setCurrentFrame(frame);
    setIsPlaying(false); // pause while scrubbing
  }, []);

  const handlePlayPause = useCallback(() => {
    setIsPlaying((p) => !p);
    setSeekFrame(null); // clear any pending seek when resuming
  }, []);

  const modelUrl = `/models/${selectedModel}`;

  return (
    <div className="relative h-full w-full">
      <GarmentViewer
        modelUrl={modelUrl}
        bgMode={bgMode}
        wireframe={wireframe}
        clothColor={clothColor}
        isPlaying={isPlaying}
        playbackSpeed={playbackSpeed}
        seekFrame={seekFrame}
        onAnimationUpdate={handleAnimationUpdate}
      />
      <ViewerControls
        models={models}
        selectedModel={selectedModel}
        onModelChange={setSelectedModel}
        bgMode={bgMode}
        onBgModeChange={setBgMode}
        wireframe={wireframe}
        onWireframeChange={setWireframe}
        clothColor={clothColor}
        onClothColorChange={setClothColor}
        hasAnimation={hasAnimation}
        isPlaying={isPlaying}
        onPlayPause={handlePlayPause}
        currentFrame={currentFrame}
        totalFrames={totalFrames}
        playbackSpeed={playbackSpeed}
        onSpeedChange={setPlaybackSpeed}
        onSeek={handleSeek}
      />
    </div>
  );
}
