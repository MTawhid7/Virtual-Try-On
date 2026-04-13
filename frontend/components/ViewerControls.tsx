"use client";

import type { BgMode } from "@/app/viewer/page";

const CLOTH_PRESETS = [
  { label: "Sky", color: "#4ac8ea" },
  { label: "White", color: "#f5f5f5" },
  { label: "Charcoal", color: "#2d2d2d" },
  { label: "Red", color: "#d94f4f" },
  { label: "Sage", color: "#7aab7a" },
  { label: "Navy", color: "#2b4080" },
];

// Fraction of total keyframes that belong to the sew phase.
// Matches sew_frames/total_frames = 240/390 in garment_drape.py.
const SEW_PHASE_FRACTION = 240 / 390;

const SPEED_OPTIONS = [
  { label: "0.25×", value: 0.25 },
  { label: "0.5×",  value: 0.5  },
  { label: "1×",    value: 1.0  },
  { label: "2×",    value: 2.0  },
];

interface Props {
  models: string[];
  selectedModel: string;
  onModelChange: (m: string) => void;
  bgMode: BgMode;
  onBgModeChange: (m: BgMode) => void;
  wireframe: boolean;
  onWireframeChange: (v: boolean) => void;
  clothColor: string;
  onClothColorChange: (c: string) => void;
  // Animation
  hasAnimation: boolean;
  isPlaying: boolean;
  onPlayPause: () => void;
  currentFrame: number;
  totalFrames: number;
  playbackSpeed: number;
  onSpeedChange: (s: number) => void;
  onSeek: (frame: number) => void;
}

export default function ViewerControls({
  models,
  selectedModel,
  onModelChange,
  bgMode,
  onBgModeChange,
  wireframe,
  onWireframeChange,
  clothColor,
  onClothColorChange,
  hasAnimation,
  isPlaying,
  onPlayPause,
  currentFrame,
  totalFrames,
  playbackSpeed,
  onSpeedChange,
  onSeek,
}: Props) {
  return (
    <div className="absolute top-4 right-4 z-10 flex flex-col gap-3 w-56 rounded-xl bg-black/60 backdrop-blur-md p-4 text-white text-sm shadow-2xl border border-white/10">
      <p className="font-semibold tracking-wide text-xs uppercase text-neutral-400">
        Viewer Controls
      </p>

      {/* Model selector */}
      {models.length > 0 && (
        <div className="flex flex-col gap-1">
          <label className="text-xs text-neutral-400">Model</label>
          <select
            value={selectedModel}
            onChange={(e) => onModelChange(e.target.value)}
            className="w-full rounded-md bg-white/10 border border-white/20 px-2 py-1.5 text-xs text-white focus:outline-none focus:ring-1 focus:ring-white/40"
          >
            {models.map((m) => (
              <option key={m} value={m} className="bg-neutral-800">
                {m.replace(".glb", "")}
              </option>
            ))}
          </select>
        </div>
      )}

      {/* Background */}
      <div className="flex flex-col gap-1">
        <label className="text-xs text-neutral-400">Background</label>
        <div className="flex gap-2">
          <button
            onClick={() => onBgModeChange("light")}
            className={`flex-1 rounded-md py-1.5 text-xs font-medium transition-colors ${
              bgMode === "light"
                ? "bg-white text-black"
                : "bg-white/10 text-neutral-300 hover:bg-white/20"
            }`}
          >
            Light
          </button>
          <button
            onClick={() => onBgModeChange("dark")}
            className={`flex-1 rounded-md py-1.5 text-xs font-medium transition-colors ${
              bgMode === "dark"
                ? "bg-neutral-600 text-white"
                : "bg-white/10 text-neutral-300 hover:bg-white/20"
            }`}
          >
            Dark
          </button>
        </div>
      </div>

      {/* Cloth color */}
      <div className="flex flex-col gap-1.5">
        <label className="text-xs text-neutral-400">Cloth Color</label>
        <div className="flex flex-wrap gap-1.5">
          {CLOTH_PRESETS.map((p) => (
            <button
              key={p.color}
              title={p.label}
              onClick={() => onClothColorChange(p.color)}
              className="w-7 h-7 rounded-full border-2 transition-transform hover:scale-110"
              style={{
                backgroundColor: p.color,
                borderColor:
                  clothColor === p.color
                    ? "white"
                    : "rgba(255,255,255,0.2)",
              }}
            />
          ))}
          {/* Custom color input */}
          <label
            className="w-7 h-7 rounded-full border-2 border-white/20 flex items-center justify-center cursor-pointer overflow-hidden hover:scale-110 transition-transform"
            title="Custom color"
          >
            <span className="text-[9px] text-neutral-400">+</span>
            <input
              type="color"
              value={clothColor}
              onChange={(e) => onClothColorChange(e.target.value)}
              className="sr-only"
            />
          </label>
        </div>
      </div>

      {/* Wireframe */}
      <div className="flex items-center justify-between">
        <label className="text-xs text-neutral-400">Wireframe</label>
        <button
          onClick={() => onWireframeChange(!wireframe)}
          className={`relative w-9 h-5 rounded-full transition-colors ${
            wireframe ? "bg-sky-500" : "bg-white/20"
          }`}
        >
          <span
            className={`absolute top-0.5 left-0.5 w-4 h-4 rounded-full bg-white transition-transform ${
              wireframe ? "translate-x-4" : "translate-x-0"
            }`}
          />
        </button>
      </div>

      {/* Animation controls — only shown for animated GLBs */}
      {hasAnimation && (
        <div className="border-t border-white/10 pt-3 flex flex-col gap-2">
          <div className="flex items-center justify-between">
            {/* Phase badge */}
            <span className={`text-[10px] font-semibold px-1.5 py-0.5 rounded ${
              totalFrames > 0 && currentFrame / totalFrames < SEW_PHASE_FRACTION
                ? "bg-amber-500/30 text-amber-300"
                : "bg-sky-500/30 text-sky-300"
            }`}>
              {totalFrames > 0 && currentFrame / totalFrames < SEW_PHASE_FRACTION
                ? "SEW"
                : "DRAPE"}
            </span>

            {/* Play / Pause */}
            <button
              onClick={onPlayPause}
              className="flex items-center justify-center w-8 h-8 rounded-full bg-white/10 hover:bg-white/20 transition-colors"
              title={isPlaying ? "Pause" : "Play"}
            >
              {isPlaying ? (
                /* Pause icon */
                <svg width="12" height="12" viewBox="0 0 12 12" fill="currentColor">
                  <rect x="1" y="1" width="3.5" height="10" rx="1"/>
                  <rect x="7.5" y="1" width="3.5" height="10" rx="1"/>
                </svg>
              ) : (
                /* Play icon */
                <svg width="12" height="12" viewBox="0 0 12 12" fill="currentColor">
                  <path d="M2 1.5 L10 6 L2 10.5 Z"/>
                </svg>
              )}
            </button>

            {/* Frame counter */}
            <span className="text-[10px] text-neutral-400 tabular-nums">
              {currentFrame}/{totalFrames}
            </span>
          </div>

          {/* Timeline scrubber */}
          <input
            type="range"
            min={0}
            max={totalFrames}
            value={currentFrame}
            onChange={(e) => onSeek(Number(e.target.value))}
            className="w-full h-1.5 accent-sky-400 cursor-pointer"
          />

          {/* Speed selector */}
          <div className="flex gap-1">
            {SPEED_OPTIONS.map((s) => (
              <button
                key={s.value}
                onClick={() => onSpeedChange(s.value)}
                className={`flex-1 rounded py-1 text-[10px] font-medium transition-colors ${
                  playbackSpeed === s.value
                    ? "bg-sky-500 text-white"
                    : "bg-white/10 text-neutral-300 hover:bg-white/20"
                }`}
              >
                {s.label}
              </button>
            ))}
          </div>
        </div>
      )}

      {/* Camera hint */}
      <div className="border-t border-white/10 pt-2 text-[10px] text-neutral-500 leading-relaxed">
        Left-drag: orbit · Scroll: zoom · Right-drag: pan
      </div>
    </div>
  );
}
