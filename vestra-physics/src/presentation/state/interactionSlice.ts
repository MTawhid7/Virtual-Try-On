// src/presentation/state/interactionSlice.ts

import type { StateCreator } from 'zustand';
import type { SimulationStore } from './types';

/**
 * Slice responsible for handling user interactions with the cloth (picking and dragging).
 * It bridges the React UI events with the WASM physics engine's interaction system.
 */
export const createInteractionSlice: StateCreator<SimulationStore, [], [], Partial<SimulationStore>> = (set, get) => ({
    isInteracting: false,

    setInteracting: (active) => set({ isInteracting: active }),

    grabParticle: (index, [x, y, z]) => {
        get().engine?.startInteraction(index, x, y, z);
    },

    moveParticle: ([x, y, z]) => {
        get().engine?.updateInteraction(x, y, z);
    },

    releaseParticle: () => {
        get().engine?.endInteraction();
    }
});