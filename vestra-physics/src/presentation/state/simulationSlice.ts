// src/presentation/state/simulationSlice.ts

import type { StateCreator } from 'zustand';
import type { SimulationStore } from './types';
import { MAX_PHYSICS_STEP } from '../../core/constants/SimulationConstants';

/**
 * Slice responsible for the physics loop control.
 * Handles the Start/Stop state and the per-frame physics stepping.
 */
export const createSimulationSlice: StateCreator<SimulationStore, [], [], Partial<SimulationStore>> = (set, get) => ({
    isRunning: false,
    fps: 0,

    toggleSimulation: () => {
        set((state) => ({ isRunning: !state.isRunning }));
    },

    setFps: (fps: number) => {
        set({ fps });
    },

    step: (dt: number) => {
        const { engine, isRunning } = get();

        // Safety Cap: Prevent simulation explosion if the tab was inactive
        // or if the frame rate drops significantly.
        const safeDt = Math.min(dt, MAX_PHYSICS_STEP);

        if (engine && isRunning) {
            engine.step(safeDt);
        }
    },
});