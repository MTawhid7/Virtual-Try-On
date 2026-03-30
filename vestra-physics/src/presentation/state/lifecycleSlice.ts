// src/presentation/state/lifecycleSlice.ts

import type { StateCreator } from 'zustand';
import type { SimulationStore } from './types';
import { InitializeSimulation } from '../../application/use-cases/InitializeSimulation';
import { UpdateGarmentSize } from '../../application/use-cases/UpdateGarmentSize';
import { ANCHOR_SIZE } from '../../core/constants/SizingStandards';

/**
 * Slice responsible for the high-level application lifecycle.
 * Manages ASYNC loading of assets, error handling, and shirt sizing operations.
 */
export const createLifecycleSlice: StateCreator<SimulationStore, [], [], Partial<SimulationStore>> = (set, get) => ({
    // Initial State
    engine: null,
    assets: null,
    scaledVertices: null,
    shirtSize: ANCHOR_SIZE,
    isLoading: false,
    isReady: false,
    error: null,
    substeps: 0,
    solverIterations: 0,

    loadAndInitialize: async () => {
        // Prevent double-loading
        if (get().isLoading || get().isReady) return;

        set({ isLoading: true, error: null });

        try {
            const useCase = new InitializeSimulation();

            // Execute Use Case
            const result = await useCase.execute(
                get().assets, // Pass existing assets if any
                get().shirtSize
            );

            set({
                engine: result.engine,
                assets: result.assets,
                scaledVertices: result.scaledVertices,
                isReady: true,
                isLoading: false,
                substeps: result.engine.getSubsteps(),
                solverIterations: result.engine.getSolverIterations()
            });

            console.log("[Store] Simulation Initialized Successfully.");

        } catch (err) {
            console.error(err);
            set({ error: (err as Error).message, isLoading: false });
        }
    },

    setShirtSize: async (size) => {
        const { shirtSize, assets, engine } = get();

        if (shirtSize === size || !assets) return;

        // Pause and set loading state
        set({ shirtSize: size, isReady: false, isRunning: false, isLoading: true });

        // Allow UI to render the loading state before blocking the thread
        setTimeout(async () => {
            try {
                const useCase = new UpdateGarmentSize();

                const result = await useCase.execute(
                    assets,
                    size,
                    engine // Pass old engine for disposal
                );

                set({
                    engine: result.engine,
                    scaledVertices: result.scaledVertices,
                    isReady: true,
                    isLoading: false,
                    substeps: result.engine.getSubsteps(),
                    solverIterations: result.engine.getSolverIterations()
                });

            } catch (err) {
                console.error(err);
                set({ error: (err as Error).message, isLoading: false });
            }
        }, 10);
    },
});