// src/presentation/state/useSimulationStore.ts

import { create } from 'zustand';
import type { SimulationStore } from './types';
import { createLifecycleSlice } from './lifecycleSlice';
import { createSimulationSlice } from './simulationSlice';
import { createInteractionSlice } from './interactionSlice';

/**
 * The Global Store for the Simulation.
 *
 * Architecture:
 * - Uses the "Slice Pattern" to organize logic into distinct domains (Lifecycle, Simulation, Interaction).
 * - Acts as the Controller, acting as the glue between the UI and the Application Use Cases.
 * - Holds the "Single Source of Truth" for the UI state.
 */
export const useSimulationStore = create<SimulationStore>()((...a) => ({
    ...createLifecycleSlice(...a),
    ...createSimulationSlice(...a),
    ...createInteractionSlice(...a),
} as SimulationStore));