// src/presentation/hooks/useProfiler.ts

import { useCallback, useRef, useState, useEffect } from 'react';
// import { profiler_get_report, profiler_reset, profiler_set_enabled } from '../../infrastructure/physics/wasm';
import type { ProfileReport } from '../../infrastructure/physics/adapter/WasmAdapter';

interface ProfilerHook {
    profileData: ProfileReport | null;
    isEnabled: boolean;
    particleCount: number;
    substeps: number;
    solverIterations: number;
    getProfileData: () => ProfileReport | null;
    resetProfiler: () => void;
    setEnabled: (enabled: boolean) => void;
}

export function useProfiler(updateInterval = 500): ProfilerHook {
    const [profileData, setProfileData] = useState<ProfileReport | null>(null);
    const [isEnabled, setIsEnabled] = useState(true);

    // We can't easily get these from the engine through the current architecture,
    // so use defaults. These will be updated via props in the overlay.
    const particleCountRef = useRef(0);
    const substepsRef = useRef(6);
    const solverIterationsRef = useRef(12);

    const getProfileData = useCallback((): ProfileReport | null => {
        return null;
        /*
        try {
            const json = profiler_get_report();
            if (!json || json === '{}') return null;
            return JSON.parse(json) as ProfileReport;
        } catch {
            return null;
        }
        */
    }, []);

    const resetProfiler = useCallback(() => {
        // profiler_reset();
        setProfileData(null);
    }, []);

    const setEnabled = useCallback((enabled: boolean) => {
        // profiler_set_enabled(enabled);
        setIsEnabled(enabled);
    }, []);

    // Auto-update profile data
    useEffect(() => {
        if (!isEnabled) return;

        const interval = setInterval(() => {
            const data = getProfileData();
            setProfileData(data);
        }, updateInterval);

        return () => clearInterval(interval);
    }, [getProfileData, updateInterval, isEnabled]);

    return {
        profileData,
        isEnabled,
        particleCount: particleCountRef.current,
        substeps: substepsRef.current,
        solverIterations: solverIterationsRef.current,
        getProfileData,
        resetProfiler,
        setEnabled,
    };
}

export default useProfiler;
