// src/presentation/hooks/useMediaQuery.ts

import { useState, useEffect } from 'react';

/**
 * Custom hook that returns true if the media query matches.
 * Useful for building responsive components in React.
 *
 * @param query The media query to check (e.g., '(max-width: 768px)')
 * @returns boolean indicating if the query matches
 */
export function useMediaQuery(query: string): boolean {
    const [matches, setMatches] = useState(() => window.matchMedia(query).matches);

    useEffect(() => {
        const media = window.matchMedia(query);
        if (media.matches !== matches) {
            setMatches(media.matches);
        }

        const listener = () => setMatches(media.matches);
        media.addEventListener('change', listener);

        return () => media.removeEventListener('change', listener);
    }, [query, matches]);

    return matches;
}
