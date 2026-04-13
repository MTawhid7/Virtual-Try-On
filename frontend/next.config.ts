import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  // Disable StrictMode: React 19 StrictMode double-mounts components in dev,
  // which creates two WebGL contexts and causes the browser to kill the first
  // one (THREE.WebGLRenderer: Context Lost). R3F apps require this.
  reactStrictMode: false,
};

export default nextConfig;
