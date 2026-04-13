import { NextResponse } from "next/server";
import fs from "fs";
import path from "path";

export async function GET() {
  const modelsDir = path.join(process.cwd(), "public", "models");

  if (!fs.existsSync(modelsDir)) {
    return NextResponse.json({ models: [] });
  }

  const files = fs
    .readdirSync(modelsDir)
    .filter((f) => f.endsWith(".glb"))
    .sort();

  return NextResponse.json({ models: files });
}
