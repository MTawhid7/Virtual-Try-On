"""
Export GLB files to the Next.js frontend public/models/ directory.

Copies simulation output GLBs from backend/storage/ into
frontend/public/models/ so the web viewer can load them at /models/<name>.

Usage (run from backend/):
    python -m scripts.export_for_viewer                               # copy all storage/*.glb
    python -m scripts.export_for_viewer storage/garment_drape.glb    # specific file
    python -m scripts.export_for_viewer --open                        # copy + open browser
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import webbrowser
from pathlib import Path


_BACKEND_DIR = Path(__file__).parent.parent
_STORAGE_DIR = _BACKEND_DIR / "storage"
_FRONTEND_MODELS_DIR = _BACKEND_DIR.parent / "frontend" / "public" / "models"


def copy_glb(src: Path, dst_dir: Path) -> Path:
    dst_dir.mkdir(parents=True, exist_ok=True)
    dst = dst_dir / src.name
    shutil.copy2(src, dst)
    return dst


def main() -> None:
    parser = argparse.ArgumentParser(description="Copy simulation GLBs to frontend viewer.")
    parser.add_argument(
        "files",
        nargs="*",
        help="GLB files to copy. Defaults to all files in storage/.",
    )
    parser.add_argument(
        "--open",
        action="store_true",
        help="Open the viewer in the browser after copying (requires `npm run dev` to be running).",
    )
    args = parser.parse_args()

    if args.files:
        sources = [Path(f) for f in args.files]
    else:
        sources = sorted(_STORAGE_DIR.glob("*.glb"))

    if not sources:
        print(f"No GLB files found in {_STORAGE_DIR}")
        sys.exit(1)

    print(f"Destination: {_FRONTEND_MODELS_DIR}")
    for src in sources:
        if not src.exists():
            print(f"  SKIP (not found): {src}")
            continue
        dst = copy_glb(src, _FRONTEND_MODELS_DIR)
        size_kb = dst.stat().st_size / 1024
        print(f"  Copied: {src.name}  ({size_kb:.0f} KB)")

    print(f"\nDone. {len(sources)} file(s) available at /models/<name>.glb")
    print("Start the viewer with:  cd frontend && npm run dev")

    if args.open:
        webbrowser.open("http://localhost:3000")


if __name__ == "__main__":
    main()
