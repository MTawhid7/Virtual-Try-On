"""
CLI entry point for the simulation engine.

Usage:
    python -m simulation --scene freefall
    python -m simulation --scene constrained_fall  (Sprint 1 Layer 2)
    python -m simulation --scene sphere_drape      (Sprint 1 Layer 3a)
    python -m simulation --pattern tshirt --fabric cotton  (Sprint 2)
"""

import argparse
import sys

from simulation.scenes import run_freefall, run_constrained_fall, run_sphere_drape

def main() -> None:
    parser = argparse.ArgumentParser(description="Garment Simulation Engine")
    parser.add_argument(
        "--scene", type=str, default="freefall",
        choices=["freefall", "constrained_fall", "sphere_drape"],
        help="Built-in test scene to run",
    )
    parser.add_argument(
        "--visualize", "-v", action="store_true",
        help="Launch Taichi live 3D visualizer alongside simulation.",
    )
    # Future args:
    # parser.add_argument("--pattern", type=str, help="Pattern JSON file")
    # parser.add_argument("--fabric", type=str, help="Fabric preset name")

    args = parser.parse_args()

    if args.scene == "freefall":
        run_freefall(visualize=args.visualize)
    elif args.scene == "constrained_fall":
        run_constrained_fall(visualize=args.visualize)
    elif args.scene == "sphere_drape":
        run_sphere_drape(visualize=args.visualize)
    else:
        print(f"Unknown scene: {args.scene}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
