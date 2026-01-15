# Virtual Try-On Research Lab

A consolidated research monorepo dedicated to **Virtual Try-On (VTO)** and **Computational Fashion**. This repository houses my experimental engines for simulating high-fidelity cloth physics, garment collision, and real-time interaction using varied technology stacks (Python/MuJoCo and Rust/WASM).

## 📂 Repository Structure

This monorepo is organized into specialized sub-projects, each exploring a different approach to fabric simulation:

```text
virtual-try-on/
│
├── mujoco-simulation/       # [Python] High-fidelity offline simulation
│   ├── core/               # Physics stepping & model generation
│   ├── config/             # Gravity, damping & continuum mechanics
│   └── README.md           # Documentation for this specific engine
│
├── verlet-rapier-xpbd-simulation/        # [Rust + Web] Real-time browser-based VTO
│   ├── physics-core/       # The Rust XPBD solver (WASM)
│   ├── src/v4/             # React Three Fiber frontend
│   └── README.md           # Documentation for this specific engine
│
└── README.md               # You are here

```

---

## 🔬 Project Overviews

### 1. MuJoCo Cloth Simulation (M1 Optimized)

* **Location:** `/mujoco-simulation`
* **Focus:** High-precision offline physics and continuum mechanics.
* **Tech Stack:** Python 3.11, MuJoCo 3.x (`<flexcomp>`), `mjpython`.
* **Key Features:**
* Uses the `<flexcomp>` engine for realistic elasticity and edge damping.
* Modular architecture separating model generation, simulation, and rendering.
* Optimized for Apple Silicon (M1/M2) using `mjpython` for correct windowing.
* Includes interactive mouse controls to drag and throw cloth.

### 2. 3D Garment Visualization Engine (V4)

* **Location:** `/verlet-rapier-xpbd-simulation`
* **Focus:** Industrial-grade, real-time web simulation using Extended Position Based Dynamics (XPBD).
* **Tech Stack:** Rust (WASM), React Three Fiber, TypeScript.
* **Key Features:**
* **Hybrid Compute Model:** Offloads physics to Rust while rendering in React.
* **Zero-Copy Architecture:** JavaScript reads physics data directly from Rust memory without serialization overhead.
* **Advanced Collision:** Uses Analytic SDFs (Signed Distance Fields) for body collision and Spatial Hashing for self-collision.
* **Skinning:** Maps high-res visual meshes (~15k verts) to low-res physics proxies (~1k verts).

---

## 🚀 Getting Started

To work on a specific engine, navigate to its directory and follow its specific installation instructions.

### For the Python Engine

```bash
cd mujoco-simulation
# Follow the setup in mujoco-simulation/README.md

```

### For the Web Engine

```bash
cd verlet-rapier-xpbd-simulation
# Follow the setup in verlet-rapier-xpbd-simulation/README.md

```

---

## 📜 History & Context

This repository represents the evolution of my R&D into digital fashion:

1. **Early Experiments:** Initial trials in Python to understand constraint formulation.
2. **Web Migration:** Moving to TypeScript/React for accessibility.
3. **Performance Optimization:** Rewriting core solvers in Rust (WASM) to achieve zero-vibration stability and real-time interaction.
4. **Fidelity Research:** Using MuJoCo to study continuum mechanics and ground-truth physics behaviors.

## 🤝 License

This project is open-source. Please see the individual sub-folder `LICENSE` files for specific usage rights regarding the different technology stacks.
