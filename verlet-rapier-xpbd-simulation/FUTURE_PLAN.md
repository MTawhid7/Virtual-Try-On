# 🚀 Project Roadmap: Industrial-Grade Virtual Try-On (V4)

## 🎯 Objective

To build a high-performance, browser-based Virtual Try-On engine capable of simulating arbitrary garments on arbitrary bodies (SMPL) with realistic draping, zero vibration, and mobile compatibility.

---

## 🏗️ Architecture V4: The Hybrid Compute Model

We are transitioning from a pure TypeScript Object-Oriented approach to a **Data-Oriented Rust/WASM** architecture.

### 1. The Core (Rust + WASM)

* **Role:** Physics Solver, Collision Detection, Constraint Management.
* **Memory Model:** **Zero-Copy Shared Memory**. Rust owns the data; JavaScript views it via `Float32Array`.
* **Solver:** XPBD (Extended Position Based Dynamics) with sub-stepping.
* **Collision:** **Signed Distance Fields (SDF)**. Trilinear interpolation for smooth, O(1) collision checks against the body.

### 2. The Bridge (TypeScript)

* **Role:** Orchestration and Rendering.
* **Asset Loading:** Parsing GLB files, **Welding Vertices**, and extracting data for Rust.
* **Rendering:** React Three Fiber (R3F) renders the mesh using the shared memory buffer directly.
* **Skinning:** Maps the High-Res visual mesh to the Low-Res physics proxy.

---

## 📅 Execution Phases

### ✅ Phase 1: The Rust Foundation (Completed)

* **Goal:** Port V3 logic to Rust to eliminate GC pauses and enable SDFs.
* **Achievements:**
  * [x] Structure of Arrays (SoA) data layout.
  * [x] Analytic SDF Collider (T-Shape Capsule) implemented.
  * [x] Zero-Copy Memory Bridge established.
  * [x] Visual Skinning (Visual Mesh driven by Physics Proxy).
  * [x] **Stability:** Solved "Confetti" issues via Geometry Welding and Tether Constraints.

### 🚧 Phase 2: The Fitting Pipeline (Next)

* **Goal:** Support arbitrary shirts on arbitrary mannequins.
* **Key Features:**
  * **SDF Generation:** Convert GLB Mannequin -> 3D Texture (SDF) for accurate collision.
  * **Landmark Matching:** Auto-align neck/shoulders.
  * **Inflation Routine:** Pre-simulation step to resolve initial intersections (preventing explosions).
  * **SMPL Integration:** Support for parametric bodies.

### 🔮 Phase 3: Advanced Physics

* **Goal:** Realism and Material Accuracy.
* **Key Features:**
  * **Fabric Library:** Real-world material constants (Cotton, Silk, Denim).
  * **Aerodynamics:** Lift and Drag based on triangle normals.
  * **Seam Stiffness:** Reinforced constraints for structural edges.

---

## 🧠 Lessons Learned (From Phase 1)

1. **Topology Matters:** GLB meshes often have split vertices. **Welding** is mandatory before physics, or the cloth falls apart.
2. **Structure:** Distance constraints alone are insufficient. **Bending** and **Tether** constraints are required to hold the garment's shape.
3. **Collision:** Simple capsules are too slippery. A **T-Shape (Spine + Shoulders)** SDF with high friction on top surfaces is required for stable draping.
4. **Visuals:** Rendering the physics proxy looks bad. **Skinning** a high-res mesh to the low-res proxy is essential for visual fidelity.
