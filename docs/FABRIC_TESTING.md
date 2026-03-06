# Fabric Testing Guide

Vistio supports simulating various authentic fabric types (e.g., Silk Charmeuse, Denim) using a physical material database based on Kawabata Evaluation System (KES) parameters.

This guide explains how you can configure your scenarios to test different types of fabrics, and how to evaluate their behavior in the simulation correctly.

## 1. Using Fabric Properties

To simulate specific real-world fabrics, you instantiate the garment with a `FabricProperties` definition from the `vistio_material::database`.

In `crates/vistio-bench/src/scenarios.rs`, you can attach a material when initializing a scenario. The easiest method is to use:

```rust
use vistio_material::FabricProperties;

// In scenarios.rs or in your executable runner:
let mut scenario = Scenario::cusick_drape()
    .with_material(FabricProperties::silk_charmeuse());
```

Available curated presets include:

- `FabricProperties::silk_charmeuse()`: Highly flexible, very low bending stiffness.
- `FabricProperties::denim_14oz()`: Very stiff, heavy, highly anisotropic.
- `FabricProperties::cotton_jersey()`: Stretchy, isotropic.

*(You can explore or add more definitions in `crates/vistio-material/src/database.rs`)*

## 2. Running Simulations for Evaluation

When evaluating material behavior, you can test them via the headless CLI (`vistio-bench`) or visualize them in real-time (`vistio-viewer`).

### Headless Benchmarking and Diagnostics

To precisely measure the stress, stretch, and kinetic energy differences between fabrics, use the `vistio-bench` headless runner.

```bash
cargo run --release --bin vistio-bench -- cusick_drape
```

As scenarios run, the `vistio-bench` harness evaluates macroscopic structure metrics. For example, in the `cusick_drape` scenario, it outputs a metric called **Drape Coefficient**, the standard 2D shadow projection metric representing how beautifully a fabric cascades under its own weight.

You can trace outputs and dump metrics (such as the difference in internal tension between materials in `uniaxial_stretch`) via the tools provided in `tools/`:

```bash
cargo run --release --bin dump_diagnostics
```

### Visual Testing

Run the Bevy viewer to see the draped silhouette differences:

```bash
cargo run --release --bin vistio-viewer -- cusick_drape
```

**Things to look for:**

- **Silk Charmeuse**: You should see many small, soft, unstructured folds dropping straight down.
- **Denim_14oz**: You should see only a few large, wide folds. The material will hold a significantly rigid structure and won't buckle as easily.

### Anisotropy Check

Modern woven fabrics are "Orthotropic" (they behave differently when stretched along the warp thread vs the weft thread). Because Vistio implements an `AnisotropicCoRotationalModel` (Tier 3), applying a predefined woven material will natively compute this difference.

Test it with the **uniaxial_stretch** geometry. By pinning the ends of different materials and pulling, you'll see a visual "necking" effect (Poisson's ratio) and differing stretch resistances based on the fabric's warp/weft matrices!
