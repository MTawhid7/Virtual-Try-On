# MuJoCo Cloth Simulation (M1 Optimized)

A modular, object-oriented physics simulation built with Python and the official [MuJoCo](https://mujoco.org/) bindings. This project demonstrates high-fidelity cloth physics using the `<flexcomp>` (Flexible Composite) engine, optimized for Apple Silicon (M1/M2/M3) architecture.

## 🌟 Features

* **Modular Architecture:** Separation of concerns between Model Generation, Physics Simulation, Rendering, and Logging.
* **Modern Cloth Physics:** Uses MuJoCo 3.x `<flexcomp>` with continuum mechanics (elasticity/edge damping).
* **Interactive:** Drag, pull, and throw the cloth using mouse controls.
* **Data Logging:** High-frequency CSV logging of kinetic energy, contact points, and warning stats.
* **M1 Native:** Uses `mjpython` for correct Cocoa windowing support on macOS.

## 📂 Project Structure

```text
├── config/           # Simulation constants (gravity, grid size, timestep)
├── core/
│   ├── model_builder.py  # Procedural MJCF (XML) generation
│   ├── simulation.py     # Physics stepping and data management
│   └── renderer.py       # Passive viewer wrapper
├── utils/            # Buffered CSV logging
├── assets/           # Generated XMLs and run logs (ignored by git)
└── main.py           # Entry point
```

## 🚀 Installation

1. **Prerequisites:** Python 3.11 recommended.
2. **Clone the repo:**

    ```bash
    git clone https://github.com/MTawhid7/mujoco-cloth-simulation.git
    cd mujoco-cloth-simulation
    ```

3. **Create a virtual environment:**

    ```bash
    python -m venv .venv
    source .venv/bin/activate
    ```

4. **Install dependencies:**

    ```bash
    pip install -r requirements.txt
    ```

## 🎮 Usage

**Important for macOS Users:**
Due to GUI threading constraints on Apple Silicon, you must run the simulation using the `mjpython` wrapper (installed automatically with the mujoco package).

```bash
mjpython main.py
```

### Controls

* **Right-Click + Drag:** Grab and pull the cloth.
* **Left-Click + Drag:** Rotate camera.
* **Scroll:** Zoom in/out.
* **Double-Right-Click:** Select a specific particle (highlight).

## 📊 Performance Note

If the simulation appears to go into "slow motion" when the cloth hits an object, this is **Time Dilation**. The physics engine is calculating complex collision constraints that take longer than the real-time timestep (2ms). This ensures physical accuracy over visual speed.

## 🛠️ Troubleshooting

* **`RuntimeError: launch_passive requires... mjpython`**: You ran `python main.py`. Use `mjpython main.py` instead.
* **Cloth explodes:** Check `config/settings.py` and reduce the `timestep` or increase `damping`.
