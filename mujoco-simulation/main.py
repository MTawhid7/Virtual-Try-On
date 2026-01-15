import time
import signal
import sys
import mujoco
from config.settings import SimConfig
from core.model_builder import ModelBuilder
from core.simulation import SimulationManager
from core.renderer import MuJoCoRenderer
from utils.logger import PhysicsLogger

def signal_handler(sig, frame):
    print("\n[Main] Interrupt received, shutting down...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)

    config = SimConfig()

    logger = PhysicsLogger(output_dir=config.output_dir, buffer_size=config.log_flush_interval)
    logger.log_console("Initializing MuJoCo Cloth Simulation (M1 Optimized)...")

    builder = ModelBuilder(config)
    xml_path = builder.save_xml("cloth_scene.xml")
    logger.log_console(f"Generated MJCF saved to: {xml_path}")

    sim = SimulationManager(xml_path, config, logger)

    renderer = MuJoCoRenderer(sim.model, sim.data, config)
    renderer.start()

    logger.log_console("Starting simulation loop...")

    steps = 0
    last_render_time = time.time()

    try:
        while renderer.is_running():
            success = sim.step()
            if not success:
                logger.log_console("Simulation stopped due to physics error.", level="FATAL")
                break

            if steps % config.log_frequency_steps == 0:
                diag = sim.get_diagnostics()
                logger.log_step(diag)

            current_time = time.time()
            if current_time - last_render_time >= config.render_interval:
                renderer.update()
                last_render_time = current_time

            steps += 1

    except Exception as e:
        logger.log_console(f"Unexpected crash: {e}", level="FATAL")
    finally:
        logger.close()
        renderer.close()
        logger.log_console("Shutdown complete.")

if __name__ == "__main__":
    main()