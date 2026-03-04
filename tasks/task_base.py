# isaac sim
from isaacsim.simulation_app import SimulationApp
import omni
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.stage import is_stage_loading
# project
from common.logger_loader import logger
from common.config_loader import config_loader
# others
import os
import time
import hashlib
import numpy as np

logger.debug("task base imports all deps")


class TaskRunnerBase(object):

    def __init__(self,
                 simulation_app: SimulationApp,
                 physics_dt: float = 1.0 / 120,
                 render_dt: float = 1.0 / 120,
                 stage_units_in_meters: float = 1.00,
                 environment_path: str = None):
        logger.debug("TaskRunnerBase in")
        self.simulation_app = simulation_app
        # Set environment path with random selection if directory
        if environment_path:
            # Use the provided path directly (selection is handled by task_manager)
            self.abs_environment_path = os.path.abspath(environment_path)
        else:
            logger.warning("No environment path provided")
            self.abs_environment_path = None
        logger.debug(f"Using environment path: {self.abs_environment_path}")
        omni.usd.get_context().open_stage(self.abs_environment_path)
        logger.success("Digital env loaded")
        omni.usd.get_context().disable_save_to_recent_files()
        self.physics_dt = physics_dt
        self.render_dt = render_dt
        self.simulation_context = SimulationContext(physics_dt=self.physics_dt,
                                                    rendering_dt=self.render_dt,
                                                    stage_units_in_meters=stage_units_in_meters)
        logger.debug("simulation_context loaded")
        self.update_sim(50)
        while is_stage_loading():
            self.simulation_app.update()
        logger.debug("TaskRunnerBase Done")

    def init_states(self) -> None:
        self.sim_step = 0
        self.buffer_pool_align = {
            "puppet": {},
            "master": {},
            "camera_observations": {
                "color_images": {},
                "depth_images": {}
            }
        }
        for cam_name in config_loader.robot_config['cam']:
            resolution = config_loader.robot_config['cam'][cam_name]['cam_resolution']
            self.buffer_pool_align["camera_observations"]["color_images"][cam_name] = np.zeros(
                (resolution[1], resolution[0], 3), dtype=np.uint8)
            self.buffer_pool_align["camera_observations"]["depth_images"][cam_name] = np.zeros(
                (resolution[1], resolution[0]), dtype=np.float32)
        logger.success("init states with buffer pool")

    def play(self) -> None:
        omni.timeline.get_timeline_interface().stop()
        omni.timeline.get_timeline_interface().play()
        self.update_sim(10)

    def update_sim(self, num_steps):
        for _ in range(num_steps):
            self.simulation_app.update()

    def one_step(self):
        self.simulation_context.step()

    def shut_down(self, kill_instantly: bool = True):
        logger.info("trying to shutdown")
        if kill_instantly:
            self.simulation_context.clear_all_callbacks()
            self.simulation_context.clear_instance()
            logger.info("clear all camera")
            del self.robot
            logger.info("clear robot")
            del self.simulation_context
            logger.info("clear simulation_context")
            os.system("pkill -9 -f \"python.sh ~\"")
            self.simulation_app.close()
        else:
            while self.simulation_app.is_running():
                self.simulation_app.update()
