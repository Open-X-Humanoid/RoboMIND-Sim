import sys
import os
# append to sys path
project_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if project_path not in sys.path:
    sys.path.append(project_path)
######################################################################
# isaac sim
from isaacsim.simulation_app import SimulationApp
import omni
import carb
import omni.kit.commands
from isaacsim.core.utils.stage import is_stage_loading
# project
from common.logger_loader import logger
from common.config_loader import config_loader
from common.x_registry import Registry
# import robots
from tasks.TienKung_tasks.TienKung_task_base import TienKung_Task_Base
# others
import numpy as np
# settings
np.set_printoptions(precision=4)
carb.settings.get_settings().set("persistent/app/omniverse/gamepadCameraControl", False)
logger.info("TienKung_task_03 imports all deps")


@Registry.register("TienKung_task_03")
class TienKung_Task_03(TienKung_Task_Base):

    def __init__(
        self,
        simulation_app: SimulationApp,
        physics_dt=1.0 / 120,
        render_dt=1.0 / 120,
        stage_units_in_meters=1.00,
        environment_path=None,
        robot_init_position=None,
        robot_init_orientation=None,
        status_file_path=None,
    ):
        logger.debug("TienKung_Task_03 init")
        super().__init__(simulation_app=simulation_app,
                         physics_dt=physics_dt,
                         render_dt=render_dt,
                         stage_units_in_meters=stage_units_in_meters,
                         environment_path=environment_path,
                         status_file_path=status_file_path)
        # Set rendering
        # Smaller number reduces material resolution to avoid out-of-memory, max is 15
        carb.settings.get_settings().set("/rtx-transient/resourcemanager/maxMipCount", 15)
        # Enable DLSS
        carb.settings.get_settings().set("/rtx-transient/dlssg/enabled", True)
        # omni.kit.commands.execute("ModifyLayerMetadata",
        #                           layer_identifier=self.environment_path,
        #                           parent_layer_identifier=None,
        #                           meta_index=2,
        #                           value=120)
        # "Auto": 3, "Performance": 0, "Balanced": 1, "Quality": 2
        carb.settings.get_settings().set("/rtx/post/dlss/execMode", 0)
        logger.info("rendering mode all set")
        # Tiangong
        self.robot = Registry.create(
            "TienKung2_Inspire",
            init_position=robot_init_position,
            init_orientation=robot_init_orientation,
            l_arm_home_pose=self.l_arm_home_pose,
            r_arm_home_pose=self.r_arm_home_pose,
        )
        logger.info('TienKung2_Inspire initialized.')
        while is_stage_loading():
            simulation_app.update()
        # Task-specific configuration
        self.container_prim = config_loader.task_config["task"]["container_prim"]
        self.switch1_prim = config_loader.task_config["task"]["switch1_prim"]
        self.switch2_prim = config_loader.task_config["task"]["switch2_prim"]
        self.init_states()
        logger.success('TienKung_Task_03 initialized')

    def check_success_callback(self, step_size) -> None:
        if self.task_checker.check_relative_position(
            a_path=self.switch1_prim,
            b_path=self.container_prim,
            relation="inside",
            inside_tolerance=np.zeros(6))\
        and self.task_checker.check_relative_position(
             a_path=self.switch2_prim,
             b_path=self.container_prim,
             relation="inside",
             inside_tolerance=np.zeros(6)):
            self.task_success_flag = True
            logger.info("Task Success!")
            # Write success status to file
            self.write_task_status(completed=True, success=True, score=1.0)
            self.stop()
