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
from tasks.TienKung_tasks.TienKung_task_base import TienKung_Task_Base
# others
import numpy as np
# settings
np.set_printoptions(precision=4)
carb.settings.get_settings().set("persistent/app/omniverse/gamepadCameraControl", False)
logger.info("TienKung_task_04 imports all deps")


@Registry.register("TienKung_task_04")
class TienKung_Task_04(TienKung_Task_Base):

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
        logger.debug("tiangong2 runner in")
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
        stage = omni.usd.get_context().get_stage()
        self.container_prim = config_loader.task_config["task"]["container_prim"]
        self.switch1_prim = config_loader.task_config["task"]["switch1_prim"]
        self.joint1_prim = config_loader.task_config["task"]["joint1_prim"]
        self.joint2_prim = config_loader.task_config["task"]["joint2_prim"]
        self.joint3_prim = config_loader.task_config["task"]["joint3_prim"]
        self.joint1_usd_path = stage.GetPrimAtPath(self.joint1_prim)
        self.joint2_usd_path = stage.GetPrimAtPath(self.joint2_prim)
        self.joint3_usd_path = stage.GetPrimAtPath(self.joint3_prim)
        self.init_states()
        logger.success('tiangong2 runner initialized')

    # update_joint_callback and collect_data_callback are inherited from base

    def check_success_callback(self, step_size) -> None:
        joint1_state, _ = self.task_checker.get_joint_state(joint_prim=self.joint1_usd_path)
        joint2_state, _ = self.task_checker.get_joint_state(joint_prim=self.joint2_usd_path)
        joint3_state, _ = self.task_checker.get_joint_state(joint_prim=self.joint3_usd_path)
        if self.task_checker.check_relative_position(
            a_path=self.switch1_prim,
            b_path=self.container_prim,
            relation="inside",
            inside_tolerance=np.zeros(6)) \
        and joint1_state > -0.02 \
        and joint2_state > -0.02 \
        and joint3_state > -0.02:
            self.task_success_flag = True
            print("task success")
            logger.info("Task Success!")
            # Write success status to file
            self.write_task_status(completed=True, success=True, score=1.0)
            self.stop()
        # return self.task_success_flag


