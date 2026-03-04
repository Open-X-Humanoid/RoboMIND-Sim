# project
from common.logger_loader import logger
from common.config_loader import config_loader
# isaac
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.types import ArticulationActions
# others
import os
import numpy as np

logger.debug("arm base imports all deps")


class ArmBase(object):

    def __init__(self):
        """
        Robot initialization
        """
        # Ensure absolute path based on project root
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        relative_usd_path = config_loader.robot_config["robot"]["config"]["usd_path"]
        self.robot_usd_path = os.path.join(project_root, relative_usd_path)
        self.NAMESPACE = config_loader.robot_config["robot"]["config"]["namespace"]
        self.num_arm_dof = config_loader.robot_config["robot"]["joint"]["num_arm_dof"]
        self.arm_type = config_loader.robot_config["robot"]["config"]["arm_type"]
        # Left arm
        self.left_recv_arm_positions = np.zeros(self.num_arm_dof)
        self.left_arm_joint_handles = []
        # Left gripper or hands
        self.left_recv_ee_positions = None
        self.left_ee_joint_handles = []
        if self.arm_type == "dual":
            self.right_recv_arm_positions = np.zeros(self.num_arm_dof)
            self.right_arm_joint_handles = []
            self.right_recv_ee_positions = None  # only one joint in puppet
            self.right_ee_joint_handles = []
        self.art = None  #type: Articulation

    def change_pose(self, pos: np.ndarray, rot: np.ndarray, is_degrgripper: bool) -> None:
        raise NotImplementedError

    def active_art(self) -> None:
        raise NotImplementedError

    def joint_callback(self, step_size: int) -> None:
        raise NotImplementedError
