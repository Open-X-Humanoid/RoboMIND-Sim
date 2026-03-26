# project
from common.logger_loader import logger
from common.config_loader import config_loader
# isaac
from omni.isaac.dynamic_control import _dynamic_control
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
        # Robot dynamic_control
        self.dc = _dynamic_control.acquire_dynamic_control_interface()

    def change_pose(self, pos: np.ndarray, rot: np.ndarray, is_degrgripper: bool) -> None:
        raise NotImplementedError

    def active_art(self) -> None:
        self.art = self.dc.get_articulation(self.NAMESPACE + config_loader.robot_config["robot"]["config"]["art_prim"])
        if self.art == _dynamic_control.INVALID_HANDLE:
            logger.error("robot is not an articulation")
        self.dc.wake_up_articulation(self.art)
        # Left arm
        for item in config_loader.robot_config["robot"]["joint"]["l_arm_joint_name"]:
            self.left_arm_joint_handles.append(self.dc.find_articulation_dof(self.art, item))
        if self.arm_type == "dual":
            # Right arm
            for item in config_loader.robot_config["robot"]["joint"]["r_arm_joint_name"]:
                self.right_arm_joint_handles.append(self.dc.find_articulation_dof(self.art, item))
        logger.debug("arm activated")

    def joint_callback(self, step_size: int) -> None:
        # Arm
        for idx in range(self.num_arm_dof):
            self.dc.set_dof_position_target(self.left_arm_joint_handles[idx], self.left_recv_arm_positions[idx])
            if self.arm_type == "dual":
                self.dc.set_dof_position_target(self.right_arm_joint_handles[idx], self.right_recv_arm_positions[idx])
