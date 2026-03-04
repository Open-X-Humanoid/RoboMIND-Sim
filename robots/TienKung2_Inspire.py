# project
from common.logger_loader import logger
from common.config_loader import config_loader
from common.x_registry import Registry
from robots.arm_base import ArmBase
from robots.cam.sim_cam import SimCam
from common.utils.transform_utils import StandaloneUtils
# isaac sim
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.types import ArticulationActions
from isaacsim.core.prims import SingleXFormPrim
import isaacsim.core.utils.rotations as rotations_utils
from pxr import Gf, UsdPhysics
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.prims import get_prim_at_path
# others
from typing import Optional
import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation as R
import math

logger.debug("TienKung2 import passed")


@Registry.register("TienKung2_Inspire")
class TienKung2_Inspire(ArmBase):

    def __init__(
        self,
        init_position: Optional[npt.NDArray[np.float64]] = None,
        init_orientation: Optional[npt.NDArray[np.float64]] = None,
        l_arm_home_pose: Optional[npt.NDArray[np.float64]] = None,
        r_arm_home_pose: Optional[npt.NDArray[np.float64]] = None,
    ):
        logger.debug("in TienKung class")
        config_loader.load_robot_toml("TienKung2_Inspire")
        super().__init__()
        logger.debug("humanoid base class passed")
        self.init_position = init_position if init_position is not None else config_loader.robot_config["robot"][
            "config"]["init_position"]
        self.init_orientation = init_orientation if init_orientation is not None else config_loader.robot_config[
            "robot"]["config"]["init_orientation"]
        self.l_arm_home_pose = l_arm_home_pose if l_arm_home_pose is not None else config_loader.robot_config["robot"][
            "config"]["l_arm_home_pose"]
        self.r_arm_home_pose = r_arm_home_pose if r_arm_home_pose is not None else config_loader.robot_config["robot"][
            "config"]["r_arm_home_pose"]
        logger.debug(self.init_position)
        logger.debug(self.init_orientation)
        logger.debug("humanoid base class passed")
        # Insert robot
        add_reference_to_stage(self.robot_usd_path, self.NAMESPACE)
        logger.debug("robot already in stage")
        # Update robot world pose.
        self.change_pose(self.init_position, self.init_orientation, True)
        logger.debug("change pose")
        self.robot_prim = SingleXFormPrim(prim_path=self.NAMESPACE,
                                          position=np.array([self.init_position[0], self.init_position[1], 0.0]),
                                          orientation=rotations_utils.euler_angles_to_quat(self.init_orientation, True))
        self.left_ee_prim = SingleXFormPrim(self.NAMESPACE + "/humanoid/handleft9183")
        self.right_ee_prim = SingleXFormPrim(self.NAMESPACE + "/humanoid/handright9253")
        self.base_prim = SingleXFormPrim(self.NAMESPACE + "/humanoid/pelvis")
        self.init_cam()
        logger.success("init TienKung2")

    def change_pose(self, pos, rot, degrees=True) -> None:
        #pelvis
        pelvis_joint_prim = get_prim_at_path(self.NAMESPACE + "/humanoid/pelvis/FixedJoint")
        pelvis_joint = UsdPhysics.Joint(pelvis_joint_prim)
        # pelvis_joint_new_locpos0 = np.array([0.0, 0.0, 0.0]) + pos
        pelvis_joint.GetLocalRot0Attr().Set(
            Gf.Quatf(*rotations_utils.euler_angles_to_quat(rot, degrees=degrees).astype(float)))
        pelvis_joint.GetLocalPos0Attr().Set(Gf.Vec3f(*np.array(pos, dtype=float)))

    def init_cam(self):
        self.cam_dict = {}
        self.cam_dict["camera_head"] = SimCam(
            resolution=tuple(config_loader.robot_config['cam']['camera_head']['cam_resolution']),
            frequency=int(config_loader.robot_config['cam']["camera_head"]['frequency']),
            focal_length=float(config_loader.robot_config['cam']["camera_head"]['focal_length']),
            clip_range=(0.01, 10.0),
            cam_name=self.NAMESPACE + config_loader.robot_config['cam']["camera_head"]['base_link'],
            translation=config_loader.robot_config['cam']["camera_head"]['position'],
            orientation=config_loader.robot_config['cam']["camera_head"]['orientation'],
            camera_matrix=config_loader.robot_config['cam']["camera_head"]['camera_matrix'])
        logger.debug("camera_head")

    def active_art(self):
        self.art = Articulation(self.NAMESPACE + config_loader.robot_config["robot"]["config"]["art_prim"])
        if self.art.is_non_root_articulation_link == True:
            logger.error("robot is not an articulation")
        self.art.initialize()
        self.left_arm_joint_handles = [
            self.art.get_dof_index(name) for name in config_loader.robot_config["robot"]["joint"]["l_arm_joint_name"]
        ]
        self.right_arm_joint_handles = [
            self.art.get_dof_index(name) for name in config_loader.robot_config["robot"]["joint"]["r_arm_joint_name"]
        ]
        self.left_ee_joint_handles = [
            self.art.get_dof_index(name) for name in config_loader.robot_config["robot"]["joint"]["l_ee_joint_name"]
        ]
        self.right_ee_joint_handles = [
            self.art.get_dof_index(name) for name in config_loader.robot_config["robot"]["joint"]["r_ee_joint_name"]
        ]
        #logger.debug(f"self.left_arm_joint_handles:{self.left_arm_joint_handles}")#[3, 8, 13, 18, 22, 26, 28]
        #logger.debug(f"self.right_arm_joint_handles:{self.right_arm_joint_handles}")#[4, 9, 14, 19, 23, 27, 29]
        #logger.debug(f"self.left_ee_joint_handles:{self.left_ee_joint_handles}")#[34, 44, 50, 52, 30, 40, 31, 41, 32, 42, 33, 43]
        #logger.debug(f"self.right_ee_joint_handles:{self.right_ee_joint_handles}")#[38, 48, 51, 53, 35, 45, 39, 49, 36, 46, 37, 47]
        logger.debug("joints activated")

    def update_command(self, command_dict):
        self.left_recv_arm_positions = command_dict["left_arm"]
        self.right_recv_arm_positions = command_dict["right_arm"]
        self.left_recv_ee_positions = command_dict["left_hand"]
        self.right_recv_ee_positions = command_dict["right_hand"]

    def pub_master_l_r_joints(self, curr_time):
        return {
            "arm_left_position_raw": {
                "timestamp": curr_time,
                "is_intervene": False,
                "data": self.left_recv_arm_positions
            },
            "arm_right_position_raw": {
                "timestamp": curr_time,
                "is_intervene": False,
                "data": self.right_recv_arm_positions
            },
            "end_effector_left_position_raw": {
                "timestamp": curr_time,
                "is_intervene": False,
                "data": self.left_recv_ee_positions
            },
            "end_effector_right_position_raw": {
                "timestamp": curr_time,
                "is_intervene": False,
                "data": self.right_recv_ee_positions
            },
        }

    def pub_l_r_joints(self, curr_time):
        positions = self.art.get_joint_positions().squeeze()
        return {
            "arm_left_position_raw": {
                "timestamp":
                    curr_time,
                "is_intervene":
                    False,
                "data": [
                    positions[3], positions[8], positions[13], positions[18], positions[22], positions[26],
                    positions[28]
                ]
            },
            "arm_right_position_raw": {
                "timestamp":
                    curr_time,
                "is_intervene":
                    False,
                "data": [
                    positions[4], positions[9], positions[14], positions[19], positions[23], positions[27],
                    positions[29]
                ]
            },
            "end_effector_left_position_raw": {
                "timestamp":
                    curr_time,
                "is_intervene":
                    False,
                "data": [
                    positions[34],
                    positions[44],
                    positions[50],
                    positions[52],
                    positions[30],
                    positions[40],
                    positions[31],
                    positions[41],
                    positions[32],
                    positions[42],
                    positions[33],
                    positions[43],
                ]
            },
            "end_effector_right_position_raw": {
                "timestamp":
                    curr_time,
                "is_intervene":
                    False,
                "data": [
                    positions[38],
                    positions[48],
                    positions[51],
                    positions[53],
                    positions[35],
                    positions[45],
                    positions[39],
                    positions[49],
                    positions[36],
                    positions[46],
                    positions[37],
                    positions[47],
                ]
            },
            "end_effector_left_pose_raw": {
                "timestamp":
                    curr_time,
                "is_intervene":
                    False,
                "data":
                    StandaloneUtils.affine_to_xyz_quaternion(
                        StandaloneUtils.transform_xfroms_pose(self.left_ee_prim, self.base_prim))
            },
            "end_effector_right_pose_raw": {
                "timestamp":
                    curr_time,
                "is_intervene":
                    False,
                "data":
                    StandaloneUtils.affine_to_xyz_quaternion(
                        StandaloneUtils.transform_xfroms_pose(self.right_ee_prim, self.base_prim))
            },
        }

    def joint_callback(self, step_size) -> None:
        # Arm control in base class
        for idx in range(self.num_arm_dof):
            self.art.apply_action(
                ArticulationActions(joint_positions=self.left_recv_arm_positions[idx],
                                    joint_indices=self.left_arm_joint_handles[idx]))
            self.art.apply_action(
                ArticulationActions(joint_positions=self.right_recv_arm_positions[idx],
                                    joint_indices=self.right_arm_joint_handles[idx]))
        # Dex
        # Right
        offset = 20.0
        # Thumb 40,41,42,43
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-90.0), joint_indices=self.right_ee_joint_handles[0]))
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(5.0) + math.radians(80. - offset) *
                                (1.0 - float(self.right_recv_ee_positions[-2])),
                                joint_indices=self.right_ee_joint_handles[1]))
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(0.) - math.radians(10) *
                                (1.0 - float(self.right_recv_ee_positions[-1])),
                                joint_indices=self.right_ee_joint_handles[2]))
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(0.0), joint_indices=self.right_ee_joint_handles[3]))
        # Index 00,01
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-5.0) + math.radians(-70.0 + offset) *
                                (1.0 - float(self.right_recv_ee_positions[3])),
                                joint_indices=self.right_ee_joint_handles[4]))
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-10.0) + math.radians(-85.0 + offset) *
                                (1.0 - float(self.right_recv_ee_positions[3])),
                                joint_indices=self.right_ee_joint_handles[5]))
        # Middle 10,11
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-5.0) + math.radians(-70.0 + offset) *
                                (1.0 - float(self.right_recv_ee_positions[2])),
                                joint_indices=self.right_ee_joint_handles[6]))
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-10.0) + math.radians(-85.0 + offset) *
                                (1.0 - float(self.right_recv_ee_positions[2])),
                                joint_indices=self.right_ee_joint_handles[7]))
        # Ring 20,21
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-5.0) + math.radians(-70.0 + offset) *
                                (1.0 - float(self.right_recv_ee_positions[1])),
                                joint_indices=self.right_ee_joint_handles[8]))
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-10.0) + math.radians(-85.0 + offset) *
                                (1.0 - float(self.right_recv_ee_positions[1])),
                                joint_indices=self.right_ee_joint_handles[9]))
        # Little 30,31
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-5.0) + math.radians(-70.0 + offset) *
                                (1.0 - float(self.right_recv_ee_positions[0])),
                                joint_indices=self.right_ee_joint_handles[10]))
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-10.0) + math.radians(-85.0 + offset) *
                                (1.0 - float(self.right_recv_ee_positions[0])),
                                joint_indices=self.right_ee_joint_handles[11]))

        # Left
        offset = 20.0
        # Thumb 40,41,42,43
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(90.0), joint_indices=self.left_ee_joint_handles[0]))
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-5.0) - math.radians(80. - offset) *
                                (1.0 - float(self.left_recv_ee_positions[-2])),
                                joint_indices=self.left_ee_joint_handles[1]))
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(0.) - math.radians(10) *
                                (1.0 - float(self.left_recv_ee_positions[-1])),
                                joint_indices=self.left_ee_joint_handles[2]))
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(0.0), joint_indices=self.left_ee_joint_handles[3]))
        # Index 00,01
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-5.0) + math.radians(-70.0 + offset) *
                                (1.0 - float(self.left_recv_ee_positions[3])),
                                joint_indices=self.left_ee_joint_handles[4]))
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-5.0) + math.radians(-70.0 + offset) *
                                (1.0 - float(self.left_recv_ee_positions[3])),
                                joint_indices=self.left_ee_joint_handles[5]))
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-10.0) + math.radians(-85.0 + offset) *
                                (1.0 - float(self.left_recv_ee_positions[3])),
                                joint_indices=self.left_ee_joint_handles[5]))
        # Middle 10,11
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-5.0) + math.radians(-70.0 + offset) *
                                (1.0 - float(self.left_recv_ee_positions[2])),
                                joint_indices=self.left_ee_joint_handles[6]))
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-10.0) + math.radians(-85.0 + offset) *
                                (1.0 - float(self.left_recv_ee_positions[2])),
                                joint_indices=self.left_ee_joint_handles[7]))
        # Ring 20,21
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-5.0) + math.radians(-70.0 + offset) *
                                (1.0 - float(self.left_recv_ee_positions[1])),
                                joint_indices=self.left_ee_joint_handles[8]))
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-10.0) + math.radians(-85.0 + offset) *
                                (1.0 - float(self.left_recv_ee_positions[1])),
                                joint_indices=self.left_ee_joint_handles[9]))
        # Little 30,31
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-5.0) + math.radians(-70.0 + offset) *
                                (1.0 - float(self.left_recv_ee_positions[0])),
                                joint_indices=self.left_ee_joint_handles[10]))
        self.art.apply_action(
            ArticulationActions(joint_positions=math.radians(-10.0) + math.radians(-85.0 + offset) *
                                (1.0 - float(self.left_recv_ee_positions[0])),
                                joint_indices=self.left_ee_joint_handles[11]))
