# project
from common.logger_loader import logger
from common.config_loader import config_loader
from common.x_registry import Registry
from robots.arm_base import ArmBase
from robots.cam.sim_cam import SimCam
from common.utils.transform_utils import StandaloneUtils
# isaac sim
from isaacsim.core.prims import SingleXFormPrim
from robots.arm_base import _dynamic_control
import isaacsim.core.utils.rotations as rotations_utils
# from isaacsim.simulation_app import SimulationApp
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
        super().active_art()
        # Dex
        for item in config_loader.robot_config["robot"]["joint"]["l_ee_joint_name"]:
            self.left_ee_joint_handles.append(self.dc.find_articulation_dof(self.art, item))
        for item in config_loader.robot_config["robot"]["joint"]["r_ee_joint_name"]:
            self.right_ee_joint_handles.append(self.dc.find_articulation_dof(self.art, item))
        logger.debug("extra joints activated")

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
        dof_states = self.dc.get_articulation_dof_states(self.art, _dynamic_control.STATE_ALL)
        return {
            "arm_left_position_raw": {
                "timestamp":
                    curr_time,
                "is_intervene":
                    False,
                "data": [
                    dof_states["pos"][3], dof_states["pos"][8], dof_states["pos"][13], dof_states["pos"][18],
                    dof_states["pos"][22], dof_states["pos"][26], dof_states["pos"][28]
                ]
            },
            "arm_right_position_raw": {
                "timestamp":
                    curr_time,
                "is_intervene":
                    False,
                "data": [
                    dof_states["pos"][4], dof_states["pos"][9], dof_states["pos"][14], dof_states["pos"][19],
                    dof_states["pos"][23], dof_states["pos"][27], dof_states["pos"][29]
                ]
            },
            "end_effector_left_position_raw": {
                "timestamp":
                    curr_time,
                "is_intervene":
                    False,
                "data": [
                    dof_states["pos"][34],
                    dof_states["pos"][44],
                    dof_states["pos"][50],
                    dof_states["pos"][52],
                    dof_states["pos"][30],
                    dof_states["pos"][40],
                    dof_states["pos"][31],
                    dof_states["pos"][41],
                    dof_states["pos"][32],
                    dof_states["pos"][42],
                    dof_states["pos"][33],
                    dof_states["pos"][43],
                ]
            },
            "end_effector_right_position_raw": {
                "timestamp":
                    curr_time,
                "is_intervene":
                    False,
                "data": [
                    dof_states["pos"][38],
                    dof_states["pos"][48],
                    dof_states["pos"][51],
                    dof_states["pos"][53],
                    dof_states["pos"][35],
                    dof_states["pos"][45],
                    dof_states["pos"][39],
                    dof_states["pos"][49],
                    dof_states["pos"][36],
                    dof_states["pos"][46],
                    dof_states["pos"][37],
                    dof_states["pos"][47],
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
        super().joint_callback(step_size)
        # Dex
        # Right
        offset = 20.0
        # Thumb 40,41,42,43
        self.dc.set_dof_position_target(self.right_ee_joint_handles[0], math.radians(-90.0))
        self.dc.set_dof_position_target(
            self.right_ee_joint_handles[1],
            math.radians(5.0) + math.radians(80. - offset) * (1.0 - float(self.right_recv_ee_positions[-2])))
        self.dc.set_dof_position_target(
            self.right_ee_joint_handles[2],
            math.radians(0.) - math.radians(10) * (1.0 - float(self.right_recv_ee_positions[-1])))
        self.dc.set_dof_position_target(self.right_ee_joint_handles[3], math.radians(0.0))
        # Index 00,01
        self.dc.set_dof_position_target(
            self.right_ee_joint_handles[4],
            math.radians(-5.0) + math.radians(-70.0 + offset) * (1.0 - float(self.right_recv_ee_positions[3])))
        self.dc.set_dof_position_target(
            self.right_ee_joint_handles[5],
            math.radians(-10.0) + math.radians(-85.0 + offset) * (1.0 - float(self.right_recv_ee_positions[3])))
        # Middle 10,11
        self.dc.set_dof_position_target(
            self.right_ee_joint_handles[6],
            math.radians(-5.0) + math.radians(-70.0 + offset) * (1.0 - float(self.right_recv_ee_positions[2])))
        self.dc.set_dof_position_target(
            self.right_ee_joint_handles[7],
            math.radians(-10.0) + math.radians(-85.0 + offset) * (1.0 - float(self.right_recv_ee_positions[2])))
        # Ring 20,21
        self.dc.set_dof_position_target(
            self.right_ee_joint_handles[8],
            math.radians(-5.0) + math.radians(-70.0 + offset) * (1.0 - float(self.right_recv_ee_positions[1])))
        self.dc.set_dof_position_target(
            self.right_ee_joint_handles[9],
            math.radians(-10.0) + math.radians(-85.0 + offset) * (1.0 - float(self.right_recv_ee_positions[1])))
        # Little 30,31
        self.dc.set_dof_position_target(
            self.right_ee_joint_handles[10],
            math.radians(-5.0) + math.radians(-70.0 + offset) * (1.0 - float(self.right_recv_ee_positions[0])))
        self.dc.set_dof_position_target(
            self.right_ee_joint_handles[11],
            math.radians(-10.0) + math.radians(-85.0 + offset) * (1.0 - float(self.right_recv_ee_positions[0])))

        # Left
        offset = 20.0
        # Thumb 40,41,42,43
        self.dc.set_dof_position_target(self.left_ee_joint_handles[0], math.radians(90.0))
        self.dc.set_dof_position_target(
            self.left_ee_joint_handles[1],
            math.radians(-5.0) - math.radians(80. - offset) * (1.0 - float(self.left_recv_ee_positions[-2])))
        self.dc.set_dof_position_target(
            self.left_ee_joint_handles[2],
            math.radians(0.) - math.radians(10) * (1.0 - float(self.left_recv_ee_positions[-1])))
        self.dc.set_dof_position_target(self.left_ee_joint_handles[3], math.radians(0.0))
        # Index 00,01
        self.dc.set_dof_position_target(
            self.left_ee_joint_handles[4],
            math.radians(-5.0) + math.radians(-70.0 + offset) * (1.0 - float(self.left_recv_ee_positions[3])))
        self.dc.set_dof_position_target(
            self.left_ee_joint_handles[5],
            math.radians(-10.0) + math.radians(-85.0 + offset) * (1.0 - float(self.left_recv_ee_positions[3])))
        # Middle 10,11
        self.dc.set_dof_position_target(
            self.left_ee_joint_handles[6],
            math.radians(-5.0) + math.radians(-70.0 + offset) * (1.0 - float(self.left_recv_ee_positions[2])))
        self.dc.set_dof_position_target(
            self.left_ee_joint_handles[7],
            math.radians(-10.0) + math.radians(-85.0 + offset) * (1.0 - float(self.left_recv_ee_positions[2])))
        # Ring 20,21
        self.dc.set_dof_position_target(
            self.left_ee_joint_handles[8],
            math.radians(-5.0) + math.radians(-70.0 + offset) * (1.0 - float(self.left_recv_ee_positions[1])))
        self.dc.set_dof_position_target(
            self.left_ee_joint_handles[9],
            math.radians(-10.0) + math.radians(-85.0 + offset) * (1.0 - float(self.left_recv_ee_positions[1])))
        # Little 30,31
        self.dc.set_dof_position_target(
            self.left_ee_joint_handles[10],
            math.radians(-5.0) + math.radians(-70.0 + offset) * (1.0 - float(self.left_recv_ee_positions[0])))
        self.dc.set_dof_position_target(
            self.left_ee_joint_handles[11],
            math.radians(-10.0) + math.radians(-85.0 + offset) * (1.0 - float(self.left_recv_ee_positions[0])))
