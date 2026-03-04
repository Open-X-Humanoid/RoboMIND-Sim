import sys
import os
# append to sys path to allow absolute project imports
project_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if project_path not in sys.path:
    sys.path.append(project_path)
######################################################################
# isaac sim
from isaacsim.simulation_app import SimulationApp
import omni
# project
from common.logger_loader import logger
from common.config_loader import config_loader
from common.utils.p2p_traj import P2P_Trajectory
from common.utils.zmq_utils import ZmqPublisher, ZmqReceiver
from common.utils.task_check import Task_Check
from tasks.task_base import TaskRunnerBase
# others
import numpy as np
import time
import json
import tqdm


class TienKung_Task_Base(TaskRunnerBase):
    """TienKung task base class providing common task behaviors for 01/02/03/04.

    Subclasses should:
    - Create and assign self.robot in their __init__ using Registry.create("TienKung2_Inspire", ...)
    - Implement task-specific check_success_callback
    - Optionally override init_play if they need custom warm-up sequences
    """

    def __init__(self,
                 simulation_app: SimulationApp,
                 physics_dt: float = 1.0 / 120,
                 render_dt: float = 1.0 / 120,
                 stage_units_in_meters: float = 1.00,
                 environment_path: str = None,
                 status_file_path: str = None):
        logger.debug("TienKung_Task_Base init")
        super().__init__(simulation_app=simulation_app,
                         physics_dt=physics_dt,
                         render_dt=render_dt,
                         stage_units_in_meters=stage_units_in_meters,
                         environment_path=environment_path)

        # Basic runtime states
        self.start_flag = False
        self.task_success_flag = False
        self.status_file_path = status_file_path
        self.current_step = 0
        # ZMQ client
        self.zmq_publisher = ZmqPublisher(port=5556)
        self.zmq_receiver = ZmqReceiver(port=5557)
        self.task_checker = Task_Check()
        self.l_arm_init_pose = config_loader.task_config["robot"]["l_arm_init_pose"]
        self.r_arm_init_pose = config_loader.task_config["robot"]["r_arm_init_pose"]
        self.l_arm_home_pose = config_loader.task_config["robot"]["l_arm_home_pose"]
        self.r_arm_home_pose = config_loader.task_config["robot"]["r_arm_home_pose"]
        logger.debug(f"l_arm_home_pose: {self.l_arm_home_pose}")
        logger.debug(f"r_arm_home_pose: {self.r_arm_home_pose}")
        logger.success('TienKung_Task_Base initialized')

    # -------------------------- Shared helpers --------------------------
    def write_task_status(self, completed=False, success=False, score=0.0, failure_reason=None):
        """Write task status to file for communication with task_manager"""
        if not self.status_file_path:
            return

        try:
            status_data = {
                'completed': completed,
                'success': success,
                'score': score,
                'completion_step': self.current_step,
                'failure_reason': failure_reason,
                'source': 'task_execution',
                'environment_usd_path': getattr(self, 'abs_environment_path', None)
            }

            with open(self.status_file_path, 'w') as f:
                json.dump(status_data, f, indent=2)

            logger.debug(f"Task status written: {status_data}")
        except Exception as e:
            logger.warning(f"Failed to write task status: {e}")

    def init_play(self, step_num: int):
        """Common warm-up and callback registration, then go to home pose."""
        for _ in tqdm.tqdm(range(step_num)):
            self.one_step()
        # Activate joints
        self.robot.active_art()
        logger.info("art activated")
        # Add callbacks
        self.physics_callback_dict = {
            'execute_joint': self.robot.joint_callback,
            'pub_joint': self.collect_data_callback,
            'check_success': self.check_success_callback,
            'update_joint': self.update_joint_callback,
        }
        for physics_callback_name, physics_callback_fn in self.physics_callback_dict.items():
            self.simulation_context.add_physics_callback(physics_callback_name, callback_fn=physics_callback_fn)
        self.to_arm_init_pose()
        for _ in tqdm.tqdm(range(5)):
            self.one_step()
        self.to_home_pose()
        logger.success("init play done")

    def update_joint_callback(self, step_size) -> None:
        if self.start_flag == True:
            # Allow empty receive: when no ZMQ message yet, do nothing instead of raising
            recv = self.zmq_receiver.receive_msg(timeout=10)
            if recv is None:
                return
            topic, data = recv
            if topic == b"action":
                self.robot.update_command(data)

    def collect_data_callback(self, step_size) -> None:
        self.sim_step += 1
        if self.sim_step % 4 == 0:
            curr_time = int(omni.timeline.get_timeline_interface().get_current_time() * 1000)
            self.buffer_pool_align["puppet"] = self.robot.pub_l_r_joints(curr_time)
            self.buffer_pool_align["camera_observations"]["timestamp"] = curr_time
            self.robot.cam_dict["camera_head"].get_rgb(
                out_buffer=self.buffer_pool_align["camera_observations"]["color_images"]["camera_head"])
            if self.start_flag == True:
                self.zmq_publisher.send_msg(data=self.buffer_pool_align, topic=b"obs")

    def check_success_callback(self, step_size) -> None:
        """Default implementation does nothing. Subclasses should override."""
        pass

    def to_home_pose(self):
        # pass
        left_arm_motion = []
        right_arm_motion = []
        time_duration = 2
        steps = int(time_duration / self.physics_dt)
        for idx in range(self.robot.num_arm_dof):
            left_arm_motion.append(
                P2P_Trajectory(start_p=self.l_arm_init_pose[idx],
                               end_p=self.l_arm_home_pose[idx],
                               start_v=0.0,
                               end_v=0.0,
                               start_a=0.0,
                               end_a=0.0,
                               start_t=0,
                               end_t=time_duration))
            right_arm_motion.append(
                P2P_Trajectory(start_p=self.r_arm_init_pose[idx],
                               end_p=self.r_arm_home_pose[idx],
                               start_v=0.0,
                               end_v=0.0,
                               start_a=0.0,
                               end_a=0.0,
                               start_t=0,
                               end_t=time_duration))

        for step in range(steps):
            cmd_left_positions = np.zeros(self.robot.num_arm_dof)
            cmd_right_positions = np.zeros(self.robot.num_arm_dof)
            for idx in range(self.robot.num_arm_dof):
                cmd_left_positions[idx] = left_arm_motion[idx].get_point(step * self.physics_dt)[0]
                cmd_right_positions[idx] = right_arm_motion[idx].get_point(step * self.physics_dt)[0]
            # Preserve current ee positions if available
            # left_hand = self.robot.left_recv_ee_positions if self.robot.left_recv_ee_positions is not None else 1.0
            # right_hand = self.robot.right_recv_ee_positions if self.robot.right_recv_ee_positions is not None else 1.0
            left_hand = config_loader.task_config["robot"]["l_hand_home_pose"]
            right_hand = config_loader.task_config["robot"]["r_hand_home_pose"]
            self.robot.update_command({
                "left_arm": cmd_left_positions,
                "right_arm": cmd_right_positions,
                "left_hand": left_hand,
                "right_hand": right_hand,
            })
            self.one_step()

    def play(self, num_steps: int = 60000) -> None:
        super().play()
        self.init_play(step_num=50)
        self.start_flag = True
        logger.debug("start sending obs")
        for i in range(num_steps):
            self.current_step = i
            self.one_step()
        self.stop()

    def start(self):
        logger.success("Attempt to start sim")
        for _ in range(10):
            self.zmq_publisher.send_msg(data=None, topic=b"test")
            time.sleep(0.1)
            recv_msg = self.zmq_receiver.receive_msg(timeout=10)
            if recv_msg is not None and recv_msg[0] == b"test":
                logger.info("Sim recv func warmed up")
        # add depth to annotator registry
        self.robot.cam_dict["camera_head"].init_depth()
        omni.timeline.get_timeline_interface().set_current_time(0)
        self.sim_step = 0
        self.zmq_publisher.send_msg(data=b"TienKung", topic=b"start")

    def stop(self):
        # Write final status if not already written (for timeout or failure cases)
        if not self.task_success_flag:
            self.write_task_status(completed=True,
                                   success=False,
                                   score=0.0,
                                   failure_reason="Task completed without success")

        self.zmq_publisher.send_msg(data=None, topic=b"reset")
        self.shut_down()

    def to_arm_init_pose(self):
        left_arm_motion = []
        right_arm_motion = []
        time_duration = 2
        steps = int(time_duration / self.physics_dt)
        for idx in range(self.robot.num_arm_dof):
            left_arm_motion.append(
                P2P_Trajectory(start_p=0.0,
                               end_p=self.l_arm_init_pose[idx],
                               start_v=0.0,
                               end_v=0.0,
                               start_a=0.0,
                               end_a=0.0,
                               start_t=0,
                               end_t=time_duration))
            right_arm_motion.append(
                P2P_Trajectory(start_p=0.0,
                               end_p=self.r_arm_init_pose[idx],
                               start_v=0.0,
                               end_v=0.0,
                               start_a=0.0,
                               end_a=0.0,
                               start_t=0,
                               end_t=time_duration))

        for step in range(steps):
            cmd_left_positions = np.zeros(self.robot.num_arm_dof)
            cmd_right_positions = np.zeros(self.robot.num_arm_dof)
            for idx in range(self.robot.num_arm_dof):
                cmd_left_positions[idx] = left_arm_motion[idx].get_point(step * self.physics_dt)[0]
                cmd_right_positions[idx] = right_arm_motion[idx].get_point(step * self.physics_dt)[0]
            # Preserve current ee positions if available
            # left_hand = self.robot.left_recv_ee_positions if self.robot.left_recv_ee_positions is not None else 1.0
            # right_hand = self.robot.right_recv_ee_positions if self.robot.right_recv_ee_positions is not None else 1.0
            self.robot.update_command({
                "left_arm": cmd_left_positions,
                "right_arm": cmd_right_positions,
                "left_hand": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                "right_hand": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            })
            self.one_step()
