import isaacsim.core.utils.bounds as bounds_utils
import omni
import numpy as np
import math
from pxr import Gf, Sdf, UsdPhysics, PhysxSchema ,Usd 

class Task_Check:

    def compute_aabb_for_prim(self, prim_path: str, cache=None):
        """Calculate the AABB in world coordinates based on the prim path
        
        Args:
            prim_path (str): USD Prim path of the object, e.g. "/env/object"
            cache: Optional AABB cache object
        Returns:
            np.ndarray: [min_x, min_y, min_z, max_x, max_y, max_z]
        """
        if prim_path is None or len(prim_path) == 0:
            raise ValueError("prim_path cannot be empty")
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if prim is None or not prim.IsValid():
            raise ValueError(f"Cannot find valid Prim: {prim_path}")
        if cache is None:
            cache = bounds_utils.create_bbox_cache()
        aabb = bounds_utils.compute_aabb(cache, prim_path=Sdf.Path(prim_path))
        if aabb is None or len(aabb) != 6:
            raise RuntimeError(f"Cannot compute AABB: {prim_path}")
        return np.array(aabb, dtype=float)

    def check_relative_position(self,
                                a_path: str,
                                b_path: str,
                                relation: str,
                                height_tolerance: float = 0.01,
                                inside_tolerance: np.ndarray = np.array([0.01, 0.01, 0.01, 0.06, 0.06, 0.06]),
                                overlap_threshold: float = 0.5) -> bool:
        """Determine the relative spatial relationship between object A and object B (based on world coordinate AABB).
        
        Supports two types of relationships:
        - "inside": Whether A is inside B (strictly contained, allowing inside_tolerance margin)
        - "on_top": Whether A is placed on top of B (A's bottom touches B's top, and XY projections have sufficient overlap)
        
        Args:
            a_path (str): Prim path of object A
            b_path (str): Prim path of object B
            relation (str): "inside" or "on_top"
            height_tolerance (float): Height threshold for top-bottom contact determination (meters)
            inside_tolerance (float): Tolerance for containment determination (meters)
            overlap_threshold (float): Threshold for the ratio of XY overlap area to A's bottom area
        Returns:
            bool: Returns True if condition is met, otherwise False
        """
        relation = relation.lower().strip()
        cache = bounds_utils.create_bbox_cache()
        a_aabb = self.compute_aabb_for_prim(a_path, cache)
        # print("a_aabb",a_aabb)
        b_aabb = self.compute_aabb_for_prim(b_path, cache)

        a_min = a_aabb[:3]
        a_max = a_aabb[3:]
        b_min = b_aabb[:3]
        b_max = b_aabb[3:]

        if relation == "inside":
            # A inside B: AABB strictly contained within BABB (considering tolerance)
            cond_min = np.all(a_min >= (b_min - inside_tolerance[:3])) # Considering penetration, so subtract tolerance
            cond_max = np.all(a_max <= (b_max + inside_tolerance[3:])) # Considering A is placed inside B but has protruding parts
            return bool(cond_min and cond_max)

        if relation == "on_top":
            # Vertical contact: A's bottom close to B's top
            a_bottom = a_min[2]
            b_top = b_max[2]
            vertical_contact = abs(a_bottom - b_top) <= height_tolerance

            # XY plane overlap (determine "placed on top" by area ratio)
            a_len_x = max(0.0, a_max[0] - a_min[0])
            a_len_y = max(0.0, a_max[1] - a_min[1])
            a_area = a_len_x * a_len_y
            if a_area <= 0:
                return False

            overlap_x = max(0.0, min(a_max[0], b_max[0]) - max(a_min[0], b_min[0]))
            overlap_y = max(0.0, min(a_max[1], b_max[1]) - max(a_min[1], b_min[1]))
            overlap_area = overlap_x * overlap_y
            overlap_ratio = overlap_area / a_area

            return bool(vertical_contact and (overlap_ratio >= overlap_threshold))

        raise ValueError("relation only supports 'inside' or 'on_top'")

    def get_upright_state(self,pose, up_angle_threshold_deg: float = 20.0, flat_angle_threshold_deg: float = 70.0) -> str:
        """Determine the upright state of an object (upright/tilted/fallen) based on pose.
        
        Basis: The angle between the object's local Z-axis and the world Z-axis.
        - Upright: angle <= up_angle_threshold_deg
        - Tilted: up_angle_threshold_deg < angle < flat_angle_threshold_deg
        - Fallen: angle >= flat_angle_threshold_deg
        
        Args:
            pose: Pose tuple (position(np.ndarray[3]), orientation(np.ndarray[4])) obtained from get
            up_angle_threshold_deg (float): Upright angle threshold
            flat_angle_threshold_deg (float): Fallen angle threshold
        Returns:
            str: 'upright' | 'tilted' | 'fallen'
        """
        if pose is None or len(pose) != 2:
            raise ValueError("pose should be a (position, orientation) tuple")
        _, quat = pose

        # Calculate the direction of the object's local Z-axis in the world frame (get_world_pose() returns [w, x, y, z])
        w, x, y, z = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
        orientation = Gf.Quatf(w, x, y, z)
        rot_mat_gf = Gf.Matrix3f(orientation)
        rot_mat = np.array([[rot_mat_gf[0][0], rot_mat_gf[0][1], rot_mat_gf[0][2]],
                            [rot_mat_gf[1][0], rot_mat_gf[1][1], rot_mat_gf[1][2]],
                            [rot_mat_gf[2][0], rot_mat_gf[2][1], rot_mat_gf[2][2]]], dtype=float)
        local_z = np.array([0.0, 0.0, 1.0], dtype=float)
        world_z_dir = rot_mat @ local_z

        # Angle with world Z-axis [0,0,1]
        world_up = np.array([0.0, 0.0, 1.0], dtype=float)
        dot_val = float(np.clip(np.dot(world_z_dir / (np.linalg.norm(world_z_dir) + 1e-8), world_up), -1.0, 1.0))
        angle_deg = float(np.degrees(np.arccos(dot_val)))

        if angle_deg <= up_angle_threshold_deg:
            return 'upright'
        if angle_deg >= flat_angle_threshold_deg:
            return 'fallen'
        return 'tilted'


    def get_z_axis_angle(self, prim) -> float:
        """Compute the object's rotation angle about the world Z-axis (yaw) in degrees.

        Uses prim.get_world_pose() to obtain (position, quaternion). The quaternion is
        assumed to be ordered as [w, x, y, z], consistent with Isaac Sim conventions
        used elsewhere in this file.

        Args:
            prim: An object providing get_world_pose() -> (position, quaternion)

        Returns:
            float: Yaw angle in degrees in the range [-180, 180].
        """
        if prim is None or not hasattr(prim, "get_world_pose"):
            raise ValueError("prim must provide get_world_pose()")

        _, quat = prim.get_world_pose()

        # Convert quaternion [w, x, y, z] to rotation matrix via pxr.Gf
        w, x, y, z = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
        orientation = Gf.Quatf(w, x, y, z)
        rot_mat_gf = Gf.Matrix3f(orientation)

        # Yaw about world Z from rotation matrix
        # atan2(r21, r11) using row-major indexing
        yaw_rad = math.atan2(rot_mat_gf[1][0], rot_mat_gf[0][0])
        yaw_deg = math.degrees(yaw_rad)
        return float(yaw_deg)


    def has_upright_state_changed(self,
                                initial_pose,
                                current_pose,
                                up_angle_threshold_deg: float = 20.0,
                                flat_angle_threshold_deg: float = 70.0) -> bool:
        """Determine whether the "upright state" of an object has changed relative to its initial pose.
        
        Based on the angle between the object's local Z-axis and the world Z-axis, the state is classified as: upright, tilted, or fallen.
        When the initial state differs from the current state classification, it is considered a state change.
        
        Args:
            initial_pose: Initial pose (pos, quat)
            current_pose: Current pose (pos, quat)
            up_angle_threshold_deg (float): Upright angle threshold
            flat_angle_threshold_deg (float): Fallen angle threshold
        Returns:
            bool: Whether the state has changed
        """
        init_state = self.get_upright_state(initial_pose, up_angle_threshold_deg, flat_angle_threshold_deg)
        curr_state = self.get_upright_state(current_pose, up_angle_threshold_deg, flat_angle_threshold_deg)
        return bool(init_state != curr_state)

    def get_joint_state(self,joint_prim: Usd.Prim) -> tuple[float, float]:
        """Get the current joint state (position, velocity) from a joint prim.

        Reads from the PhysxSchema.JointStateAPI. If the API or its attributes
        are not found, returns (0.0, 0.0).

        Args:
            joint_prim: The USD joint prim (Revolute or Prismatic).

        Returns:
            A tuple containing the (position, velocity).
        """
        position = 0.0
        velocity = 0.0

        if not joint_prim or not joint_prim.IsValid():
            return position, velocity

        # Determine the appropriate drive type name
        drive_type = None
        if joint_prim.IsA(UsdPhysics.RevoluteJoint):
            drive_type = "angular"
        elif joint_prim.IsA(UsdPhysics.PrismaticJoint):
            drive_type = "linear"
        else:
            # Not a supported joint type for state reading
            return position, velocity

        # Check if the joint state API exists
        if joint_prim.HasAPI(PhysxSchema.JointStateAPI, drive_type):
            joint_state_api = PhysxSchema.JointStateAPI.Get(joint_prim, drive_type)
            if joint_state_api:
                pos_attr = joint_state_api.GetPositionAttr()
                if pos_attr:
                    pos_val = pos_attr.Get()
                    if pos_val is not None:
                        position = pos_val

                vel_attr = joint_state_api.GetVelocityAttr()
                if vel_attr:
                    vel_val = vel_attr.Get()
                    if vel_val is not None:
                        velocity = vel_val

        return position, velocity






