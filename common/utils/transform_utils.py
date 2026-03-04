import numpy as np
np.set_printoptions(precision=4)
import numpy.typing as npt
import transforms3d as t3d
from scipy.spatial.transform import Rotation
from isaacsim.core.prims import SingleXFormPrim

class StandaloneUtils(object):

    @staticmethod
    def get_xform_affines(xform: SingleXFormPrim, cam_type=None) -> npt.NDArray[np.float64]:
        """Get world pose affines matrix for the given xform.

        Args:
            xform: The xform prim.
                                  
        Returns:
            The affines matrix of shape (4, 4).
        """
        if cam_type:
            # xform is a cam
            pose = xform.get_world_pose(camera_axes=cam_type)
            # print("cam type", cam_type)
            # print("pose,", pose)
        else:
            pose = xform.get_world_pose()
        # xform position (in unit meter) and rotation (quaternions)
        # world_xyz, world_wxyz = world_pose[0]/100.0, world_pose[1]
        xyz, wxyz = pose[0], pose[1]

        world_rotation_matrix = t3d.quaternions.quat2mat(wxyz)
        world_affines = t3d.affines.compose(T=xyz, R=world_rotation_matrix, Z=np.ones(3), S=np.zeros(3))
        return world_affines

    @staticmethod
    def transform_xfroms_pose(xform: SingleXFormPrim, target_xform: SingleXFormPrim, xform_type=None,target_type=None) -> npt.NDArray[np.float64]:
        """Transform the pose of xform (for example, an object) to the target_xform (for example, camera).

        For example, transform the object pose from world frame to the camera frame, we only need to calculate
        $^{camera}T_{world} @ ^{world}T_{object-pose} = ^{camera}T_{object-pose}$,
        here $^{camera}T_{world} = ^{world}T_{camera}-1$.

        Args:
            xform: The source xform.
            target_xform: The target xform.

        Returns:
            The affines matrix, i.e., the representation of the origin of xform in the target_xform coordinate system.
        """

        if xform_type in ["ros", "world"]:
            xform_world_affines = StandaloneUtils.get_xform_affines(xform, xform_type)
            print(f"xform_world_affines,{xform_world_affines}")
        else:
            xform_world_affines = StandaloneUtils.get_xform_affines(xform)
        # print(f"xform_world_affines,{xform_world_affines}")
        if target_type in ["ros", "world"]:
            target_xform_world_affines = StandaloneUtils.get_xform_affines(target_xform, target_type)
            print(f"target_xform_world_affines,{target_xform_world_affines}")
        else:
            target_xform_world_affines = StandaloneUtils.get_xform_affines(target_xform)
        target_xform_world_affines_inv = np.linalg.inv(target_xform_world_affines)
        result_affine = np.matmul(target_xform_world_affines_inv, xform_world_affines)
        return np.asarray(result_affine, dtype=float)

    @staticmethod
    def affine_to_xyz_quaternion(affine_matrix):
        """
        Convert a 4x4 affine matrix to a 1D NumPy array [x, y, z, qx, qy, qz, qw].
        
        Args:
            affine_matrix (np.ndarray): 4x4 affine transformation matrix.
        
        Returns:
            np.ndarray: 1D array of shape (7,) containing translation (x, y, z) and quaternion (qx, qy, qz, qw).
        """
        # Ensure input is a 4x4 matrix
        if affine_matrix.shape != (4, 4):
            raise ValueError("Input matrix must be 4x4")
        
        # Extract translation (last column, first 3 rows)
        translation = affine_matrix[:3, 3]
        
        # Extract 3x3 rotation matrix
        rotation_matrix = affine_matrix[:3, :3]
        
        # Convert rotation matrix to quaternion using scipy
        rotation = Rotation.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()  # Returns [qx, qy, qz, qw]
        
        # Combine translation and quaternion into a single 1D array
        result = np.concatenate([translation, quaternion])
        
        return result
