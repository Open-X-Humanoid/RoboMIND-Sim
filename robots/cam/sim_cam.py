# isaac sim
import omni
# import omni.kit.app
from isaacsim.sensors.camera import Camera
from isaacsim.core.utils.viewports import set_camera_view
import omni.replicator.core as rep
# project
# from common.utils.transform_utils import StandaloneUtils
from common.logger_loader import logger
# others
# from scipy.spatial.transform import Rotation as R
# import cv2
import numpy as np

logger.info("sim cam imports all deps")


class SimCam:

    _ISAAC_SENSOR_CAMERA_OFFSET_WXYZ = np.array([0.5, 0.5, -0.5, -0.5], dtype=np.float64)
    @staticmethod
    def _quat_normalize_wxyz(q) -> np.ndarray:
        q = np.asarray(q, dtype=np.float64).reshape(4,)
        n = np.linalg.norm(q)
        if n <= 0:
            raise ValueError(f"invalid quaternion (norm=0): {q}")
        return q / n
    @staticmethod
    def _quat_mul_wxyz(a, b) -> np.ndarray:
        """Hamilton product, both in WXYZ order."""
        a = np.asarray(a, dtype=np.float64).reshape(4,)
        b = np.asarray(b, dtype=np.float64).reshape(4,)
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        return np.array([
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ], dtype=np.float64)
    @staticmethod
    def _quat_inv_wxyz(q) -> np.ndarray:
        """Inverse of quaternion in WXYZ order."""
        q = np.asarray(q, dtype=np.float64).reshape(4,)
        w, x, y, z = q
        n2 = w * w + x * x + y * y + z * z
        if n2 <= 0:
            raise ValueError(f"invalid quaternion (norm^2=0): {q}")
        return np.array([w, -x, -y, -z], dtype=np.float64) / n2
    @classmethod
    def cfg_wxyz_to_isaac_sensor_input_wxyz(cls, q_cfg_wxyz) -> np.ndarray:
        """
        quaternion in Isaac Sim UI  = cfg ”
        """
        q_cfg = cls._quat_normalize_wxyz(q_cfg_wxyz)
        q_offset = cls._quat_normalize_wxyz(cls._ISAAC_SENSOR_CAMERA_OFFSET_WXYZ)
        q_in = cls._quat_mul_wxyz(q_cfg, cls._quat_inv_wxyz(q_offset))
        return cls._quat_normalize_wxyz(q_in)


    def __init__(self,
                 resolution: tuple = (640, 480),
                 fake_resolution=None,
                 frequency: int = 30,
                 focal_length: float = 1.93,
                 clip_range: tuple = (0.01, 10.),
                 cam_name=None,
                 translation=None,
                 orientation=None,
                 camera_matrix=None,
                 camera_vector=None,
                 match_cfg_quat_to_isaac_ui: bool = False) -> None:
        """Create a fake camera in isaac sim
        Args:
            position (np.array, optional): position of the camera. Defaults to np.zeros(3).
            quat (np.array, optional): orientation of the robot, w last. Defaults to np.asarray([0., 0., 0., 1]).
            resolution (tuple, optional): size of the image. Defaults to (640, 480).
            frequency (int, optional): fps. Defaults to 30.
            camera_matrix (list, optional): camera intrinsic matrix. Defaults to [[958.8, 0.0, 957.8], [0.0, 956.7, 589.5], [0.0, 0.0, 1.0]].
            focal_length (float, optional): camera focal length in mm. Defaults to 1.93.
            clip_range (tuple, optional): clipping range in m. Defaults to (0.01, 10.).
        """
        self.translation = translation

        orientation_to_pass = orientation
        if orientation_to_pass is not None and match_cfg_quat_to_isaac_ui:
            try:
                # 仅支持 WXYZ 四元数：先归一化，再做固定偏置补偿
                q_cfg_wxyz = self._quat_normalize_wxyz(orientation_to_pass)
                orientation_to_pass = self.cfg_wxyz_to_isaac_sensor_input_wxyz(q_cfg_wxyz).tolist()
                logger.debug(
                    f"SimCam orientation compensation enabled. cfg_wxyz={q_cfg_wxyz.tolist()} -> camera_input_wxyz={orientation_to_pass}"
                )
            except Exception as e:
                logger.warning(f"SimCam failed to compensate orientation, fallback to raw orientation. err={e}")
                orientation_to_pass = orientation

        self.camera = Camera(prim_path=f"{cam_name}",
                             frequency=frequency,
                             resolution=resolution,
                             translation=translation,
                             orientation=orientation_to_pass)
        print("cam init")

        self.camera.initialize()
        width, height = resolution
        if fake_resolution is not None:
            width, height = fake_resolution
        if camera_matrix is not None:
            ((fx, _, cx), (_, fy, cy), (_, _, _)) = camera_matrix
        elif camera_vector is not None:
            (fx, fy, cx, cy) = camera_vector
        else:
            raise ValueError("must input intrinsics")
        print(fx, fy, cx, cy)

        aspect_ratio = height / width
        horizontal_aperture = width * focal_length / fx
        vertical_aperture = aspect_ratio * horizontal_aperture
        self.camera.set_focal_length(focal_length / 10.0)
        self.camera.set_horizontal_aperture(horizontal_aperture / 10.0)
        self.camera.set_vertical_aperture(vertical_aperture / 10.0)
        self.camera.set_clipping_range(clip_range[0], clip_range[1])
        self.cam_name = cam_name
        self.resolution = resolution
        self.init_depth()

    def init_depth(self):
        self.render_product = rep.create.render_product(f"{self.cam_name}", self.resolution)
        self.render_product_path = self.render_product.path
        self.depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        self.depth_annotator.attach(self.render_product_path)

    def get_rgb(self, to_bytes: bool = True, out_buffer=None):
        rgb_data = self.camera.get_rgb()
        if out_buffer is not None:
            # Ensure output buffer shape matches RGB data
            if out_buffer.shape == rgb_data.shape:
                np.copyto(out_buffer, rgb_data)
                return out_buffer
            else:
                logger.warning(f"RGB buffer shape mismatch: expected {rgb_data.shape}, actual {out_buffer.shape}")
        if to_bytes:
            return rgb_data.tobytes()
        return rgb_data
    
    def get_depth(self, out_buffer=None):
        depth_data = self.depth_annotator.get_data()*1000
        if out_buffer is not None:
            # Ensure output buffer shape matches depth data
            if out_buffer.shape == depth_data.shape:
                np.copyto(out_buffer, depth_data)
                return out_buffer
            else:
                logger.warning(f"Depth buffer shape mismatch: expected {depth_data.shape}, actual {out_buffer.shape}")
        return depth_data

    def set_cam_view(self, eye: np.array, target: np.array, viewport_api=None, is_random=False, scope=0.01) -> None:
        if is_random:
            random_eye = np.array([
                np.random.uniform(eye[0] - scope, eye[0] + scope),
                np.random.uniform(eye[1] - scope, eye[1] + scope),
                np.random.uniform(eye[2] - scope, eye[2] + scope)
            ])
            random_data = np.array([random_eye[0] - eye[0], random_eye[1] - eye[1], random_eye[2] - eye[2]])
            print("The random offset value is:", random_data)
            set_camera_view(eye=random_eye, target=target, camera_prim_path=self.cam_name, viewport_api=viewport_api)
        else:
            set_camera_view(eye=eye, target=target, camera_prim_path=self.cam_name, viewport_api=viewport_api)

   
