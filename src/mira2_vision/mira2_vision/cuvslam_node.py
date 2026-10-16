"""Independent native cuVSLAM ROS estimator consuming registered RGB-D."""
from types import SimpleNamespace

import numpy as np
import rclpy
from cv_bridge import CvBridgeError
from utils.parameters import Setting, declare_config, choices, positive

from mira2_vision.rgbd_inputs import RGBDInputs
from mira2_vision.depth import decode_depth
from mira2_vision.cuvslam_odometry import CuVslamOdometry, IMU_NOISE


class CuVslamNode(RGBDInputs):
    def __init__(self):
        super().__init__('cuvslam_odometry')
        config = declare_config(self, {
            'odom_topic': '/vision/vo_odom', 'odom_frame': 'vo_odom',
            'base_frame': 'base_link', 'enable_slam': True,
            'tracking_mode': Setting('rgbd', choices('rgbd', 'multisensor')),
            'imu_topic': '', 'imu_frame': '',
            'imu_noise': {name: Setting(value, positive) for name, value in IMU_NOISE.items()},
        }, namespace='cuvslam')
        noise = {name: config.pop('imu_noise.' + name) for name in IMU_NOISE}
        self.estimator = CuVslamOdometry(self, max_depth=self.config['max_depth'], imu_noise=noise, **config)
        self.calibration = None

    def on_pair(self, rgb, depth):
        if not self._valid_pair(rgb, depth):
            return
        # Changing calibration mid-track would silently corrupt metric poses.
        calibration = (rgb.header.frame_id, rgb.width, rgb.height,
                       tuple(self.info.k), tuple(self.info.d), self.info.distortion_model)
        if self.calibration is not None and self.calibration != calibration:
            self.estimator._publish_lost(rgb.header)
            self.get_logger().error('cuVSLAM calibration changed; restart the estimator')
            return
        if self.info.distortion_model not in ('', 'plumb_bob', 'rational_polynomial'):
            self.get_logger().error('cuVSLAM requires pinhole/OpenCV polynomial calibration')
            return
        try:
            image = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
            metric = decode_depth(depth)
        except (ValueError, RuntimeError, CvBridgeError) as exc:
            self.get_logger().warn(f'Invalid RGB-D input: {exc}', throttle_duration_sec=5.0)
            return
        self.calibration = calibration
        self.estimator.process(SimpleNamespace(
            header=rgb.header, image=image, matrix=np.asarray(self.info.k).reshape(3, 3),
            dist_coeffs=self.info.d, depth=SimpleNamespace(image=metric)))

    def destroy_node(self):
        if hasattr(self, 'estimator'):
            self.estimator.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CuVslamNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
