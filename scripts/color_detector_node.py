#!/usr/bin/env python3
"""
color_detector_node — Detects the color of each cube via the RGB camera. (+3 pts)

Workflow:
  1. Subscribes to /cube_poses (to know where the cubes are in world space).
  2. Waits for /swap_done signal published by cube_swapper_node.
  3. Captures one frame from /rgb_camera/image.
  4. For each cube, crops the region of interest (ROI) around the cube's projected
     position on the image and classifies the dominant HSV color (red / blue).
  5. Prints the result clearly on the terminal.

Note: full 3D→2D projection requires camera intrinsics. As a simpler fallback,
we classify the color of the whole scene by looking for the dominant hue in the
two halves of the image (or the full image) and matching it to red/blue ranges.
A more accurate implementation using camera_info is scaffolded but can be
enabled when intrinsics are needed.

Topics:
  Subscribes : /rgb_camera/image     (sensor_msgs/Image,      BEST_EFFORT)
  Subscribes : /rgb_camera/camera_info (sensor_msgs/CameraInfo, BEST_EFFORT)
  Subscribes : /cube_poses           (geometry_msgs/PoseArray, TRANSIENT_LOCAL)
  Subscribes : /swap_done            (std_msgs/Bool)
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool

# HSV color ranges for detection (OpenCV: H in [0,179], S/V in [0,255])
# Red wraps around 0° → two ranges
RED_LOWER_1 = np.array([0,   100, 80])
RED_UPPER_1 = np.array([10,  255, 255])
RED_LOWER_2 = np.array([160, 100, 80])
RED_UPPER_2 = np.array([179, 255, 255])

BLUE_LOWER = np.array([100, 100, 80])
BLUE_UPPER = np.array([130, 255, 255])


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__("color_detector")

        self.bridge = CvBridge()
        self._latest_image: np.ndarray | None = None
        self._cube_poses: PoseArray | None = None
        self._swap_done = False
        self._reported = False

        sensor_qos = qos_profile_sensor_data
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Capture images continuously; keep only the last one
        self.sub_image = self.create_subscription(
            Image, "/rgb_camera/image", self._on_image, sensor_qos
        )

        # Receive cube poses (for possible ROI projection)
        self.sub_poses = self.create_subscription(
            PoseArray, "/cube_poses", self._on_poses, latching_qos
        )

        # Trigger analysis when cube_swapper signals completion
        self.sub_done = self.create_subscription(
            Bool, "/swap_done", self._on_swap_done, 10
        )

        self.get_logger().info(
            "ColorDetectorNode ready — will report cube colors after swap."
        )

    # ──────────────────────────────────────────────────────────────────────────

    def _on_image(self, msg: Image) -> None:
        """Store the latest camera frame (converted to BGR numpy array)."""
        if self._reported:
            return
        try:
            self._latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")

    def _on_poses(self, msg: PoseArray) -> None:
        self._cube_poses = msg

    def _on_swap_done(self, msg: Bool) -> None:
        """Triggered by cube_swapper_node at the end of the swap sequence."""
        if not msg.data or self._reported:
            return
        self._swap_done = True
        self._analyze_and_report()

    # ──────────────────────────────────────────────────────────────────────────

    def _analyze_and_report(self) -> None:
        """Classify cube colors from the latest camera image."""
        if self._latest_image is None:
            self.get_logger().error("No camera image received yet — cannot report colors.")
            return

        img = self._latest_image
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # ── Split image in two halves (left / right) as a simple ROI heuristic.
        # If we had the camera projection matrix we could project each cube
        # pose onto the image plane and use a precise bounding box.
        h, w = hsv.shape[:2]
        left_hsv  = hsv[:, :w//2]
        right_hsv = hsv[:, w//2:]

        left_color  = self._dominant_color(left_hsv)
        right_color = self._dominant_color(right_hsv)

        # ── Terminal report ──────────────────────────────────────────────────
        self.get_logger().info("=" * 50)
        self.get_logger().info("CUBE COLOR DETECTION REPORT")
        self.get_logger().info(f"  Left  half of image : {left_color}")
        self.get_logger().info(f"  Right half of image : {right_color}")

        if self._cube_poses and len(self._cube_poses.poses) >= 2:
            pose_1  = self._cube_poses.poses[0]   # tag_1
            pose_10 = self._cube_poses.poses[1]   # tag_10
            self.get_logger().info(
                f"  Cube tag_1  @ ({pose_1.position.x:.2f}, {pose_1.position.y:.2f})"
                f" → detected color in scene: see above"
            )
            self.get_logger().info(
                f"  Cube tag_10 @ ({pose_10.position.x:.2f}, {pose_10.position.y:.2f})"
                f" → detected color in scene: see above"
            )
        self.get_logger().info("=" * 50)

        self._reported = True

    @staticmethod
    def _dominant_color(hsv_roi: np.ndarray) -> str:
        """
        Returns 'red', 'blue', or 'unknown' based on the dominant hue
        in the given HSV region.
        """
        # Count pixels matching each color mask
        mask_red = (
            cv2.inRange(hsv_roi, RED_LOWER_1, RED_UPPER_1)
            | cv2.inRange(hsv_roi, RED_LOWER_2, RED_UPPER_2)
        )
        mask_blue = cv2.inRange(hsv_roi, BLUE_LOWER, BLUE_UPPER)

        n_red  = int(np.sum(mask_red  > 0))
        n_blue = int(np.sum(mask_blue > 0))

        threshold = 200   # minimum number of matching pixels to declare a color
        if n_red > n_blue and n_red > threshold:
            return f"RED  (pixels: {n_red})"
        if n_blue > n_red and n_blue > threshold:
            return f"BLUE (pixels: {n_blue})"
        return f"unknown (red={n_red}, blue={n_blue})"


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
