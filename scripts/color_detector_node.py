#!/usr/bin/env python3
"""
color_detector_node — Detects the colour of each cube via the RGB camera. (+3 pts)

Activated AFTER the swap (waits for /swap_done published by cube_swapper_node), so
each cube is reported at its FINAL position. Per-cube detection works like this:

  1. The professor's apriltag_node keeps broadcasting a TF frame for every visible
     tag (tag_1, tag_10) at its CURRENT pose — so after the swap these frames sit on
     the cubes wherever they ended up.
  2. For each tag we take its 3D position, drop ~5 cm to aim at the cube BODY (the
     top face is covered by the black/white tag, not the cube colour), and project
     that 3D point onto the image plane using the camera intrinsics (camera_info) and
     the camera↔world transform (TF).
  3. We crop a small ROI around that pixel and classify the dominant HSV colour
     (red / blue). Black/white tag pixels don't match either range, so the coloured
     cube faces decide the result.
  4. We print one line per cube.

Topics:
  Subscribes : /rgb_camera/image       (sensor_msgs/Image,      sensor QoS)
  Subscribes : /rgb_camera/camera_info (sensor_msgs/CameraInfo, sensor QoS)
  Subscribes : /cube_poses             (geometry_msgs/PoseArray, TRANSIENT_LOCAL) — fallback
  Subscribes : /swap_done              (std_msgs/Bool,          TRANSIENT_LOCAL)
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data,
)

import cv2
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PointStamped for tf_buffer.transform

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, PointStamped
from std_msgs.msg import Bool

# HSV colour ranges (OpenCV: H in [0,179], S/V in [0,255]). Red wraps around 0°.
RED_LOWER_1 = np.array([0,   100, 80])
RED_UPPER_1 = np.array([10,  255, 255])
RED_LOWER_2 = np.array([160, 100, 80])
RED_UPPER_2 = np.array([179, 255, 255])
BLUE_LOWER  = np.array([100, 100, 80])
BLUE_UPPER  = np.array([130, 255, 255])

WORLD_FRAME = "world"
# (index in /cube_poses, tag TF frame). We do NOT assume which colour each tag is —
# the colour is what this node discovers from the camera and reports below.
CUBES = [
    (0, "tag_1"),
    (1, "tag_10"),
]

CUBE_HALF_HEIGHT = 0.05   # m, drop from the tag (cube top) to the cube body centre
ROI_HALF_PX      = 30     # px, half-size of the colour-sampling window around the cube
MIN_PIXELS       = 80     # min matching pixels to declare a colour

# Couleur du node dans le terminal (color_detector = magenta)
_MG = "\033[1;35m"
_RST = "\033[0m"


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__("color_detector")

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._latest_image: np.ndarray | None = None
        self._cam_frame: str | None = None
        self._fx = self._fy = self._cx = self._cy = None
        self._img_w = self._img_h = None
        self._cube_poses: PoseArray | None = None
        self._reported = False
        self._do_analyze = False

        sensor_qos = qos_profile_sensor_data
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.sub_image = self.create_subscription(
            Image, "/rgb_camera/image", self._on_image, sensor_qos
        )
        self.sub_info = self.create_subscription(
            CameraInfo, "/rgb_camera/camera_info", self._on_camera_info, sensor_qos
        )
        self.sub_poses = self.create_subscription(
            PoseArray, "/cube_poses", self._on_poses, latching_qos
        )
        self.sub_done = self.create_subscription(
            Bool, "/swap_done", self._on_swap_done, latching_qos
        )

        # Deferred analysis: the /swap_done callback just sets a flag; the timer does
        # the heavy work so we never block the executor inside a subscription.
        self.create_timer(0.3, self._check_analyze)

        self._say("ColorDetectorNode ready — will report each cube's colour after the swap.")

    def _say(self, msg: str) -> None:
        self.get_logger().info(f"{_MG}{msg}{_RST}")

    # ──────────────────────────────────────────────────────────────────────────

    def _on_image(self, msg: Image) -> None:
        if self._reported:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if msg.encoding in ("rgb8", "8UC3"):
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            self._latest_image = img
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._fx is not None:
            return
        k = msg.k  # row-major 3x3 intrinsic matrix
        self._fx, self._fy = k[0], k[4]
        self._cx, self._cy = k[2], k[5]
        self._cam_frame = msg.header.frame_id
        self._img_w, self._img_h = msg.width, msg.height
        self._say(
            f"Camera intrinsics received (frame '{self._cam_frame}', "
            f"{self._img_w}x{self._img_h}, fx={self._fx:.1f})."
        )

    def _on_poses(self, msg: PoseArray) -> None:
        self._cube_poses = msg

    def _on_swap_done(self, msg: Bool) -> None:
        if msg.data and not self._reported:
            self._do_analyze = True

    def _check_analyze(self) -> None:
        if self._do_analyze and not self._reported:
            self._do_analyze = False
            self._analyze_and_report()

    # ──────────────────────────────────────────────────────────────────────────

    def _tag_world_position(self, idx: int, frame: str):
        """Current world position of a tag: live TF first, /cube_poses as fallback."""
        try:
            tf = self.tf_buffer.lookup_transform(
                WORLD_FRAME, frame, rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
            t = tf.transform.translation
            return (t.x, t.y, t.z), "live TF (final position)"
        except tf2_ros.TransformException as e:
            if self._cube_poses and len(self._cube_poses.poses) > idx:
                p = self._cube_poses.poses[idx].position
                return (p.x, p.y, p.z), "fallback /cube_poses (original position)"
            self.get_logger().warn(f"No position for {frame}: {e}")
            return None, None

    def _project(self, world_xyz):
        """World point → image pixel (u, v) using TF (world→optical) + intrinsics."""
        pt = PointStamped()
        pt.header.frame_id = WORLD_FRAME
        pt.point.x, pt.point.y, pt.point.z = world_xyz
        try:
            pc = self.tf_buffer.transform(
                pt, self._cam_frame, timeout=Duration(seconds=1.0)
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"TF to camera frame failed: {e}")
            return None
        x, y, z = pc.point.x, pc.point.y, pc.point.z
        if z <= 1e-6:
            return None   # behind the camera
        u = self._fx * x / z + self._cx
        v = self._fy * y / z + self._cy
        return int(round(u)), int(round(v))

    def _classify_roi(self, u: int, v: int) -> str:
        """Dominant colour in a ROI centred on (u, v) of the latest image."""
        h, w = self._latest_image.shape[:2]
        x0, x1 = max(0, u - ROI_HALF_PX), min(w, u + ROI_HALF_PX)
        y0, y1 = max(0, v - ROI_HALF_PX), min(h, v + ROI_HALF_PX)
        if x1 <= x0 or y1 <= y0:
            return "out of frame"
        roi = self._latest_image[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_red = (cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1)
                    | cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2))
        mask_blue = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
        n_red, n_blue = int(np.sum(mask_red > 0)), int(np.sum(mask_blue > 0))
        if n_red > n_blue and n_red > MIN_PIXELS:
            return f"RED  (red={n_red}, blue={n_blue})"
        if n_blue > n_red and n_blue > MIN_PIXELS:
            return f"BLUE (red={n_red}, blue={n_blue})"
        return f"undetermined (red={n_red}, blue={n_blue})"

    def _analyze_and_report(self) -> None:
        if self._latest_image is None:
            self.get_logger().error("No camera image received yet — cannot report colours.")
            self._do_analyze = True   # retry on the next timer tick
            return
        if self._fx is None:
            self.get_logger().error("No camera_info yet — cannot project cube positions.")
            self._do_analyze = True
            return

        self._say("=" * 56)
        self._say("CUBE COLOUR REPORT (per cube, camera projection)")
        for idx, frame in CUBES:
            world_xyz, src = self._tag_world_position(idx, frame)
            if world_xyz is None:
                self._say(f"  {frame}: position not found.")
                continue
            # Aim at the cube body (below the tag) so we sample colour, not the tag.
            body = (world_xyz[0], world_xyz[1], world_xyz[2] - CUBE_HALF_HEIGHT)
            px = self._project(body)
            if px is None:
                self._say(f"  {frame}: projection failed.")
                continue
            colour = self._classify_roi(*px)
            self._say(
                f"  {frame} → detected colour: {colour}  "
                f"[px=({px[0]},{px[1]}), {src}]"
            )
        self._say("=" * 56)
        self._reported = True


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
