#!/usr/bin/env python3
"""
tag_detector_node — Converts AprilTag detections from camera frame to world frame.

The apriltag_ros C++ node (professor's code) already broadcasts a TF frame for each
detected tag (named 'tag_1' and 'tag_10' as configured in apriltag_params.yaml).
This node waits until both frames are visible, then publishes a PoseArray on
/cube_poses with the two cube top-surface positions in the 'world' frame.

Topic layout:
  Subscribes : /detections  (apriltag_msgs/AprilTagDetectionArray)
  Publishes  : /cube_poses  (geometry_msgs/PoseArray, TRANSIENT_LOCAL QoS)
                index 0 = tag_1  (red cube)
                index 1 = tag_10 (blue cube)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy

import tf2_ros
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, PoseArray

TAG_IDS = [1, 10]
TAG_FRAMES = {1: "tag_1", 10: "tag_10"}
WORLD_FRAME = "world"

# Couleur du node dans le terminal (tag_detector = vert)
_GR = "\033[1;32m"
_RST = "\033[0m"

# ── Calibration empirique de la détection AprilTag (PAR TAG) ─────────────────
# L'estimation de pose PnP du tag a un biais systématique qui dépend de la
# position du cube vis-à-vis de la caméra (fixe) : il n'est donc PAS identique
# pour les deux cubes. Un offset commun laissait le cube rouge ~3 mm décentré,
# ce qui suffit à rater la prise car la pince Robotiq se ferme de façon
# asymétrique en Gazebo (seul le knuckle gauche est actionné, le droit ne suit
# pas le mimic). On applique donc un offset PROPRE À CHAQUE TAG, mesuré le
# 2026-06-01 contre la vérité-terrain Gazebo, pour centrer chaque cube à <1 mm.
# (frame du tag = face supérieure du cube ; cube de 0.10 m de haut.)
#   valeur = vrai sommet du cube − pose détectée  → (dx, dy, dz)
TAG_CALIB = {
    1:  (0.0299, 0.0054, -0.0163),   # tag_1  (cube rouge)  @ table1
    10: (0.0357, 0.0093, -0.0145),   # tag_10 (cube bleu)   @ table2
}


class TagDetectorNode(Node):
    def __init__(self):
        super().__init__("tag_detector")

        # TF2 buffer + listener : reçoit en continu l'arbre de transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS "latchant" : le dernier message publié est rejoué pour tout
        # nouveau subscriber → le cube_swapper reçoit les poses même s'il
        # démarre après ce node.
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # On souscrit aux détections pour savoir quand les tags sont visibles.
        # Le QoS sensor_data (BEST_EFFORT) est utilisé car la caméra publie
        # avec ce profil ; une incompatibilité de QoS donnerait zéro message.
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sub_detections = self.create_subscription(
            AprilTagDetectionArray,
            "/detections",
            self._on_detections,
            sensor_qos,
        )

        self.pub_poses = self.create_publisher(PoseArray, "/cube_poses", latching_qos)

        # Stockage des poses monde par tag_id (rempli au fur et à mesure)
        self._detected: dict = {}
        self._done = False  # publie une seule fois

        self._say("TagDetectorNode ready — waiting for tag_1 and tag_10 in the TF tree…")

    def _say(self, msg: str) -> None:
        self.get_logger().info(f"{_GR}{msg}{_RST}")

    # ──────────────────────────────────────────────────────────────────────────

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        """Called each time apriltag_ros publishes a detection array."""
        if self._done:
            return

        for detection in msg.detections:
            tag_id = detection.id
            if tag_id not in TAG_IDS or tag_id in self._detected:
                continue
            self._try_lookup(tag_id)

        if len(self._detected) == len(TAG_IDS):
            self._publish_poses()

    def _try_lookup(self, tag_id: int) -> None:
        """
        Tries to get the transform world → tag_frame.
        The tag frame origin is at the CENTER of the tag = top surface of cube.
        """
        frame = TAG_FRAMES[tag_id]
        try:
            tf = self.tf_buffer.lookup_transform(
                WORLD_FRAME,
                frame,
                rclpy.time.Time(),       # dernière transform connue
            )
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"TF lookup failed ({frame}): {e}")
            return
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(f"TF connectivity ({frame}): {e}")
            return
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f"TF extrapolation ({frame}): {e}")
            return

        dx, dy, dz = TAG_CALIB.get(tag_id, (0.0, 0.0, 0.0))
        pose = Pose()
        pose.position.x = tf.transform.translation.x + dx
        pose.position.y = tf.transform.translation.y + dy
        pose.position.z = tf.transform.translation.z + dz
        pose.orientation = tf.transform.rotation

        self._detected[tag_id] = pose
        self._say(
            f"Tag {tag_id} located (world, calibrated): "
            f"x={pose.position.x:.3f}  y={pose.position.y:.3f}  z={pose.position.z:.3f}"
        )

    def _publish_poses(self) -> None:
        """Publishes both cube poses in a fixed order (tag_1 first, tag_10 second)."""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = WORLD_FRAME

        for tag_id in TAG_IDS:          # ordre fixe : index 0 = tag_1, index 1 = tag_10
            msg.poses.append(self._detected[tag_id])

        self.pub_poses.publish(msg)
        self._done = True
        self._say("Both cubes located → /cube_poses published → cube_swapper.")


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TagDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
