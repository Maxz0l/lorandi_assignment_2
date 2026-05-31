#!/usr/bin/env python3
"""
cube_swapper_node — Orchestrates the UR5 pick-and-place swap using MoveIt!.

Waits for /cube_poses (PoseArray, 2 poses published by tag_detector_node), then
runs the full swap sequence in a background thread so the ROS 2 spin is never
blocked.

Swap algorithm
──────────────
  Given:  pos_A = original position of cube with tag_1  (red)
          pos_B = original position of cube with tag_10 (blue)
          pos_I = intermediate safe position (hardcoded, away from both cubes)

  1. pick(cube_A, pos_A)
  2. place(cube_A, pos_I)        ← park cube A safely
  3. pick(cube_B, pos_B)
  4. place(cube_B, pos_A)        ← cube B is now at A's original spot
  5. pick(cube_A, pos_I)
  6. place(cube_A, pos_B)        ← cube A is now at B's original spot
  7. go_home()

Geometry
────────
  Cube : 0.06 × 0.06 × 0.10 m  (L × W × H)
  Tag  : on the top face → tag_z ≈ top of cube
  Grasp point (tool0) : PRE_GRASP_Z above tag, then descend to GRASP_Z above tag.

MoveIt! groups (from SRDF ur_gripper.srdf)
──────────────────────────────────────────
  "ir_arm"     : 6-DOF arm, base_link → tool0
  "ir_gripper" : Robotiq 85, named states open/close

Topics:
  Subscribes : /cube_poses (geometry_msgs/PoseArray, TRANSIENT_LOCAL)
  Publishes  : /swap_done  (std_msgs/Bool) — signals color_detector to report
"""

import threading
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Bool

import moveit_commander
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import geometry_msgs.msg

# ── Geometry constants ────────────────────────────────────────────────────────
CUBE_HEIGHT = 0.10          # m — full cube height
PRE_GRASP_Z = 0.18          # m above tag surface before descending
GRASP_Z = 0.02              # m above tag surface when gripper closes
LIFT_Z = 0.18               # m above tag surface after picking

# Safe intermediate position in world frame (adjust if needed)
INTERMEDIATE = {"x": 0.3, "y": 0.4, "z": 0.30}

# Downward-facing orientation for tool0 (quaternion: 180° around X axis)
# Means the Z-axis of tool0 points downward into the table.
DOWNWARD_QUAT = geometry_msgs.msg.Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)


class CubeSwapperNode(Node):
    def __init__(self):
        super().__init__("cube_swapper")

        # ── MoveIt! initialisation ───────────────────────────────────────────
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm = MoveGroupCommander("ir_arm")
        self.gripper = MoveGroupCommander("ir_gripper")

        # Arm planning settings
        self.arm.set_max_velocity_scaling_factor(0.3)
        self.arm.set_max_acceleration_scaling_factor(0.3)
        self.arm.set_planning_time(10.0)
        self.arm.set_num_planning_attempts(5)

        # ── ROS 2 comms ──────────────────────────────────────────────────────
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sub_poses = self.create_subscription(
            PoseArray, "/cube_poses", self._on_cube_poses, latching_qos
        )
        self.pub_done = self.create_publisher(Bool, "/swap_done", 10)

        self._swap_started = False

        self.get_logger().info(
            "CubeSwapperNode ready — waiting for /cube_poses..."
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Callback
    # ──────────────────────────────────────────────────────────────────────────

    def _on_cube_poses(self, msg: PoseArray) -> None:
        """Triggered once when tag_detector_node publishes both cube poses."""
        if self._swap_started:
            return
        if len(msg.poses) < 2:
            self.get_logger().error("Expected 2 poses in /cube_poses, got fewer.")
            return

        self._swap_started = True
        pos_A = msg.poses[0]   # tag_1  (red cube)
        pos_B = msg.poses[1]   # tag_10 (blue cube)

        self.get_logger().info("Cube poses received — starting swap in background thread.")
        thread = threading.Thread(target=self._run_swap, args=(pos_A, pos_B), daemon=True)
        thread.start()

    # ──────────────────────────────────────────────────────────────────────────
    # Swap sequence (runs in background thread)
    # ──────────────────────────────────────────────────────────────────────────

    def _run_swap(self, pos_A: Pose, pos_B: Pose) -> None:
        try:
            pos_I = self._make_intermediate_pose()

            self.get_logger().info("── Step 1/6 : pick cube A (tag_1) ──")
            self._pick(pos_A)

            self.get_logger().info("── Step 2/6 : place cube A at intermediate ──")
            self._place(pos_I)

            self.get_logger().info("── Step 3/6 : pick cube B (tag_10) ──")
            self._pick(pos_B)

            self.get_logger().info("── Step 4/6 : place cube B at original pos A ──")
            self._place(pos_A)

            self.get_logger().info("── Step 5/6 : pick cube A from intermediate ──")
            self._pick(pos_I)

            self.get_logger().info("── Step 6/6 : place cube A at original pos B ──")
            self._place(pos_B)

            self.get_logger().info("── Returning to home position ──")
            self._go_home()

            self.get_logger().info("✅ Swap complete!")
            self.pub_done.publish(Bool(data=True))

        except Exception as e:
            self.get_logger().error(f"Swap failed: {e}")
            self._go_home()

    # ──────────────────────────────────────────────────────────────────────────
    # Motion primitives
    # ──────────────────────────────────────────────────────────────────────────

    def _pick(self, cube_top: Pose) -> None:
        """Pick a cube whose top surface center is at cube_top (world frame)."""
        self._open_gripper()

        # Pre-grasp : above the cube
        pre = self._pose_at_z(cube_top, cube_top.position.z + PRE_GRASP_Z)
        self._move_arm_to_pose(pre, label="pre-grasp")

        # Descend to grasp height
        grasp = self._pose_at_z(cube_top, cube_top.position.z + GRASP_Z)
        self._move_arm_to_pose(grasp, label="grasp descent")

        self._close_gripper()

        # Lift
        lift = self._pose_at_z(cube_top, cube_top.position.z + LIFT_Z)
        self._move_arm_to_pose(lift, label="lift")

    def _place(self, target_top: Pose) -> None:
        """Place (drop) the held cube so its top surface lands at target_top."""
        # Approach above target
        above = self._pose_at_z(target_top, target_top.position.z + PRE_GRASP_Z)
        self._move_arm_to_pose(above, label="place approach")

        # Descend to release height
        release = self._pose_at_z(target_top, target_top.position.z + GRASP_Z)
        self._move_arm_to_pose(release, label="place descent")

        self._open_gripper()

        # Retreat upward
        retreat = self._pose_at_z(target_top, target_top.position.z + PRE_GRASP_Z)
        self._move_arm_to_pose(retreat, label="place retreat")

    def _go_home(self) -> None:
        """Move arm to the 'home' named state defined in the SRDF."""
        self.arm.set_named_target("home")
        result = self.arm.go(wait=True)
        self.arm.stop()
        if not result:
            self.get_logger().warn("go_home: planning or execution failed.")

    def _open_gripper(self) -> None:
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)
        self.gripper.stop()
        self.get_logger().debug("Gripper opened.")

    def _close_gripper(self) -> None:
        self.gripper.set_named_target("close")
        self.gripper.go(wait=True)
        self.gripper.stop()
        self.get_logger().debug("Gripper closed.")

    def _move_arm_to_pose(self, pose: Pose, label: str = "") -> None:
        """Plans and executes a Cartesian move to the given pose."""
        target = PoseStamped()
        target.header.frame_id = "world"
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose = pose

        self.arm.set_pose_target(target)
        result = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

        if result:
            self.get_logger().info(f"Arm moved to '{label}' ✓")
        else:
            self.get_logger().warn(f"Arm move to '{label}' failed — continuing.")

    # ──────────────────────────────────────────────────────────────────────────
    # Geometry helpers
    # ──────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _pose_at_z(base: Pose, z: float) -> Pose:
        """Returns a new pose with the same XY and orientation as base, but at height z."""
        p = Pose()
        p.position.x = base.position.x
        p.position.y = base.position.y
        p.position.z = z
        p.orientation = DOWNWARD_QUAT   # tool0 always faces downward
        return p

    @staticmethod
    def _make_intermediate_pose() -> Pose:
        """Returns the fixed intermediate parking pose."""
        p = Pose()
        p.position.x = INTERMEDIATE["x"]
        p.position.y = INTERMEDIATE["y"]
        p.position.z = INTERMEDIATE["z"]
        p.orientation = DOWNWARD_QUAT
        return p


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CubeSwapperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        moveit_commander.roscpp_shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
