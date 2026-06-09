#!/usr/bin/env python3
"""
cube_swapper_node — Orchestrates the UR5 pick-and-place swap via MoveGroup action client.

Uses the standard ROS 2 action client interface to the running move_group server:
  /move_action          (moveit_msgs/action/MoveGroup)    — arm planning + execution
  /gripper_controller/gripper_cmd (control_msgs/action/GripperCommand) — gripper

This avoids the moveit_py / moveit_commander internal-node config issues and
connects directly to the already-configured move_group server (launched by the
professor's ir_movit.launch.py).

Swap workflow (2 pick-and-place moves, no buffer):
  Cube A (tag_1) starts on table1, cube B (tag_10) starts on table2. We do NOT assume
  the cubes' colours here — that is the colour_detector's job; this node just swaps the
  two detected cubes. Both original spots are occupied, so A goes to a free offset spot
  on table2:
    1. pick A @ table1  → place A on table2 (offset spot, clears B)
    2. pick B @ table2  → place B on table1 (A's now-empty original spot)
    3. go home
  Result: A ends on table2, B ends on table1 → cubes swapped.

Topics:
  Subscribes : /cube_poses  (geometry_msgs/PoseArray, TRANSIENT_LOCAL)
  Publishes  : /swap_done   (std_msgs/Bool)
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy

import rclpy.duration
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header, Float64MultiArray

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints,
    JointConstraint, MoveItErrorCodes,
    WorkspaceParameters,
    PlanningScene, AttachedCollisionObject, CollisionObject,
)
from moveit_msgs.srv import GetPositionIK, GetCartesianPath, ApplyPlanningScene
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import Vector3
from shape_msgs.msg import SolidPrimitive

# ── Geometry constants ────────────────────────────────────────────────────────
# IK targets are expressed for `tool0` (the wrist flange), but the part that
# actually touches the cube is the gripper fingertip, ~0.16 m further along
# tool0's +z axis. Every Cartesian target therefore adds TOOL0_TO_TCP so the
# *fingertips* — not the flange — reach the intended height. Without this the
# flange is placed at the tag and the fingers are driven ~0.16 m into the table
# (IK returns NO_SOLUTION with avoid_collisions=True → the pick silently fails).
# Verify/tune the length with:
#   ros2 run tf2_ros tf2_echo tool0 robotiq_85_left_finger_tip_link
TOOL0_TO_TCP = 0.16   # m, tool0 → fingertip contact (Robotiq 85 + adapter)

# Real grasp. The AprilTag sits on the cube's TOP face and the cube is 0.10 m tall
# (centre 0.05 m below the top). /cube_poses now reports the TRUE cube top (the
# tag_detector applies an empirical calibration). To actually grip, the open fingers
# (≈85 mm) must straddle the 60 mm cube BODY, so the TCP descends BELOW the top by
# |GRASP_CLEARANCE| before closing. Centring is corrected upstream so the open fingers
# clear the cube during the descent (no side contact → no 0.2 rad path-tolerance abort).
GRASP_CLEARANCE    = -0.04  # m: fingertips 4 cm BELOW the cube top → straddle the body
APPROACH_CLEARANCE = 0.03   # m above the cube for approach / retreat
# When PLACING, we release the cube with a small clearance ABOVE the table so it settles
# by gravity — we never press it down. Reasoning: our manual height assumes the cube is
# held exactly per the model, but a 2 kg cube sags/slips a little in the grip, so aiming
# at the exact surface effectively presses it too low → it loses balance / topples (and
# pressing into the rigid table makes the controller abort, per ros2_control docs). A
# few-mm release gap is robust to that sag and to z-calibration error.
PLACE_CLEARANCE    = 0.01   # m: cube released ~5 cm above table (drops flat). Tunable: more
                            # negative = lower release, more positive = higher (≤ ~+0.04 before
                            # the arm hits its reach ceiling and leaves no room for the Z retreat).

PRE_GRASP_Z_OFFSET = TOOL0_TO_TCP + APPROACH_CLEARANCE   # 0.19 → tool0 above the cube (reachable)
GRASP_Z_OFFSET     = TOOL0_TO_TCP + GRASP_CLEARANCE      # 0.12 → fingertips 4 cm into the body
PLACE_Z_OFFSET     = TOOL0_TO_TCP + PLACE_CLEARANCE      # 0.17 → cube released ~5 cm above table
LIFT_Z_OFFSET      = TOOL0_TO_TCP + APPROACH_CLEARANCE   # 0.19

# Cube A's final drop spot on table2: an offset from the DETECTED cube B so it lands on
# the table surface, clear of B (which is still there during move 1). 2-move swap → A
# ends here, near B's old spot. Probed reachable; tune if it clips the table edge.
CUBE_A_DROP_DX = -0.14   # m, toward the base in x (farther from B so OMPL has room to place)
CUBE_A_DROP_DY = -0.10   # m, in -y (clears cube B, stays on table2)

EEF_LINK    = "tool0"
WORLD_FRAME = "world"

# Downward orientation for tool0 (180° around X)
DOWNWARD_QUAT = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

# Home joint positions (from SRDF 'home' named state)
HOME_JOINTS = {
    "shoulder_pan_joint":  0.0,
    "shoulder_lift_joint": -1.5707963,
    "elbow_joint":          0.0,
    "wrist_1_joint":       -1.5707963,
    "wrist_2_joint":        0.0,
    "wrist_3_joint":        0.0,
}

GRIPPER_OPEN    = 0.0   # robotiq_85_left_knuckle_joint fully open (gap ≈ 85 mm) — for picking
GRIPPER_RELEASE = 0.0   # release = fully open (gap ≈ 85 mm). A partial 0.08 (≈76 mm) left only
                        # ~8 mm clearance per finger and the 60 mm cube stuck to the fingers via
                        # Gazebo friction (cube B was carried away instead of dropped). Full open
                        # guarantees loss of contact; the speed-limited knuckle won't fling it, and
                        # at 85 mm the fingers don't touch a centred 60 mm cube → no topple.
GRIPPER_CLOSE = 0.62   # firm squeeze on the 2 kg / 60 mm cube (friction-only hold in Gazebo);
                       # 0.8 (full close) crushes/ejects it. Raise if the cube still slips.

# The joint_trajectory_controller reports the goal reached as soon as it is within
# the (loose, 0.2 rad) path tolerance — the arm can still be settling when the next
# motion starts, leaving the gripper slightly tilted/off-centre over the cube. We let
# it come to rest BEFORE the vertical descent so the grasp is repeatable, not random.
SETTLE_TIME = 2.0      # s, dwell after reaching a pre-grasp/approach pose.
# The prof's controller has a loose 0.1 rad goal tolerance, so the trajectory "ends"
# while the arm is still up to 0.1 rad from target → the XY landing over the cube
# varies run-to-run. After the trajectory completes the controller HOLDS the setpoint
# and the arm keeps converging, so a generous dwell lets it settle to the SAME
# repeatable steady-state every run (the residual is then absorbed by the open gripper).

# ── Planning-scene attach (cube follows the gripper in RViz) ──────────────────
CUBE_DIMENSIONS         = [0.06, 0.06, 0.10]      # m, real cube size (used for the attached object)
# Inflated boxes used ONLY for the cubes as WORLD obstacles: OMPL keeps the carried cube
# this far from the other cube. Bigger than the real cube → extra clearance that absorbs
# the controller's 0.2 rad path deviation, so the carried cube can't graze the other one
# (we can't lift over it: the reach ceiling keeps transport at the other cube's height).
OBSTACLE_DIMENSIONS     = [0.09, 0.09, 0.10]      # m (12 cm rendait la dépose de A près de B
                                                  # impossible à planifier → 9 cm : marge + plannable)
CUBE_CENTER_BELOW_TOOL0 = TOOL0_TO_TCP + 0.05 + GRASP_CLEARANCE   # 0.17 m: keeps the attached box centred on the real cube
GRIPPER_TOUCH_LINKS = [
    "robotiq_85_base_link",
    "robotiq_85_left_finger_link",        "robotiq_85_right_finger_link",
    "robotiq_85_left_finger_tip_link",    "robotiq_85_right_finger_tip_link",
    "robotiq_85_left_inner_knuckle_link", "robotiq_85_right_inner_knuckle_link",
    "robotiq_85_left_knuckle_link",       "robotiq_85_right_knuckle_link",
]

# ── Couleur du node dans le terminal (cube_swapper = cyan) ───────────────────
_CY  = "\033[1;36m"   # cyan gras
_DIM = "\033[2;36m"   # cyan atténué (sous-étapes)
_RST = "\033[0m"


class CubeSwapperNode(Node):
    def __init__(self):
        super().__init__("cube_swapper")

        # ── Action clients ───────────────────────────────────────────────────
        self._arm_client = ActionClient(self, MoveGroup, "/move_action")
        # ── Gripper publisher (JointGroupPositionController) ─────────────────
        # GripperActionController absent sur Jazzy → publisher direct sur
        # /gripper_position_controller/commands (Float64MultiArray).
        self._gripper_pub = self.create_publisher(
            Float64MultiArray, "/gripper_position_controller/commands", 10
        )
        # ── IK service ───────────────────────────────────────────────────────
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        # ── Cartesian straight-line moves (clean vertical descent/lift) ───────
        self._cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._exec_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        # ── Planning scene (attach/detach the grasped cube → correct RViz swap) ──
        self._scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._servers_ready = False
        self.create_timer(0.5, self._check_servers)

        # ── Subscription to cube poses ────────────────────────────────────────
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sub_poses = self.create_subscription(
            PoseArray, "/cube_poses", self._on_cube_poses, latching_qos
        )
        self.pub_done = self.create_publisher(Bool, "/swap_done", latching_qos)

        # ── Live arm configuration (to seed IK from the CURRENT pose) ─────────
        # Seeding /compute_ik with the arm's real joints (not a fixed 'home' seed)
        # makes KDL return the IK branch CLOSEST to where the arm actually is →
        # no elbow-flip / swing-around, and the post-place vertical retreat stays
        # feasible. /joint_states is published RELIABLE by joint_state_broadcaster.
        self._latest_joints = None   # dict joint→pos, last full arm snapshot
        self.sub_joints = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10
        )

        self._swap_started = False
        self._pending_poses = None   # cached (pos_A, pos_B) until the arm server is ready
        self._say("CubeSwapperNode ready — waiting for MoveIt action servers…")

    # ── Logs colorés (cube_swapper = cyan) ──────────────────────────────────
    def _say(self, msg: str) -> None:
        """Message de workflow bien visible."""
        self.get_logger().info(f"{_CY}{msg}{_RST}")

    def _step(self, msg: str) -> None:
        """Sous-étape / 'qui appelle qui' (atténué)."""
        self.get_logger().info(f"{_DIM}    ↳ {msg}{_RST}")

    def _check_servers(self) -> None:
        """Periodic timer: non-blocking poll for arm action server."""
        if self._servers_ready:
            return
        if self._arm_client.server_is_ready():
            self._servers_ready = True
            self._say("MoveIt server ready (arm operational).")
            # /cube_poses is latched (TRANSIENT_LOCAL): it may already have been
            # received and cached before the server was ready — try to start now.
            self._maybe_start_swap()

    # ──────────────────────────────────────────────────────────────────────────

    def _on_cube_poses(self, msg: PoseArray) -> None:
        if self._swap_started or len(msg.poses) < 2:
            return
        for pose in msg.poses[:2]:
            if any(math.isnan(v) or math.isinf(v)
                   for v in (pose.position.x, pose.position.y, pose.position.z)):
                self.get_logger().error("Received nan/inf in cube pose — aborting swap.")
                return
        # Cache the poses and try to start. The swap only launches once the arm
        # server is ALSO ready — this avoids dropping the single latched message
        # when it arrives before the server (startup race).
        self._pending_poses = (msg.poses[0], msg.poses[1])
        self._say("Received /cube_poses (2 cubes) ← tag_detector.")
        self._maybe_start_swap()

    def _on_joint_states(self, msg: JointState) -> None:
        """Cache the latest FULL arm joint snapshot (used to seed IK)."""
        arm = {n: p for n, p in zip(msg.name, msg.position) if n in HOME_JOINTS}
        if len(arm) == len(HOME_JOINTS):
            # Atomic reference swap (never mutate in place) → safe to read from
            # the swap worker thread under the MultiThreadedExecutor.
            self._latest_joints = arm

    def _maybe_start_swap(self) -> None:
        """Launch the swap exactly once, when both poses and arm server are ready.

        Both callers (_check_servers timer and _on_cube_poses subscription) run in
        the node's default mutually-exclusive callback group, so no lock is needed.
        """
        if self._swap_started or not self._servers_ready or self._pending_poses is None:
            return
        self._swap_started = True
        pos_A, pos_B = self._pending_poses
        threading.Thread(
            target=self._run_swap, args=(pos_A, pos_B), daemon=True,
        ).start()

    # ──────────────────────────────────────────────────────────────────────────
    # Swap sequence
    # ──────────────────────────────────────────────────────────────────────────

    def _run_swap(self, cube_a_pose: Pose, cube_b_pose: Pose) -> None:
        """2-move swap (no buffer round-trip): A → a free spot on table2, B → A's spot.

        cube_a_pose = /cube_poses[0] (tag_1), cube_b_pose = /cube_poses[1] (tag_10).
        Colour-agnostic: this node never assumes which cube is red or blue.

          1. A (table1) → free offset spot on table2 (clear of B, which is still there)
          2. B (table2) → A's exact original spot (now empty)
        A ends NEAR B's old spot (offset), not exactly on it — that's fine for the swap.
        """
        try:
            # A's destination: a free offset spot on table2, derived from B's detected pose
            # so it lands on the table surface and clears B (still present in move 1).
            cube_a_dest = Pose()
            cube_a_dest.position.x  = cube_b_pose.position.x + CUBE_A_DROP_DX
            cube_a_dest.position.y  = cube_b_pose.position.y + CUBE_A_DROP_DY
            cube_a_dest.position.z  = cube_b_pose.position.z
            cube_a_dest.orientation = DOWNWARD_QUAT

            # B's destination: A's exact original spot on table1 (empty once A has left).
            cube_b_dest = Pose()
            cube_b_dest.position.x  = cube_a_pose.position.x
            cube_b_dest.position.y  = cube_a_pose.position.y
            cube_b_dest.position.z  = cube_a_pose.position.z
            cube_b_dest.orientation = DOWNWARD_QUAT

            self._say(
                f"Plan: A(tag_1) {self._xy(cube_a_pose)} → {self._xy(cube_a_dest)} (table2) | "
                f"B(tag_10) {self._xy(cube_b_pose)} → {self._xy(cube_b_dest)} (table1)"
            )

            # Déclare les 2 cubes comme obstacles → OMPL planifie en les évitant
            # (sinon le cube transporté heurte l'autre cube en chemin).
            self._step("adding the 2 cubes as obstacles in the scene")
            self._add_world_cube("cube_a", cube_a_pose)
            self._add_world_cube("cube_b", cube_b_pose)

            self._say("━━ Move 1/2: cube A (tag_1) table1 → table2 ━━")
            self._pick(cube_a_pose, attach_id="cube_a")
            self._place(cube_a_dest, detach_id="cube_a")

            self._say("━━ Move 2/2: cube B (tag_10) table2 → A's spot (table1) ━━")
            self._pick(cube_b_pose, attach_id="cube_b")
            self._place(cube_b_dest, detach_id="cube_b")

            self._say("━━ Returning to initial position (home) ━━")
            self._go_home()

            self._say("✅ Swap complete! (A ↔ B)")
            self.pub_done.publish(Bool(data=True))
            self._step("publishing /swap_done → color_detector")
            # Leave time for DDS to actually deliver the latched /swap_done sample to
            # color_detector_node BEFORE we tear the node down — otherwise the message
            # is dropped and the colour report never fires.
            time.sleep(3.0)
            self._say("🏁 Arm back at initial position — workflow complete. Shutting down node.")

        except Exception as e:
            self.get_logger().error(f"Swap failed: {e}")
            try:
                self._go_home()
            except Exception:
                pass
        finally:
            # Le swap est terminé (succès ou échec, bras ramené home) : on
            # interrompt proprement le node pour ne pas laisser tourner le
            # workflow indéfiniment.
            self._request_stop()

    def _request_stop(self) -> None:
        """Stoppe proprement le spin du node depuis le thread du swap."""
        if rclpy.ok():
            rclpy.shutdown()

    @staticmethod
    def _xy(p: Pose) -> str:
        return f"({p.position.x:.2f},{p.position.y:.2f})"

    # ──────────────────────────────────────────────────────────────────────────
    # Motion primitives
    # ──────────────────────────────────────────────────────────────────────────

    def _pick(self, cube_top: Pose, attach_id: str = None) -> None:
        """Pick — strictly sequential, each motion fully SETTLED before the next step.

        The joint_trajectory_controller reports "goal reached" within 0.1 rad while the
        arm is still drifting, so we wait SETTLE_TIME after every motion before touching
        the gripper — otherwise the gripper would act mid-motion.
        """
        pre   = self._at_z(cube_top, cube_top.position.z + PRE_GRASP_Z_OFFSET)
        grasp = self._at_z(cube_top, cube_top.position.z + GRASP_Z_OFFSET)
        lift  = self._at_z(cube_top, cube_top.position.z + LIFT_Z_OFFSET)

        self._step("pick 1/6: open gripper")
        self._gripper_cmd(GRIPPER_OPEN)

        self._step("pick 2/6: move above the cube (pre-grasp)")
        self._move_to_pose(pre, "pre-grasp")
        time.sleep(SETTLE_TIME)

        # Recentrage XY rectiligne : corrige le petit écart d'arrivée OMPL → la descente
        # qui suit est bien verticale et les doigts n'accrochent pas le cube. (Efficace.)
        self._step("pick 3/6: XY re-centring")
        self._move_cartesian(pre, "re-centre XY")
        time.sleep(SETTLE_TIME)

        self._step("pick 4/6: vertical descent onto the cube")
        self._move_cartesian(grasp, "grasp descent")
        time.sleep(SETTLE_TIME)        # bras TOTALEMENT arrêté avant de fermer

        self._step("pick 5/6: close gripper")
        self._gripper_cmd(GRIPPER_CLOSE)
        if attach_id:
            self._attach_cube(attach_id)

        self._step("pick 6/6: lift")
        self._move_cartesian(lift, "lift")
        time.sleep(SETTLE_TIME)

    def _place(self, target_top: Pose, detach_id: str = None) -> None:
        """Place — position at the release height above the (empty) spot, drop the cube,
        then LIFT STRAIGHT UP before anything else so the arm doesn't drag the cube it
        just released on its way to the next task.
        """
        release = self._at_z(target_top, target_top.position.z + PLACE_Z_OFFSET)
        retreat = self._at_z(target_top, target_top.position.z + PRE_GRASP_Z_OFFSET)

        self._step("place 1/3: position at release height (above the target)")
        self._move_to_pose(release, "place")
        time.sleep(SETTLE_TIME)        # bras TOTALEMENT arrêté avant d'ouvrir

        self._step("place 2/3: open gripper → release the cube")
        self._gripper_cmd(GRIPPER_RELEASE)
        if detach_id:
            self._detach_cube(detach_id, target_top)
        time.sleep(SETTLE_TIME)        # laisser le cube se poser

        self._step("place 3/3: vertical lift (clears the just-placed cube)")
        self._move_cartesian(retreat, "place retreat (Z up)")
        time.sleep(SETTLE_TIME)

    def _go_home(self) -> None:
        """Move arm to 'home' named state via joint constraints."""
        constraints = Constraints()
        for joint_name, value in HOME_JOINTS.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        self._send_arm_goal(constraints, label="home")

    def _gripper_cmd(self, position: float) -> None:
        """Send a position command to the gripper via JointGroupPositionController.

        The gripper spawns nearly closed (≈0.79 rad) and the knuckle joint is limited
        to 0.5 rad/s, so a full open (0.79 → 0.0) needs ≈1.6 s. We publish a few times
        (the controller may subscribe late → a single message can be dropped) and wait
        long enough for the fingers to actually REACH the commanded position before the
        arm moves on — otherwise the gripper is still half-closed over the cube.
        """
        msg = Float64MultiArray()
        msg.data = [float(position)]
        for _ in range(3):
            self._gripper_pub.publish(msg)
            time.sleep(0.1)
        state = "open" if position < 0.1 else "closed"
        self._step(f"gripper → {state} ({position:.2f} rad) [/gripper_position_controller]")
        time.sleep(2.0)  # full-open travel from the closed spawn pose + margin

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Wrap angle to [-π, π] — fixes KDL solutions that ignore joint limits."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _pose_to_joint_constraints(self, pose: Pose, label: str = "", seed_joints=None):
        """Call /compute_ik to convert a Cartesian pose to joint-space constraints.

        Joint-space planning (JointConstraints) is vastly more efficient than
        Cartesian constraint sampling — RRTConnect solves it in milliseconds.

        Angles are normalized to [-π, π] as an extra safety net. Returns a tuple
        (constraints, solution_dict).

        seed_joints: dict joint→angle to seed the IK search. When not given, the
        arm's CURRENT live configuration (/joint_states) is used so KDL returns the
        IK branch closest to where the arm actually is — otherwise it can return an
        elbow-flipped solution and the arm swings around (knocking the cube, and
        leaving it in a stretched config where the vertical retreat is infeasible).
        Falls back to the 'home' config only before the first /joint_states arrives.
        """
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("IK service /compute_ik not available.")

        seed_map = seed_joints or self._latest_joints or HOME_JOINTS
        seed = JointState()
        seed.name     = list(seed_map.keys())
        seed.position = list(seed_map.values())

        from moveit_msgs.msg import RobotState as RS
        seed_state = RS()
        seed_state.joint_state = seed

        ik_req = PositionIKRequest()
        ik_req.group_name    = "ir_arm"
        ik_req.ik_link_name  = EEF_LINK
        ik_req.robot_state   = seed_state
        ik_req.pose_stamped  = PoseStamped(
            header=Header(frame_id=WORLD_FRAME), pose=pose
        )
        ik_req.timeout       = rclpy.duration.Duration(seconds=5.0).to_msg()
        ik_req.avoid_collisions = True

        srv_req = GetPositionIK.Request()
        srv_req.ik_request = ik_req

        event = threading.Event()
        future = self._ik_client.call_async(srv_req)
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout=10.0):
            raise RuntimeError(f"IK service timed out for '{label}'.")

        resp = future.result()
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"No IK solution for '{label}' (code {resp.error_code.val})."
            )

        arm_joints = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        }
        constraints = Constraints()
        solution = {}
        for name, pos in zip(resp.solution.joint_state.name,
                             resp.solution.joint_state.position):
            if name not in arm_joints:
                continue
            normalized = self._normalize_angle(pos)
            solution[name] = normalized
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = normalized
            jc.tolerance_above = 0.02   # tighter goal sampling → more precise XY arrival
            jc.tolerance_below = 0.02
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)

        self.get_logger().debug(
            f"IK '{label}': "
            + ", ".join(f"{jc.joint_name}={jc.position:.3f}"
                        for jc in constraints.joint_constraints)
        )
        return constraints, solution

    def _move_to_pose(self, pose: Pose, label: str = "", seed_joints=None) -> dict:
        """Resolve a Cartesian pose to joint angles via IK, then plan in joint space.

        Returns the IK joint solution (dict joint→angle) so the caller can feed it
        as the seed for the next waypoint, keeping the arm in one IK branch.
        """
        constraints, solution = self._pose_to_joint_constraints(pose, label, seed_joints)
        self._send_arm_goal(constraints, label=label)
        return solution

    def _move_cartesian(self, target: Pose, label: str = "") -> None:
        """Move tool0 in a straight line (world frame) to `target`.

        Uses /compute_cartesian_path then executes via /execute_trajectory.
        avoid_collisions=False: this is a deliberate vertical approach/retreat right
        next to the table and cube, where we do NOT want RRT to detour — the straight
        motion is exactly what keeps the grasp aligned and stops the gripper pushing
        the cube.
        """
        if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/compute_cartesian_path not available.")

        # Slow (0.1 scaling): the joint_trajectory_controller has a tight 0.2 rad
        # path tolerance and a near-edge vertical descent momentarily lags it at
        # higher speed. Retried once — replanning from the (now closer) current
        # state and re-executing clears a marginal tolerance abort.
        last_err = None
        for attempt in range(2):
            req = GetCartesianPath.Request()
            req.header.frame_id   = WORLD_FRAME
            req.start_state.is_diff = True        # plan from the robot's current state
            req.group_name        = "ir_arm"
            req.link_name         = EEF_LINK
            req.waypoints         = [target]
            req.max_step          = 0.005         # 5 mm interpolation
            req.jump_threshold    = 0.0           # disabled
            req.avoid_collisions  = False
            req.max_velocity_scaling_factor     = 0.15
            req.max_acceleration_scaling_factor = 0.15

            event = threading.Event()
            future = self._cartesian_client.call_async(req)
            future.add_done_callback(lambda _: event.set())
            if not event.wait(timeout=15.0):
                raise RuntimeError(f"Cartesian path service timed out ('{label}').")

            resp = future.result()
            if resp.fraction < 0.9:
                raise RuntimeError(
                    f"Cartesian '{label}' only {resp.fraction:.0%} feasible "
                    f"(code {resp.error_code.val})."
                )
            try:
                self.get_logger().debug(
                    f"Cartesian '{label}': {resp.fraction:.0%} planned → executing"
                    + (f" (retry {attempt})." if attempt else ".")
                )
                self._execute_trajectory(resp.solution, label)
                return
            except RuntimeError as e:
                last_err = e
                self.get_logger().warn(
                    f"Cartesian '{label}' execution aborted ({e}); replanning from current state."
                )
                time.sleep(0.5)
        raise last_err

    def _execute_trajectory(self, robot_traj, label: str = "") -> None:
        """Execute a precomputed RobotTrajectory via the /execute_trajectory action."""
        if not self._exec_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/execute_trajectory action server not available.")

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = robot_traj

        sent = threading.Event()
        future = self._exec_client.send_goal_async(goal)
        future.add_done_callback(lambda _: sent.set())
        if not sent.wait(timeout=20.0):
            raise RuntimeError(f"ExecuteTrajectory send timed out ('{label}').")
        gh = future.result()
        if gh is None or not gh.accepted:
            raise RuntimeError(f"ExecuteTrajectory goal rejected ('{label}').")

        done = threading.Event()
        result_future = gh.get_result_async()
        result_future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=90.0):
            raise RuntimeError(f"ExecuteTrajectory result timed out ('{label}').")
        res = result_future.result()
        if res is None:
            raise RuntimeError(f"ExecuteTrajectory '{label}' returned no result (timeout?).")
        if res.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"ExecuteTrajectory '{label}' failed (code {res.result.error_code.val}).")
        self._step(f"cartesian trajectory '{label}' executed ✓ [/compute_cartesian_path]")

    # ──────────────────────────────────────────────────────────────────────────
    # Planning-scene attach / detach (so the cube follows the gripper in RViz)
    # ──────────────────────────────────────────────────────────────────────────

    def _attach_cube(self, cube_id: str) -> None:
        """Attach a cube-sized box to the EEF in the planning scene."""
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(CUBE_DIMENSIONS)
        pose = Pose()
        pose.position.z   = CUBE_CENTER_BELOW_TOOL0   # along tool0's +z (gripper axis)
        pose.orientation.w = 1.0

        obj = CollisionObject()
        obj.id = cube_id
        obj.header.frame_id = EEF_LINK
        obj.primitives = [box]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        aco = AttachedCollisionObject()
        aco.link_name = EEF_LINK
        aco.object = obj
        aco.touch_links = GRIPPER_TOUCH_LINKS
        self._apply_scene(attached=[aco], label=f"attach {cube_id}")

    def _add_world_cube(self, cube_id: str, top_pose: Pose) -> None:
        """Add (or move) a cube as a WORLD collision obstacle at `top_pose`.

        Declaring both cubes as obstacles lets OMPL plan transit paths AROUND the
        other cube — otherwise the carried cube sweeps straight through it.
        """
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(OBSTACLE_DIMENSIONS)   # inflated → extra clearance margin
        pose = Pose()
        pose.position.x = top_pose.position.x
        pose.position.y = top_pose.position.y
        pose.position.z = top_pose.position.z - CUBE_DIMENSIONS[2] / 2.0  # centre on the REAL cube
        pose.orientation.w = 1.0
        obj = CollisionObject()
        obj.id = cube_id
        obj.header.frame_id = WORLD_FRAME
        obj.primitives = [box]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        self._apply_scene(world=[obj], label=f"obstacle {cube_id} @ table")

    def _detach_cube(self, cube_id: str, place_top: Pose) -> None:
        """Detach the cube from the EEF and drop it back into the world at the place pose."""
        rm = CollisionObject()
        rm.id = cube_id
        rm.operation = CollisionObject.REMOVE
        aco = AttachedCollisionObject()
        aco.link_name = EEF_LINK
        aco.object = rm
        self._apply_scene(attached=[aco], label=f"detach {cube_id}")
        # Re-add it as a world obstacle at its new resting place.
        self._add_world_cube(cube_id, place_top)

    def _apply_scene(self, attached=None, world=None, label: str = "") -> None:
        """Best-effort planning-scene diff. A failure here must not abort the swap."""
        if not self._scene_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"/apply_planning_scene unavailable — skipping '{label}'.")
            return
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        if attached:
            ps.robot_state.attached_collision_objects = attached
        if world:
            ps.world.collision_objects = world

        req = ApplyPlanningScene.Request()
        req.scene = ps
        event = threading.Event()
        future = self._scene_client.call_async(req)
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout=5.0):
            self.get_logger().warn(f"/apply_planning_scene timed out — '{label}'.")
            return
        self._step(f"planning scene: {label} ✓ [/apply_planning_scene]")

    def _send_arm_goal(self, constraints: Constraints, label: str = "") -> None:
        """Build and send a MoveGroup action goal; wait for completion."""
        request = MotionPlanRequest()
        request.group_name                       = "ir_arm"
        request.num_planning_attempts            = 3
        request.allowed_planning_time            = 25.0  # mesh-collision checks are slow on WSL2
        request.max_velocity_scaling_factor      = 0.2   # transit speed
        request.max_acceleration_scaling_factor  = 0.1   # LOW accel → the carried cube doesn't slip out of the grip
        request.goal_constraints                 = [constraints]

        # Explicit workspace bounds (base_link frame) — avoids "planning volume not specified" warning
        ws = WorkspaceParameters()
        ws.header.frame_id = "base_link"
        ws.min_corner = Vector3(x=-2.0, y=-2.0, z=-0.1)
        ws.max_corner = Vector3(x=2.0,  y=2.0,  z=2.5)
        request.workspace_parameters = ws

        goal_msg = MoveGroup.Goal()
        goal_msg.request   = request
        goal_msg.planning_options.plan_only           = False
        goal_msg.planning_options.replan              = True
        goal_msg.planning_options.replan_attempts     = 3
        goal_msg.planning_options.replan_delay        = 2.0

        # Retry once on CONTROL_FAILED: the prof's joint_trajectory_controller has a
        # tight 0.2 rad path tolerance, and a long transit near the arm's reach limit
        # can exceed it by a hair (PATH_TOLERANCE_VIOLATED → CONTROL_FAILED). The goal
        # is an absolute JOINT target, so we just let the arm settle and replan from
        # the (now stationary) current state, then re-execute. The immediate internal
        # replan (replan=True) fails because the arm isn't settled yet (start tolerance
        # 0.01) — our dwell fixes that. Mirrors the retry in _move_cartesian.
        last_err = None
        for attempt in range(2):
            code = self._execute_arm_goal_once(goal_msg, label)
            if code == MoveItErrorCodes.SUCCESS:
                self._step(
                    f"arm → '{label}' reached ✓ [/move_action MoveIt]"
                    + (f" (retry {attempt})" if attempt else "")
                )
                return
            last_err = RuntimeError(f"MoveGroup failed for '{label}' (error code {code}).")
            if code != MoveItErrorCodes.CONTROL_FAILED:
                raise last_err   # planning / other errors won't be fixed by re-executing
            self.get_logger().warn(
                f"'{label}' aborted by controller (code {code}, path tolerance); "
                f"letting the arm settle and replanning."
            )
            time.sleep(SETTLE_TIME)   # let the controller hold & the arm come to rest
        raise last_err

    def _execute_arm_goal_once(self, goal_msg, label: str = "") -> int:
        """Send one MoveGroup goal, wait for completion, return its error code."""
        sent_event = threading.Event()
        future = self._arm_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda _: sent_event.set())
        if not sent_event.wait(timeout=20.0):
            raise RuntimeError(f"Arm goal send timed out ('{label}').")

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Arm goal rejected ('{label}').")

        done_event = threading.Event()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda _: done_event.set())
        if not done_event.wait(timeout=200.0):
            raise RuntimeError(f"Arm result timed out ('{label}').")

        result = result_future.result()
        if result is None:
            raise RuntimeError(f"MoveGroup returned no result for '{label}' (timeout?).")
        return result.result.error_code.val

    # ──────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _at_z(base: Pose, z: float) -> Pose:
        p = Pose()
        p.position.x  = base.position.x
        p.position.y  = base.position.y
        p.position.z  = z
        p.orientation = DOWNWARD_QUAT
        return p


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CubeSwapperNode()
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # _run_swap peut déjà avoir appelé rclpy.shutdown() en fin de workflow.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
