# lorandi_assignament_2 — UR5 + MoveIt! AprilTag-guided Cube Swap

Assignment 2 — Intelligent Robotics 2025/2026, University of Padua
Author: **Enzo Lorandi** — enzolorandi1234@gmail.com

- Demo video: https://youtu.be/ha9ZuNDaH0M

## Overview

This package swaps the positions of two cubes (0.06×0.06×0.10 m) with a UR5 arm and a
Robotiq 85 gripper, in a Gazebo simulation. Each cube has an AprilTag on its top face
(red → id 1, blue → id 10, 0.050 m, family 36h11). A fixed external RGB camera observes
the scene and publishes `/rgb_camera/image` and `/rgb_camera/camera_info`; its pose is
already on the TF tree.

The pipeline is: detect the tags, move the arm to each cube, do the swap safely, and
bring the arm back to its home pose. The `+3` colour-detection extra is also implemented.

The design is modular: perception, manipulation, colour reporting and gripper setup are
four independent ROS 2 nodes that only communicate through topics and the TF tree.

## Nodes

| Node | What it does | Main interfaces |
|------|--------------|-----------------|
| `tag_detector_node.py` | Turns the AprilTag detections into cube poses in the `world` frame (with a small per-tag calibration). | in: `/detections`, TF — out: `/cube_poses` (PoseArray, TRANSIENT_LOCAL) |
| `cube_swapper_node.py` | Core state machine: pick-and-place swap, then return home. | `/move_action`, `/compute_ik`, `/compute_cartesian_path`, `/execute_trajectory`, `/apply_planning_scene`, `/gripper_position_controller/commands` — out: `/swap_done` |
| `color_detector_node.py` | After the swap, projects each cube onto the image and prints its colour (red/blue). | in: `/rgb_camera/image`, `/rgb_camera/camera_info`, `/cube_poses`, `/swap_done` |
| `spawn_gripper_controller.py` | One-shot: loads and activates a `JointGroupPositionController` for the gripper at runtime. | `controller_manager` services |

## Package layout

```
launch/
  lorandi_nodes.launch.py   # launches my 4 nodes + apriltag_node (the one used for the demo)
  sim_norviz.launch.py      # dev helper: professor's sim WITHOUT RViz (faster on WSL2)
config/
  apriltag_params.yaml      # AprilTag node config (36h11, our remappings)
  gripper_controller.yaml   # gripper controller parameters
scripts/                    # the 4 node executables (see table above)
```

## Dependencies

- ROS 2 (target distribution: **Humble**; developed on Jazzy — see *Known limitations*)
- The course package `ir_2526` (simulation, UR5/MoveIt! config, AprilTag node) — used unmodified
- `apriltag_ros`, MoveIt! 2, `ros2_control` / `controller_manager`
- `cv_bridge` + OpenCV (colour-detection extra)

ROS dependencies are declared in `package.xml`.

## Build

From the workspace root:

```bash
colcon build --packages-select lorandi_assignament_2
source install/setup.bash
```

## Run (2 terminals)

The simulation (Gazebo + move_group + RViz) and my nodes are launched **separately**, so
my node logs stay readable (each node has its own colour) instead of being buried under the
Gazebo / MoveIt! output. The launch order does not matter: my nodes wait on their own until
the simulation is ready.

**Terminal 1 — the professor's simulation (with RViz):**
```bash
ros2 launch ir_launch assignment_2.launch.py
```

**Terminal 2 — my nodes + the AprilTag node:**
```bash
ros2 launch lorandi_assignament_2 lorandi_nodes.launch.py
```

The arm localises both tags, swaps the two cubes (two-move sequence, no buffer spot),
returns home, and `color_detector` prints each cube's colour at its final position.

### Dev tip

For faster iterations under WSL2 you can replace Terminal 1 with the RViz-less variant:
```bash
ros2 launch lorandi_assignament_2 sim_norviz.launch.py
```

## Known limitations

- **Grasp stability:** as the assignment warned, the Gazebo grasp is not perfectly stable;
  the 2 kg cube can sometimes slip when the gripper closes. The full pick-and-place is still
  implemented, and the planning-scene attach/detach keeps the RViz scene coherent even when
  this happens.
- **ROS distribution:** the deliverable targets Humble, but development ran on Jazzy. The
  `GripperActionController` is not available on Jazzy, so `spawn_gripper_controller.py`
  brings up a `JointGroupPositionController` instead and drives it with `Float64MultiArray`
  position commands. The distro-specific part is isolated in that single node.

## Use of AI

Anthropic's Claude (via the Claude Code assistant) was used as a development and writing
aid (debugging the MoveIt! action interface, the grasp geometry / TCP offset, the gripper
bring-up, and drafting comments/docs). All output was reviewed, tested and validated by the
author, who takes full responsibility for the final code.
