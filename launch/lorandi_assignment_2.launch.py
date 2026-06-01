from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── 1. Simulation du prof ────────────────────────────────────────────────
    # Lance Gazebo (iaslab_ur.sdf), UR5, ros2_control, bridge caméra et
    # MoveIt! move_group + RViz.
    prof_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ir_launch"),
                "launch",
                "assignment_2.launch.py",
            ])
        )
    )

    # ── 2. Node AprilTag (C++ du prof, notre config YAML) ───────────────────
    # Souscrit à image_rect + camera_info → détecte les tags → broadcast TF.
    # REMAPPINGS : le bridge expose rgb_camera/image (pas image_rect).
    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("lorandi_assignament_2"),
                "config",
                "apriltag_params.yaml",
            ])
        ],
        remappings=[
            ("image_rect",  "rgb_camera/image"),
            ("camera_info", "rgb_camera/camera_info"),
        ],
        # Le node apriltag inonde le terminal de warnings de synchro
        # image/camera_info → on ne garde que ses erreurs réelles.
        arguments=["--ros-args", "--log-level", "apriltag:=ERROR"],
        output="screen",
    )

    # ── 3. tag_detector_node ─────────────────────────────────────────────────
    # Lit les détections AprilTag, fait la conversion TF caméra→monde,
    # publie les poses des 2 cubes sur /cube_poses.
    tag_detector = Node(
        package="lorandi_assignament_2",
        executable="tag_detector_node.py",
        name="tag_detector",
        output="screen",
    )

    # ── 4. cube_swapper_node ─────────────────────────────────────────────────
    # Machine à états pick & place.
    # Le node charge lui-même la config MoveIt! via MoveItConfigsBuilder.
    cube_swapper = Node(
        package="lorandi_assignament_2",
        executable="cube_swapper_node.py",
        name="cube_swapper",
        output="screen",
    )

    # ── 5. color_detector_node ───────────────────────────────────────────────
    # Détecte la couleur des cubes via HSV après le swap. (Bonus +3 pts)
    color_detector = Node(
        package="lorandi_assignament_2",
        executable="color_detector_node.py",
        name="color_detector",
        output="screen",
    )

    # ── Gripper controller (JointGroupPositionController) ────────────────────
    # GripperActionController absent sur Jazzy → notre script Python charge
    # et active un JointGroupPositionController pour robotiq_85_left_knuckle_joint.
    gripper_spawner = Node(
        package="lorandi_assignament_2",
        executable="spawn_gripper_controller.py",
        name="gripper_controller_spawner",
        output="screen",
    )

    # ── Délai avant nos nodes : attendre Gazebo + MoveIt! prêts ─────────────
    delayed_gripper = TimerAction(period=10.0, actions=[gripper_spawner])
    our_nodes = TimerAction(
        period=15.0,
        actions=[apriltag_node, tag_detector, cube_swapper, color_detector],
    )

    return LaunchDescription([
        prof_launch,
        delayed_gripper,
        our_nodes,
    ])
