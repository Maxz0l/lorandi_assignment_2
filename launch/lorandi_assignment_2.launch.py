from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── 1. Simulation du prof ────────────────────────────────────────────────
    # Lance Gazebo (monde iaslab_ur.sdf), UR5, controllers ros2_control,
    # bridge caméra (rgb_camera/image + camera_info), et MoveIt! move_group.
    prof_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ir_launch"),
                "launch",
                "assignment_2.launch.py",
            ])
        )
    )

    # ── 2. Node AprilTag (C++ du prof, configuré par nous) ──────────────────
    # Souscrit à image_rect + camera_info → détecte les tags → publie /detections
    # et broadcast TF (tag_1, tag_10) dans le frame de la caméra.
    #
    # REMAPPINGS OBLIGATOIRES :
    #   image_rect   ← le bridge expose rgb_camera/image (pas image_rect)
    #   camera_info  ← le bridge expose rgb_camera/camera_info
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
            ("image_rect",   "rgb_camera/image"),
            ("camera_info",  "rgb_camera/camera_info"),
        ],
        output="screen",
    )

    # ── 3. Nos nodes Python ──────────────────────────────────────────────────

    # tag_detector_node : lit les détections AprilTag, utilise TF2 pour
    # convertir les poses du frame caméra vers le frame "world", et publie
    # les positions des cubes sous forme de PoseStamped sur /cube_poses.
    tag_detector = Node(
        package="lorandi_assignament_2",
        executable="tag_detector_node.py",
        name="tag_detector",
        output="screen",
    )

    # cube_swapper_node : machine à états qui pilote MoveIt! pour réaliser
    # le swap des deux cubes (pick → place intermédiaire → pick → place → home).
    # Attend que /cube_poses ait reçu les deux poses avant de démarrer.
    cube_swapper = Node(
        package="lorandi_assignament_2",
        executable="cube_swapper_node.py",
        name="cube_swapper",
        output="screen",
    )

    # color_detector_node : souscrit à rgb_camera/image, détecte la couleur
    # dominante de chaque cube via HSV, affiche le résultat en fin de swap.
    # (Points bonus +3)
    color_detector = Node(
        package="lorandi_assignament_2",
        executable="color_detector_node.py",
        name="color_detector",
        output="screen",
    )

    # ── Délai : attendre que Gazebo + MoveIt! soient prêts avant nos nodes ──
    # 10 s est un délai conservateur ; on peut réduire si la machine est rapide.
    our_nodes = TimerAction(
        period=10.0,
        actions=[apriltag_node, tag_detector, cube_swapper, color_detector],
    )

    return LaunchDescription([
        prof_launch,
        our_nodes,
    ])
