"""
lorandi_nodes.launch.py — Lance UNIQUEMENT nos nodes, dans un terminal propre.

But : un affichage lisible où l'on ne voit QUE nos nodes (chacun sa couleur), sans le
bruit de Gazebo / move_group / RViz / ros2_control. Ces derniers sont lancés par le
prof avec output="screen" : impossible de les faire taire depuis notre launch. On les
lance donc SÉPARÉMENT, dans un autre terminal :

    Terminal 1 (la simulation du prof, avec RViz — à minimiser) :
        ros2 launch ir_launch assignment_2.launch.py

    Terminal 2 (nos nodes, propre et coloré) :
        ros2 launch lorandi_assignament_2 lorandi_nodes.launch.py

Nos nodes attendent eux-mêmes que la sim soit prête (services move_group,
controller_manager, TF des tags), donc l'ordre de lancement est tolérant.

C'est la méthode de rendu : 2 terminaux. Le Terminal 1 est la sim du prof
TELLE QUELLE (elle inclut déjà RViz) ; le Terminal 2 lance tous nos nodes.
"""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _node(package, executable, name, **kwargs):
    """Node helper : sortie écran + TTY émulé (pour que les couleurs ANSI passent)."""
    return Node(
        package=package,
        executable=executable,
        name=name,
        output="screen",
        emulate_tty=True,
        **kwargs,
    )


def generate_launch_description():
    # Format de log console épuré : [niveau] [node] message
    clean_format = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}"
    )
    force_colors = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    apriltag_node = _node(
        "apriltag_ros", "apriltag_node", "apriltag",
        parameters=[PathJoinSubstitution([
            FindPackageShare("lorandi_assignament_2"), "config", "apriltag_params.yaml",
        ])],
        remappings=[
            ("image_rect",  "rgb_camera/image"),
            ("camera_info", "rgb_camera/camera_info"),
        ],
        # apriltag inonde de warnings de synchro → on ne garde que ses erreurs.
        arguments=["--ros-args", "--log-level", "apriltag:=ERROR"],
    )

    tag_detector  = _node("lorandi_assignament_2", "tag_detector_node.py",       "tag_detector")
    cube_swapper  = _node("lorandi_assignament_2", "cube_swapper_node.py",       "cube_swapper")
    color_detector = _node("lorandi_assignament_2", "color_detector_node.py",    "color_detector")
    gripper_spawner = _node("lorandi_assignament_2", "spawn_gripper_controller.py",
                            "gripper_controller_spawner")

    return LaunchDescription([
        clean_format,
        force_colors,
        gripper_spawner,
        apriltag_node,
        tag_detector,
        cube_swapper,
        color_detector,
    ])
