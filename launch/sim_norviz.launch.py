"""
sim_norviz.launch.py — La simulation du prof, mais SANS RViz (itérations dev plus rapides).

Identique à `ir_launch/assignment_2.launch.py` (Gazebo + ros2_control + MoveIt move_group),
sauf qu'on passe launch_rviz:=false → RViz n'est pas lancé. Sous WSL2, RViz est lourd ;
le retirer libère du CPU → la simulation tourne plus vite. On garde la fenêtre Gazebo
pour voir les cubes.

On ne modifie AUCUN fichier du prof : on ré-inclut simplement ses sous-launches
(ir_base + ir_movit) avec les bons arguments.

Usage (dev) :
    Terminal 1 :  ros2 launch lorandi_assignament_2 sim_norviz.launch.py
    Terminal 2 :  ros2 launch lorandi_assignament_2 lorandi_nodes.launch.py

Pour le rendu, utiliser la sim complète du prof avec RViz : ros2 launch ir_launch assignment_2.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = "ur5"

    controllers_file = PathJoinSubstitution(
        [FindPackageShare("ir_movit_config"), "config", "ros2_controllers.yaml"]
    )
    description_file = PathJoinSubstitution(
        [FindPackageShare("ir_desription"), "urdf", "ur_gripper.urdf"]
    )
    moveit_launch_file = PathJoinSubstitution(
        [FindPackageShare("ir_movit_config"), "launch", "ir_movit.launch.py"]
    )

    # Gazebo + UR + ros2_control (déjà sans RViz dans assignment_2)
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ir_movit_config"), "launch", "ir_base.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": "true",
            "controllers_file": controllers_file,
            "description_file": description_file,
            "launch_rviz": "false",
        }.items(),
    )

    # MoveIt move_group — SANS RViz (la seule différence avec assignment_2)
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": "false",
        }.items(),
    )

    return LaunchDescription([ur_control_launch, ur_moveit_launch])
