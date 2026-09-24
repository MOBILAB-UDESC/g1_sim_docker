from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

POLICY_CONFIG = {
    "unitree_rl_lab": {
        "package": "g1_policy_runner",
        "executable": "policy_runner",
        "ready_topic": "/policy_runner/ready",
    },
    "gr00t_wbc": {
        "package": "gr00t_wbc_runner",
        "executable": "gr00t_wbc_runner",
        "ready_topic": "/gr00t_wbc_runner/ready",
    },
}


def generate_launch_description():
    policy_arg = DeclareLaunchArgument(
        "policy",
        default_value="gr00t_wbc",
        choices=list(POLICY_CONFIG.keys()) + ["none"],
        description="Walking policy to launch: 'gr00t_wbc' (legs+waist only), "
                    "'unitree_rl_lab' (g1_policy_runner, 29-dof), or 'none'.",
    )

    g1_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('g1_simulation'),
                'launch', 'unitree_g1.launch.py',
            )
        ),
        launch_arguments={'terrain': 'flat'}.items(),
    )

    policy_nodes = []
    for policy_name, cfg in POLICY_CONFIG.items():
        condition = LaunchConfigurationEquals("policy", policy_name)
        policy_nodes.append(
            Node(
                package=cfg["package"],
                executable=cfg["executable"],
                output="screen",
                condition=condition,
            )
        )
        policy_nodes.append(
            Node(
                package="g1_simulation",
                executable="pin_pelvis",
                output="screen",
                parameters=[{"ready_topic": cfg["ready_topic"]}],
                condition=condition,
            )
        )

    return LaunchDescription([policy_arg, g1_sim, *policy_nodes])