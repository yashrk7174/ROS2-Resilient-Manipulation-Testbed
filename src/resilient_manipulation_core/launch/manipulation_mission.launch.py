from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    object_pose_topic = LaunchConfiguration(
        "object_pose_topic"
    )

    resilience_enabled = LaunchConfiguration(
        "resilience_enabled"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "moveit_resources_panda"
        )
        .to_moveit_configs()
    )

    mission_node = Node(
        package="resilient_manipulation_core",
        executable="manipulation_mission",
        name="manipulation_mission",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "object_pose_topic": ParameterValue(
                    object_pose_topic,
                    value_type=str,
                ),
                "resilience_enabled": ParameterValue(
                    resilience_enabled,
                    value_type=bool,
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "object_pose_topic",
                default_value="/object_pose_estimate",
                description=(
                    "Pose topic consumed by the manipulation mission."
                ),
            ),

            DeclareLaunchArgument(
                "resilience_enabled",
                default_value="true",
                description=(
                    "Enable resilient estimate recovery and "
                    "bounded planning retry."
                ),
            ),

            mission_node,
        ]
    )
