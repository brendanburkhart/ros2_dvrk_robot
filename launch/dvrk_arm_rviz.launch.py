from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchContext, LaunchDescription, Substitution

from typing import Text

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

class StringJoinSubstitution(Substitution):
    """Generate source list for a given arm"""

    def __init__(self, separator: str, strings: LaunchConfiguration):
        super().__init__()
        self.separator = separator
        self.__strings = strings

    def describe(self) -> Text:
        return "StringJointSubstitution({})".format(self.__strings.describe())

    def perform(self, context: LaunchContext) -> Text:
        strings_raw = self.__strings.perform(context)
        strings = strings_raw.split()
        return "{}".format(self.separator.join(strings))

def generate_launch_description():
    # Retrieve substitutions for all launch arguments
    arm_names = LaunchConfiguration("arms")
    dvrk_config = LaunchConfiguration("config")

    arm_names_text = StringJoinSubstitution("_", arm_names)

    # Generate default values
    dvrk_config_default = [
        PathJoinSubstitution([FindPackageShare("dvrk_config"), "share", "console", ""]),
        "console-",
        arm_names_text,
        "_KIN_SIMULATED.json",
    ]

    # Declare launch arguments
    arm_names_arg = DeclareLaunchArgument("arms")
    dvrk_config_arg = DeclareLaunchArgument("config", default_value=dvrk_config_default)

    rviz_config = [
        PathJoinSubstitution([FindPackageShare("dvrk_model"), "rviz", ""]),
        arm_names_text,
        ".rviz",
    ]

    # Declare nodes
    dvrk_console_node = Node(
        package="dvrk_robot",
        executable="dvrk_console_json",
        name=["dvrk_", arm_names_text, "_node"],
        arguments=["-j", dvrk_config],
    )

    arm1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dvrk_robot'),
                'launch',
                'dvrk_arm.launch.py'
            ])
        ]),
        launch_arguments={
            'arm': 'PSM1',
        }.items()
    )

    arm2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dvrk_robot'),
                'launch',
                'dvrk_arm.launch.py'
            ])
        ]),
        launch_arguments={
            'arm': 'PSM2',
        }.items(),
        # Only include node for second arm when specified
        condition=IfCondition(
            PythonExpression(
                ["len(\"", arm_names, "\".split()) > 1"]
            )
        )
    )

    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", rviz_config]
    )

    return LaunchDescription(
        [
            arm_names_arg,
            dvrk_config_arg,
            dvrk_console_node,
            arm1,
            arm2,
            rviz_node,
        ]
    )
