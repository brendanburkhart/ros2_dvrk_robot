from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchContext, LaunchDescription, Substitution

from typing import Text

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

class ArmSourceListSubstitution(Substitution):
    """Generate source list for a given arm"""

    def __init__(self, arm_name: LaunchConfiguration):
        super().__init__()
        self.__arm_name = arm_name

    def describe(self) -> Text:
        return "ArmSourceList({})".format(self.__arm_name.describe())

    def perform(self, context: LaunchContext) -> Text:
        arm_name = self.__arm_name.perform(context)
        source_list = ["/{}/measured_js".format(arm_name)]
        if arm_name.startswith("PSM"):
            source_list.append("/{}/jaw/measured_js".format(arm_name))
        elif arm_name.startswith("MTM"):
            source_list.append("/{}/gripper/measured_js".format(arm_name))

        # launch_ros parses values from yaml, so source list needs to be yaml
        sources = ",".join(source for source in source_list)
        yaml =  "[{}]".format(sources)
        return yaml

def generate_launch_description():
    arm_name = LaunchConfiguration("arm")

    robot_model_default = [
        PathJoinSubstitution([FindPackageShare("dvrk_model"), "model", ""]),
        arm_name,
        ".urdf.xacro",
    ]

    #robot_model = LaunchConfiguration("model", default=robot_model_default)

    # Declare launch arguments
    #robot_model_arg = DeclareLaunchArgument("model", default_value=robot_model_default)

    # Use xacro to process robot model at substitution time
    # Can't happen until substitution time when we know robot_model
    robot_description = ParameterValue(
        Command(
            [FindExecutable(name="xacro"), " ", *robot_model_default]
        ),
        value_type=str,
    )

    # Declare nodes
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        namespace=[arm_name],
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"source_list": ArmSourceListSubstitution(arm_name)}],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        namespace=[arm_name],
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    return LaunchDescription(
        [
            #arm_name_arg,
            #robot_model_arg,
            joint_state_publisher_node,
            robot_state_publisher_node,
        ]
    )
