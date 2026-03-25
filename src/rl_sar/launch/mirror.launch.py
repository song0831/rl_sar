# Copyright (c) 2024-2025
# SPDX-License-Identifier: Apache-2.0
#
# mirror.launch.py — Launch Gazebo with the robot FIXED in the air for joint mirror testing.
#
# Usage:
#   ros2 launch rl_sar mirror.launch.py rname:=Qmini
#
# The robot's base_link is fixed to the world via a fixed joint, so it hangs
# in the air and you can observe each joint's motion independently.
# No rviz is launched to save resources.

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    rname = LaunchConfiguration("rname").perform(context)
    spawn_z = LaunchConfiguration("spawn_z")

    wname = "earth"  # simple flat world, no stairs
    rl_sar_share = get_package_share_directory("rl_sar")

    robot_name = ParameterValue(rname, value_type=str)
    gazebo_model_name = ParameterValue(rname + "_gazebo", value_type=str)

    # Build robot description with an extra fixed joint to world
    # This pins the robot's base_link in the air so it cannot fall or fly away
    robot_description = ParameterValue(
        Command([
            "bash -c \"",
            "xacro ",
            "$(ros2 pkg prefix ", rname, "_description)/share/", rname, "_description/xacro/robot.xacro",
            " | sed '/<robot /a\\",
            "  <link name=\\\"world\\\"/>\\n",
            "  <joint name=\\\"fixed_base_joint\\\" type=\\\"fixed\\\">\\n",
            "    <parent link=\\\"world\\\"/>\\n",
            "    <child link=\\\"base_link\\\"/>\\n",
            "    <origin xyz=\\\"0 0 0.5\\\" rpy=\\\"0 0 0\\\"/>\\n",
            "  </joint>",
            "'\"",
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": os.path.join(rl_sar_share, "worlds", wname + ".world"),
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "robot_model",
        ],
        output="screen",
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 0.0,
        }],
    )

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        parameters=[{
            "robot_name": robot_name,
            "gazebo_model_name": gazebo_model_name,
        }],
    )

    # No rviz — keep it lightweight for mirror testing

    return [
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_node,
        joy_node,
        param_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "rname",
            description="Robot name (e.g., Qmini)",
            default_value=TextSubstitution(text="Qmini"),
        ),
        DeclareLaunchArgument(
            "spawn_z",
            description="Spawn height (ignored — robot is fixed at 0.5m)",
            default_value=TextSubstitution(text="0.50"),
        ),
        OpaqueFunction(function=launch_setup),
    ])
