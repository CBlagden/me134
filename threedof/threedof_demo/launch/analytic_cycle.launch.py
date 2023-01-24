"""Launch the 134 HEBI demo

This instantiates the HEBI node and runs the simple 134 demo node.

"""
import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir("threedof_description"), "rviz/viewurdf.rviz")

    # Locate/load the robot's URDF file (XML).
    urdf = os.path.join(pkgdir("threedof_description"), "urdf/threedof.urdf")
    with open(urdf, "r") as file:
        robot_description = file.read()

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the robot_state_publisher.
    node_robot_state_publisher_ACTUAL = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    node_robot_state_publisher_COMMAND = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        remappings=[("/joint_states", "/joint_commands")],
    )

    # Configure a node for RVIZ
    node_rviz = Node(
        name="rviz",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rvizcfg],
        on_exit=Shutdown(),
    )

    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the hebi interface.
    node_hebi = Node(
        name="hebi",
        package="hebiros",
        executable="hebinode",
        output="screen",
        parameters=[
            {"family": "robotlab"},
            {"motors": ["7.3", "7.5", "7.2"]},
            {"joints": ["pan_joint", "middle_joint", "penguin_joint"]},
        ],
    )

    # Configure a node for the simple demo.
    node_pointnclick = Node(
        name="analytic_cycle",
        package="threedof_demo",
        executable="analytic_cycle",
        output="screen",
    )

    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST

    # Return the description, built as a python list.
    return LaunchDescription(
        [
            # start the other three nodes
            node_robot_state_publisher_ACTUAL,
            # node_robot_state_publisher_COMMAND,
            node_rviz,
            # Start the hebi and demo nodes.
            node_hebi,
            node_pointnclick,
        ]
    )
