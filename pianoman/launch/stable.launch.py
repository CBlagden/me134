"""

Launch for all stable nodes

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
    rvizcfg = os.path.join(pkgdir("pianoman_description"), "rviz/viewurdf.rviz")

    # Locate/load the robot's URDF file (XML).
    urdf = os.path.join(pkgdir("pianoman_description"), "urdf/pianoman.urdf")
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
            {"motors": ["7.7", "8.3", "7.5", "7.4", "7.3", "7.1", "7.6", "8.4", "7.2"]},
            {"joints": ["base_joint", "R_gripper_joint", "R_pan_joint", "R_lower_joint", "R_upper_joint", "L_gripper_joint", "L_pan_joint", "L_lower_joint", "L_upper_joint"]},
        ],
    )

    # Custom Nodes
    # NONE

    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST

    # Return the description, built as a python list.
    return LaunchDescription(
        [
            # start the other three nodes
            node_robot_state_publisher_ACTUAL,
            # node_robot_state_publisher_COMMAND,
            node_rviz,
            # Start the hebi.
            node_hebi,
        ]
    )
