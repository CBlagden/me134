import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Parameters
    pkg_name = "pianoman_description"
    urdf_file_name = "urdf/pianoman.urdf"
    rviz_file_name = "rviz/viewurdf.rviz"

    urdf = os.path.join(get_package_share_directory(pkg_name), urdf_file_name)
    with open(urdf, "r") as infp:
        robot_desc = infp.read()

    rvizconfig = os.path.join(get_package_share_directory(pkg_name), rviz_file_name)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
        arguments=[urdf],
    )

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher'
    # )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig],
    )

    return LaunchDescription(
        [
            # joint_state_publisher_node,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
