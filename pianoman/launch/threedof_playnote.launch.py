"""

Launch threedof local planner and play a note

"""
import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch import LaunchDescription
from launch.actions import Shutdown, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # Configure the USB camera node
    node_usbcam = Node(
        name       = 'usb_cam', 
        package    = 'usb_cam',
        executable = 'usb_cam_node_exe',
        namespace  = 'usb_cam',
        output     = 'screen',
        parameters = [{'video_device': '/dev/video0'},
                      {'framerate':    30.0},
                      {'pixel_format': 'yuyv'},
                      {'image_width':  640},
                      {'image_height': 480},
                      {'camera_name':  'logitech'}])


    # Use the standard realsense launch description.  But override
    # what we need.
    rsfile = os.path.join(pkgdir('realsense2_camera'), 'launch/rs_launch.py')

    # Profile is Width x Height x FPS.  0 is default.
    rsargs = {'camera_name':             'camera',  # camera unique name
              'depth_module.profile':    '0,0,0',   # depth W, H, FPS
              'rgb_camera.profile':      '0,0,0',   # color W, H, FPS
              'enable_color':            'true',    # enable color stream
              'enable_infra1':           'false',   # enable infra1 stream
              'enable_infra2':           'false',   # enable infra2 stream
              'enable_depth':            'false',    # enable depth stream
              'align_depth.enable':      'false',   # enabled aligned depth
              'pointcloud.enable':       'false',   # Turn on point cloud
              'allow_no_texture_points': 'true'}    # All points without texture

    incl_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsfile),
        launch_arguments=rsargs.items())

    # Custom Nodes
    node_statemachine = Node(
        name="state_machine",
        package="pianoman",
        executable="state_machine",
        output="screen",
    )
    node_localplanner = Node(
        name="local_planner",
        package="pianoman",
        executable="local_planner",
        output="screen",
    )
    # Configure the keyboard detector node
    node_keyboard = Node(
        name       = 'keyboard',
        package    = 'pianoman',
        executable = 'keyboard_detector',
        output     = 'screen',
        remappings = [('/image_raw', '/camera/color/image_raw'),
                      ('/camera_info', '/camera/color/camera_info')])

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
            # Start the state machine publisher
            node_statemachine,
            # Start up the local planner
            node_localplanner,

            # start the realsense
            incl_realsense,
            # start the keyboard
            node_keyboard,
        ]
    )
