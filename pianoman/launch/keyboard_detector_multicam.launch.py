"""Launch the USB camera node and keyboard detector.

This launch file is intended show how the pieces come together.
Please copy the relevant pieces.

"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription
from launch.actions                    import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure the USB camera node
    node_usbcam1 = Node(
        name       = 'usb_cam', 
        package    = 'usb_cam',
        executable = 'usb_cam_node_exe',
        namespace  = 'usb_cam',
        output     = 'screen',
        parameters = [{'video_device': '/dev/video0'},
                      {'framerate':    30.0},
                      {'pixel_format': 'yuyv'},
                      {'image_width':  1280},
                      {'image_height': 720},
                      {'camera_name':  'logitech1'}])

    node_usbcam2 = Node(
        name       = 'usb_cam', 
        package    = 'usb_cam',
        executable = 'usb_cam_node_exe',
        namespace  = 'usb_cam',
        output     = 'screen',
        parameters = [{'video_device': '/dev/video1'},
                      {'framerate':    30.0},
                      {'pixel_format': 'yuyv'},
                      {'image_width':  1280},
                      {'image_height': 720},
                      {'camera_name':  'logitech2'}])

    # Configure the keyboard detector node
    node_keyboard = Node(
        name       = 'keyboard',
        package    = 'pianoman',
        executable = 'keyboard_detector',
        output     = 'screen',
        remappings = [('/image_raw', '/usb_cam/image_raw'),
                      ('/camera_info', '/usb_cam/camera_info')])


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

    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([

        # Start the nodes.
        node_usbcam,
        # incl_realsense,
        node_keyboard,
    ])
