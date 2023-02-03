"""Launch the USB camera node and face detector.

This launch file is intended show how the pieces come together.
Please copy the relevant pieces.

"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import Shutdown
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

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

    # Configure the face detector node
    node_facedetector = Node(
        name       = 'facedetector', 
        package    = 'detectors',
        executable = 'facedetector',
        output     = 'screen',
        remappings = [('/image_raw', '/usb_cam/image_raw')])


    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([

        # Start the nodes.
        node_usbcam,
        node_facedetector,
    ])
