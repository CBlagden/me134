#!/usr/bin/env python3
#
#   depthincenter.py
#
#   Report the depth in the center of the image.
#
#   Node:           /depthincenter
#   Subscribers:    /camera/depth/image_rect_raw
#
import numpy as np

# ROS Imports
import rclpy

from rclpy.node         import Node
from sensor_msgs.msg    import Image


#
#   Node Class
#
class CustomNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Finally, subscribe to the incoming image topic.  Using a
        # queue size of one means only the most recent message is
        # stored for the next subscriber callback.
        self.sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.process, 1)

        # Report.
        self.get_logger().info(name + "running...")

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()


    # Process the depth image (detect the face).
    def process(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "16UC1")
        # self.get_logger().info(
        #     "Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (msg.width, msg.height, msg.step/msg.width, msg.encoding))

        # Extract the depth image information (distance in mm as uint16).
        width  = msg.width
        height = msg.height
        depth  = np.frombuffer(msg.data, np.uint16).reshape(height, width)

        # Report.
        col = width//2
        row = height//2
        d   = depth[row][col]
        print(row, col, d)
        self.get_logger().info(
            "Distance at (row %d, col %d) = %dmm" % (row, col, d))


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = CustomNode('depthincenter')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
