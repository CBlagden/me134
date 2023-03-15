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


# (height, width)
TOP_LEFT = (170, 350)
TOP_RIGHT = (170, 680)
BOTTOM_LEFT = (360, 350)
BOTTOM_RIGHT = (360, 680)

#
#   Node Class
#
class PersonDetectorNode(Node):
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
        self.get_logger().info(f"shape {np.shape(depth)}")

        TOP_LEFT = (350, 170)
        TOP_RIGHT = (680 ,170)
        BOTTOM_LEFT = (350, 360)
        BOTTOM_RIGHT = (680, 360)

        left_table_side = depth[TOP_LEFT[0], TOP_LEFT[1]:BOTTOM_LEFT[1]]
        # self.get_logger().info(f"Mean: {np.mean(left_table_side)}, Std: {np.std(left_table_side)}, min: {np.min(left_table_side)}, max: {np.max(left_table_side)}, quantiles: {np.percentile(left_table_side, [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100])}")

        front_table_side = depth[TOP_LEFT[0]:TOP_RIGHT[0], TOP_LEFT[1]]

        # right_table_side = depth[TOP_RIGHT[0], TOP_RIGHT[1]:BOTTOM_RIGHT[1]]

        if np.mean(left_table_side) < 1700:
            self.get_logger().warning("Someone has crossed the left side!")

        if np.mean(front_table_side) < 1700:
            self.get_logger().warning("Someone has crossed the front side!")

        # if np.mean(right_table_side) < 1700:
        #     self.get_logger().warning("Someone has crossed the right side!")

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
    node = PersonDetectorNode('depthincenter')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
