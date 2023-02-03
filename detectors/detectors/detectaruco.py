#!/usr/bin/env python3
#
#   detectaruco.py
#
#   Detect the ArUco marker with OpenCV.
#
#   Node:           /detectaruco
#   Subscribers:    /usb_cam/image_raw          Source image
#   Publishers:     /detectaruco/image_raw      Debug image
#
import cv2
import numpy as np

# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image


#
#  Detector Node Class
#
class DetectorNode(Node):
    # Pick some colors, assuming RGB8 encoding.
    red    = (255,   0,   0)
    green  = (  0, 255,   0)
    blue   = (  0,   0, 255)
    yellow = (255, 255,   0)
    white  = (255, 255, 255)

    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a publisher for the processed (debugging) image.
        # Store up to three images, just in case.
        self.pub = self.create_publisher(Image, name+'/image_raw', 3)

        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()

        # Set up the ArUco detector.  Use a dictionary with the
        # appropriate tags.  DICT_6x6_1000 has 1000 options (way too
        # many).  DICT_6x6_250 has 250 options.  DICT_6x6_50 is
        # probably enough for our projects...
        self.dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self.params = cv2.aruco.DetectorParameters_create()

        # Finally, subscribe to the incoming image topic.  Using a
        # queue size of one means only the most recent message is
        # stored for the next subscriber callback.
        self.sub = self.create_subscription(
            Image, '/image_raw', self.process, 1)

        # Report.
        self.get_logger().info("ArUco detector running...")

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()


    # Process the image (detect the aruco).
    def process(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "rgb8")
        # self.get_logger().info(
        #     "Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (msg.width, msg.height, msg.step/msg.width, msg.encoding))

        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        # Detect.
        (boxes, ids, rejected) = cv2.aruco.detectMarkers(
            frame, self.dict, parameters=self.params)

        # Loop over each marker: the list of corners and ID for this marker.
        if len(boxes) > 0:
            for (box, id) in zip(boxes, ids.flatten()):

                # The corners are top-left, top-right, bottom-right,
                # bottom-left of the original marker (not the bounding
                # box).  Each is a 2x1 numpy array of pixel
                # coordinates.  Their type is floating-point though
                # they contain integer values.
                (topLeft, topRight, bottomRight, bottomLeft) = box[0]
                center = (topLeft + bottomRight)/2

                # Draw the box around the marker.
                pts = [box.reshape(-1,1,2).astype(int)]
                cv2.polylines(frame, pts, True, self.green, 2)
            
                # Draw the center circle and write the ID there.
                ctr = tuple(center.astype(int))
                cv2.circle(frame, ctr, 4, self.red, -1)
                cv2.putText(frame, str(id), ctr,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.green, 2)

        # Convert the frame back into a ROS image and republish.
        self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = DetectorNode('detectaruco')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
