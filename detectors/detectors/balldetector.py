#!/usr/bin/env python3
#
#   balldetector.py
#
#   Detect the tennis balls with OpenCV.
#
#   Node:           /balldetector
#   Subscribers:    /usb_cam/image_raw          Source image
#   Publishers:     /balldetector/image_raw     Debug image
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

        # Finally, subscribe to the incoming image topic.  Using a
        # queue size of one means only the most recent message is
        # stored for the next subscriber callback.
        self.sub = self.create_subscription(
            Image, '/image_raw', self.process, 1)

        # Report.
        self.get_logger().info("Ball detector running...")

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()


    # Process the image (detect the ball).
    def process(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "rgb8")
        # self.get_logger().info(
        #     "Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (msg.width, msg.height, msg.step/msg.width, msg.encoding))

        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Help to determine the HSV range...
        if True:
            # Draw the center lines.  Note the row is the first dimension.
            (H, W, _) = frame.shape
            xc = W//2
            yc = H//2
            frame = cv2.line(frame, (xc,0), (xc,H-1), self.white, 1)
            frame = cv2.line(frame, (0,yc), (W-1,yc), self.white, 1)

            # Report the center HSV values.  Note the row comes first.
            self.get_logger().info(
                "HSV = (%3d, %3d, %3d)" % tuple(hsv[yc, xc]))

        
        # Threshold in Hmin/max, Smin/max, Vmin/max
        hsvLower = (20,  70,  60)
        hsvUpper = (30, 170, 255)
        binary = cv2.inRange(hsv, hsvLower, hsvUpper)

        # Erode and Dilate. Definitely adjust the iterations!
        binary = cv2.erode( binary, None, iterations=10)
        binary = cv2.dilate(binary, None, iterations=20)
        binary = cv2.erode( binary, None, iterations=10)


        # Find contours in the mask and initialize the current
        # (x, y) center of the ball
        (contours, hierarchy) = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw all contours on the original image for debugging.
        cv2.drawContours(frame, contours, -1, self.blue, 2)

        # Only proceed if at least one contour was found.  You may
        # also want to loop over the contours...
        if len(contours) > 0:
            # Pick the largest contour.
            contour = max(contours, key=cv2.contourArea)

            # Find the enclosing circle (convert to pixel values)
            ((xr, yr), radius) = cv2.minEnclosingCircle(contour)
            xr     = int(xr)
            yr     = int(yr)
            radius = int(radius)

            # Draw the circle (yellow) and centroid (red) on the
            # original image.
            cv2.circle(frame, (xr, yr), int(radius), self.yellow,  2)
            cv2.circle(frame, (xr, yr), 5,           self.red,    -1)

            # Report.
            self.get_logger().info(
                "Found Ball enclosed by radius %d about (%d,%d)" %
                (radius, xr, yr))

        # Convert the frame back into a ROS image and republish.
        self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

        # Alternatively, publish the black/white image.
        # self.pub.publish(self.bridge.cv2_to_imgmsg(binary))


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = DetectorNode('balldetector')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
