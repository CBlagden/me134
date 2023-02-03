#!/usr/bin/env python3
#
#   facedetector.py
#
#   Detect faces with OpenCV.
#
#   Node:           /facedetector
#   Subscribers:    /usb_cam/image_raw          Source image
#   Publishers:     /facedetector/image_raw     Debug image
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

        # Get face detector model from xml file.
        folder  = "/usr/share/opencv4/haarcascades/"
        faceXML = "haarcascade_frontalface_default.xml"
        eyeXML1 = "haarcascade_eye.xml"
        eyeXML2 = "haarcascade_eye_tree_eyeglasses.xml"

        self.faceCascade = cv2.CascadeClassifier(folder + faceXML)
        self.eyeCascade  = cv2.CascadeClassifier(folder + eyeXML1)

        # Finally, subscribe to the incoming image topic.  Using a
        # queue size of one means only the most recent message is
        # stored for the next subscriber callback.
        self.sub = self.create_subscription(
            Image, '/image_raw', self.process, 1)

        # Report.
        self.get_logger().info("Face detector running...")

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()


    # Process the image (detect the face).
    def process(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "rgb8")
        # self.get_logger().info(
        #     "Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (msg.width, msg.height, msg.step/msg.width, msg.encoding))

        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        # Convert to gray scale.
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # Grab the faces - the cascade detector returns the bounding boxes.
        faces = self.faceCascade.detectMultiScale(
            gray,
            scaleFactor = 1.2,
            minNeighbors = 5,
            minSize = (30,30),
            flags = cv2.CASCADE_SCALE_IMAGE)

        # Process the face: Draw the bounding box and look for eyes.
        for (x, y, w, h) in faces: 
            # Draw the bounding box.
            frame = cv2.rectangle(frame, (x, y), (x+w, y+h), self.green, 3)

            # Also look for eyes - only within the face!
            eyes = self.eyeCascade.detectMultiScale(gray[y:y+h,x:x+w])

            # Draw circles around the eyes :)
            for (xe,ye,we,he) in eyes:
                eye_center = (x + xe + we//2, y + ye + he//2)
                eye_radius = int(round((we + he)*0.25))
                frame = cv2.circle(frame, eye_center, eye_radius, self.red, 3)

        # Convert the frame back into a ROS image and republish.
        self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = DetectorNode('facedetector')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
